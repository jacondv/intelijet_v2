# ui/scan_pipeline_worker.py
"""Runs the point-cloud convert/color/save/report pipeline off the GUI
thread, so App.on_cloud_received() no longer blocks the UI for the several
seconds (sometimes much longer) that coloring, VTK conversion, PLY save and
PDF report export used to take synchronously.

Mirrors the structure of ui.compare_cloud_worker.CompareWorker (a QThread
member owned by App, signals carry only plain data - no widgets). Unlike
CompareWorker (one-shot, a fresh instance per user action), this worker is
long-lived: App keeps ONE instance and calls submit() for every incoming
cloud message, because clouds arrive from ROS callbacks on their own
schedule, not from a single user click.

Threading contract:
  - submit() is called from the GUI thread only.
  - run() (and everything it calls) executes on the worker thread and must
    NEVER touch a QWidget, self.ui.*, or vtk_viewer. It only touches
    CloudPipelineService / ReportService / JobStore (all widget-free, see
    Phase 4) and emits signals - all UI updates happen in App's slots.
"""
import os
import threading

from PyQt5.QtCore import QThread, pyqtSignal


class ScanPipelineWorker(QThread):
    # (polydata, metadata dict) - GUI thread updates vtk_viewer + App state from this.
    cloud_ready = pyqtSignal(object, dict)
    # Ask the GUI thread to emit PPSCommand.START_COMPARE (keeps ui_send_cmd_signal
    # ownership/threading entirely on the GUI side).
    compare_requested = pyqtSignal()
    report_done = pyqtSignal(str)
    report_failed = pyqtSignal(str)
    # (source, message, level) - routed straight to NotificationCenter.push().
    notify = pyqtSignal(str, str, str)

    def __init__(self, cloud_pipeline, report_service, job_store, topics,
                 project_dir, thickness_default, tolerance_default, parent=None):
        super().__init__(parent)
        self.cloud_pipeline = cloud_pipeline
        self.report_service = report_service
        self.job_store = job_store
        self.topics = topics  # dict: pre_scan, post_scan, compared, compared_upsample, compared_manual, compared_upsample_manual
        self.project_dir = project_dir
        self.thickness_default = thickness_default
        self.tolerance_default = tolerance_default

        self._lock = threading.Lock()
        self._current_job = None
        self._pending_job = None

    def submit(self, job):
        """Thread-safe (call from the GUI thread). If idle, starts
        processing `job` immediately. If busy, `job` replaces whatever was
        queued - only the single newest pending job is kept, matching the
        "1-slot queue" design (no long backlog, no two workers racing on
        Open3D/VTK)."""
        with self._lock:
            if self.isRunning():
                if self._pending_job is not None:
                    self.notify.emit(
                        "cloud",
                        f"Superseded pending cloud on {self._pending_job['topic_name']} "
                        f"before it was processed",
                        "info",
                    )
                self._pending_job = job
                return
            self._current_job = job
        self.start()

    def run(self):
        while True:
            job = self._current_job
            try:
                self._process(job)
            except Exception as e:
                self.notify.emit(
                    "cloud", f"Scan pipeline error on {job.get('topic_name')}: {e}", "error"
                )
            with self._lock:
                if self._pending_job is None:
                    self._current_job = None
                    return
                self._current_job = self._pending_job
                self._pending_job = None

    # ------------------------------------------------------------------
    def _process(self, job):
        topics = self.topics
        msg = job["msg"]
        topic_name = job["topic_name"]

        o3d_cloud = self.cloud_pipeline.pointcloud2_to_o3d(msg)

        # define job_folder based on current selected job or manual compare mode
        if topic_name in (topics["compared_manual"], topics["compared_upsample_manual"]):
            jobs_folder = os.path.dirname(job["post_scan_path_snapshot"])
            job_number = jobs_folder.split("/")[-1]
        else:
            current_job = self.job_store.get_current_job()
            project_name = current_job.split("/")[0]
            job_number = current_job.split("/")[1]
            jobs_folder = os.path.join(self.project_dir, project_name, job_number)

        # 1. Assign Color
        if topic_name in (topics["compared"], topics["compared_upsample"],
                           topics["compared_manual"], topics["compared_upsample_manual"]):
            try:
                from ui.models.job_info import JobInfo
                job_info = JobInfo.load(jobs_folder)

                if job_info:
                    target_thickness = job_info.parameters.get("target_thickness", self.thickness_default)
                    tolerance = job_info.parameters.get("tolerance", self.tolerance_default)
                else:
                    target_thickness = self.thickness_default
                    tolerance = self.tolerance_default

                highlight_range = [target_thickness - tolerance, target_thickness + tolerance]
                o3d_cloud = self.cloud_pipeline.assign_colors_for_highlight(o3d_cloud, highlight_range)

            except Exception as e:
                self.notify.emit("cloud", f"Failed to color point cloud: {e}", "warning")

        # 2. Show pointcloud and Save Data
        metadata = None
        if topic_name in (topics["post_scan"], topics["pre_scan"], topics["compared"], topics["compared_manual"]):
            polydata = self.cloud_pipeline.to_vtk(o3d_cloud)

            from ui.models.file_name import generate_filename

            if not job["is_manual"]:
                filepath = generate_filename(
                    folder=jobs_folder,
                    job=job_number,
                    scan_type=topic_name,
                    ext="ply",
                )
            else:
                filepath = generate_filename(
                    folder="", job="", scan_type=topic_name, ext="ply",
                    filepath=job["post_scan_path_snapshot"],
                )

            f_name = None
            if polydata:
                f_name = self.cloud_pipeline.save_ply(o3d_cloud, filepath)

            metadata = {
                "report_name": None,
                # Original behavior: current_post_scan_path resets to "" for
                # ANY topic in this block whenever we were in manual-compare
                # mode (not only compared topics) - preserved exactly here.
                "reset_post_scan_path": job["is_manual"],
                "reset_is_manual_compare": topic_name in (topics["compared"], topics["compared_manual"]),
                "last_prescan_path": None,
            }
            if topic_name in (topics["compared"], topics["compared_manual"]):
                metadata["report_name"] = f_name
            if topic_name == topics["pre_scan"]:
                metadata["last_prescan_path"] = filepath

            self.cloud_ready.emit(polydata, metadata)

        # 3. Ask GUI thread to trigger Compare Cloud Action
        if topic_name == topics["post_scan"]:
            if job["auto_compare_on"]:
                self.compare_requested.emit()

        # 4. Export Report
        if topic_name in (topics["compared"], topics["compared_manual"]):
            if job["auto_compare_off"] or job["auto_report_off"]:
                return  # only export report when auto compare AND auto report are on

            # metadata["report_name"] is None if save_ply() failed - .replace()
            # then raises, same as the original App.on_cloud_received did; the
            # exception is caught by run()'s wrapper and surfaced via notify().
            filename = metadata["report_name"].replace(".ply", ".pdf")
            try:
                final_path = self.report_service.export(o3d_cloud, filename)
                self.report_done.emit(final_path)
            except Exception as e:
                self.report_failed.emit(str(e))
