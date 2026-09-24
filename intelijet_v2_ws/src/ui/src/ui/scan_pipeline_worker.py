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
    CloudPipelineService / JobStore (all widget-free, see Phase 4), the
    /export_report actionlib server (see report export section below), and
    emits signals - all UI updates happen in App's slots.
"""
import os
import threading
from datetime import datetime

import rospy
import actionlib
from PyQt5.QtCore import QThread, pyqtSignal

from shared.error_codes import lookup as lookup_error_code
from services.msg import ExportReportAction, ExportReportGoal

# Report export server must accept the goal and finish within this long -
# mirrors CompareWorker's SERVER_WAIT_TIMEOUT_SEC/RESULT_WAIT_TIMEOUT_SEC
# (ui/compare_cloud_worker.py) so a dropped connection/dead server surfaces
# as a clear failure instead of hanging this worker (and therefore ANY
# further cloud processing, since submit()'s 1-slot queue depends on run()
# returning) forever.
REPORT_SERVER_WAIT_TIMEOUT_SEC = 120
REPORT_RESULT_WAIT_TIMEOUT_SEC = 120


class ScanPipelineWorker(QThread):
    # (polydata, metadata dict) - GUI thread updates vtk_viewer + App state from this.
    cloud_ready = pyqtSignal(object, dict)
    # (prescan_path, postscan_path) - ask the GUI thread to run the same
    # CompareWorker/`/compare_cloud_manual` pipeline manual compare uses
    # (see App._start_compare), with explicit paths resolved here instead
    # of relying on a ROS param that only ever got updated once at UI
    # startup (see _resolve_prescan_path).
    compare_requested = pyqtSignal(str, str)
    report_done = pyqtSignal(str)
    report_failed = pyqtSignal(str)
    # (source, message, level) - routed straight to NotificationCenter.push().
    # (source, message, level, code) - code is "" when there isn't one
    # (see shared/error_codes.py).
    notify = pyqtSignal(str, str, str, str)

    def __init__(self, cloud_pipeline, job_store, topics,
                 project_dir, thickness_default, tolerance_default, parent=None):
        super().__init__(parent)
        self.cloud_pipeline = cloud_pipeline
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
                        "",
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
                entry = lookup_error_code("SCAN-009")
                self.notify.emit(
                    "cloud",
                    f"{entry['message']} ({job.get('topic_name')}): {e}",
                    entry["level"],
                    "SCAN-009",
                )
            with self._lock:
                if self._pending_job is None:
                    self._current_job = None
                    return
                self._current_job = self._pending_job
                self._pending_job = None

    # ------------------------------------------------------------------
    def _resolve_prescan_path(self, jobs_folder):
        """Prescan path to compare the just-finished Post-Scan against.
        Always the newest Pre-Scan .ply on disk in the job's own folder -
        avoids any risk of comparing against a stale in-memory path.
        """
        from ui.models.file_name import find_latest_prescan
        return find_latest_prescan(jobs_folder)

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
                entry = lookup_error_code("SCAN-010")
                self.notify.emit(
                    "cloud", f"{entry['message']}: {e}", entry["level"], "SCAN-010"
                )

        # 2. Show pointcloud and Save Data
        metadata = None
        if topic_name in (topics["post_scan"], topics["pre_scan"], topics["compared"], topics["compared_manual"]):
            polydata = self.cloud_pipeline.to_vtk(o3d_cloud)

            from ui.models.file_name import generate_filename

            # Auto compare and manual compare publish on different ROS topics
            # ("/cloud_compared" vs "/cloud_compared_manual") so a manual
            # result's topic_name still carries the "_manual" suffix here -
            # normalize it to the same type token as auto-compare so both
            # produce identically-named files (only prefix/timestamp/index/
            # scan_id differ, not the "type" segment).
            scan_type = topics["compared"] if topic_name == topics["compared_manual"] else topic_name

            if not job["is_manual"]:
                filepath = generate_filename(
                    folder=jobs_folder,
                    job=job_number,
                    scan_type=scan_type,
                    ext="ply",
                )
            else:
                filepath = generate_filename(
                    folder="", job="", scan_type=scan_type, ext="ply",
                    filepath=job["post_scan_path_snapshot"],
                )

            f_name = None
            if polydata:
                f_name = self.cloud_pipeline.save_ply(o3d_cloud, filepath)

            metadata = {
                "report_name": None,
                # For the 3D MAIN page's cloud-info label - the file this
                # displayed cloud was actually saved to (None if save_ply
                # failed above).
                "filepath": f_name,
                # Original behavior: current_post_scan_path resets to "" for
                # ANY topic in this block whenever we were in manual-compare
                # mode (not only compared topics) - preserved exactly here.
                "reset_post_scan_path": job["is_manual"],
                "reset_is_manual_compare": topic_name in (topics["compared"], topics["compared_manual"]),
                # True only when this cloud will actually go on to a report
                # export below - lets the GUI thread hold off switching to
                # the 3D MAIN page until report_done/report_failed fires,
                # instead of jumping there the instant the compared cloud
                # itself is ready (while the report PDF is still generating).
                "report_pending": (
                    topic_name in (topics["compared"], topics["compared_manual"])
                    and not (job["auto_compare_off"] or job["auto_report_off"])
                ),
            }
            if topic_name in (topics["compared"], topics["compared_manual"]):
                metadata["report_name"] = f_name

            self.cloud_ready.emit(polydata, metadata)

        # 3. Ask GUI thread to trigger Compare Cloud Action, with explicit
        # prescan/postscan paths (see compare_requested's docstring above).
        if topic_name == topics["post_scan"]:
            if job["auto_compare_on"]:
                if not (f_name and os.path.exists(f_name)):
                    self.notify.emit(
                        "cloud",
                        f"Auto-compare skipped: Post-Scan cloud was not saved to disk ({filepath})",
                        "error",
                        "",
                    )
                else:
                    prescan_path = self._resolve_prescan_path(jobs_folder)
                    if not prescan_path:
                        self.notify.emit(
                            "cloud",
                            f"Auto-compare skipped: no Pre-Scan file found for this job in {jobs_folder}",
                            "error",
                            "",
                        )
                    else:
                        self.compare_requested.emit(prescan_path, f_name)

        # 4. Export Report - via the /export_report actionlib server (see
        # services/report_export_action_server.py), not in-process: PDF
        # rendering (weasyprint/matplotlib/Open3D offscreen rendering) is
        # mostly-Python and CPU-heavy enough to hold the GIL for seconds,
        # which used to freeze the whole UI even from this background
        # QThread - every thread in one Python process shares one GIL. A
        # separate ROS node has its own interpreter/GIL, so it can't.
        if topic_name in (topics["compared"], topics["compared_manual"]):
            if job["auto_compare_off"] or job["auto_report_off"]:
                return  # only export report when auto compare AND auto report are on

            # metadata["report_name"] is None if save_ply() failed - .replace()
            # then raises, same as the original App.on_cloud_received did; the
            # exception is caught by run()'s wrapper and surfaced via notify().
            ply_path = metadata["report_name"]
            pdf_path = ply_path.replace(".ply", ".pdf")
            try:
                final_path = self._export_report(ply_path, pdf_path, jobs_folder)
                self.report_done.emit(final_path)
            except Exception as e:
                self.report_failed.emit(str(e))

    def _export_report(self, ply_path, pdf_path, jobs_folder):
        """Send an ExportReport goal to /export_report and block (this is
        already the background worker thread) until it finishes. Raises on
        failure/timeout so run()'s caller-side except handles it exactly
        like the old in-process ReportService.export() did."""
        from ui.models.job_info import JobInfo

        # site_name/job_name/date/time: parsed straight from the filename,
        # same convention ui.services.report_service.ReportService.export()
        # used to (job_folder's parent dir name = project/site name, "#"
        # separated basename = job_name#timestamp#type#scan_id).
        basename = os.path.basename(pdf_path)
        basename_parts = basename.split("#")
        site_name = os.path.basename(os.path.dirname(jobs_folder))
        job_name = basename_parts[0] if basename_parts else "Unknown"
        date_str, time_str = None, None
        if len(basename_parts) > 1:
            try:
                dt = datetime.strptime(basename_parts[1], "%Y%m%d_%H%M%S")
                date_str = dt.strftime("%d-%b-%Y")
                time_str = dt.strftime("%H:%M:%S")
            except ValueError:
                pass

        # Independent from step 1's target_thickness/tolerance lookup above
        # (not reused) so a failure there can't also take report export down
        # with it - same isolation the old standalone ReportService.export()
        # had.
        job_info = JobInfo.load(jobs_folder)
        if job_info:
            applied_thickness = job_info.parameters.get("target_thickness", self.thickness_default)
            tolerance = job_info.parameters.get("tolerance", self.tolerance_default)
        else:
            applied_thickness = self.thickness_default
            tolerance = self.tolerance_default

        client = actionlib.SimpleActionClient('/export_report', ExportReportAction)
        if not client.wait_for_server(rospy.Duration(REPORT_SERVER_WAIT_TIMEOUT_SEC)):
            raise RuntimeError("Report export server not available")

        goal = ExportReportGoal(
            compared_ply_path=ply_path,
            output_pdf_path=pdf_path,
            site_name=site_name,
            job_name=job_name,
            applied_thickness=applied_thickness,
            tolerance=tolerance,
            date=date_str or "",
            time=time_str or "",
        )
        client.send_goal(goal)

        if not client.wait_for_result(rospy.Duration(REPORT_RESULT_WAIT_TIMEOUT_SEC)):
            client.cancel_goal()
            raise RuntimeError(f"Report export timed out for {pdf_path}")

        result = client.get_result()
        if not result or not result.success:
            raise RuntimeError(f"Report export failed for {pdf_path}")

        return result.report_path
