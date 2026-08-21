# -*- coding: utf-8 -*-
"""ReportPageManager - the new merged "COMPARE & REPORT" screen (REPORT
side-nav tab). Replaces the old CompareManager/ReportViewManager modal
dialogs' functionality with a single always-visible page: pick a job
(defaults to the header's CURRENT JOB), see every point-cloud scan for
that job in one list, and per-row:
  - "3D View"     - load that cloud into the 3D MAIN viewport
  - "View Report" - open the matching PDF report in qpdfview, if one
                    exists (report filenames share the postscan/compared
                    file's timestamp+index+scan_id - see file_name.py's
                    FILENAME_TEMPLATE - so it's found by re-deriving that
                    name rather than storing a separate mapping)
  - "Delete"      - remove the file
Checking exactly 2 rows enables "Compare Selected", which runs the same
manual-compare pipeline the old Compare dialog used (CompareWorker).

App wires the two things this page can't reasonably do itself (touch
the 3D viewport, start a ROS action worker) via the on_view_3d/
on_start_compare callbacks passed into the constructor - this file has
no other dependency on App's internals, same spirit as ProjectManager's
job_store injection.

The old compare_dlg_manager.py/report_view_dlg_manager.py are left
completely untouched for now - this is reviewed first, then deleted in
a follow-up per the plan.
"""
import os
import subprocess

from PyQt5.QtWidgets import QWidget, QFrame, QMessageBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QCheckBox

from ui.report_page_ui import Ui_frm_ReportPage
from ui.models.file_name import is_sync_junk, parse_filename
from ui.services import project_repository as repo


def _find_matching_report(ply_path):
    """Given a prescan/postscan/compared .ply path, return the path of
    its matching PDF report if one exists, else None. Report files are
    written with the exact same job/timestamp/index/scan_id as the
    compared cloud they came from (scan_pipeline_worker.py passes the
    postscan's own filepath into generate_filename() to derive the
    compared cloud's name, and report_service.py just swaps .ply for
    .pdf on that same name) - so match by those 4 fields plus the type
    token containing "compared" (it's "cloud_compared_NN" or
    "cloud_compared_manual_NN" - never a bare "compared" prefix).
    """
    parsed = parse_filename(ply_path)
    folder = os.path.dirname(ply_path)
    try:
        candidates = os.listdir(folder)
    except OSError:
        return None
    for name in candidates:
        if not name.lower().endswith(".pdf") or is_sync_junk(name):
            continue
        p = parse_filename(name)
        if (p["timestamp"] == parsed["timestamp"] and p["index"] == parsed["index"]
                and p["scan_id"] == parsed["scan_id"] and "compared" in p["type"]):
            return os.path.join(folder, name)
    return None


class ReportPageManager(QWidget, Ui_frm_ReportPage):
    def __init__(self, job_store, on_view_3d, on_start_compare):
        super().__init__()
        self.setupUi(self)

        self.job_store = job_store
        self.on_view_3d = on_view_3d
        self.on_start_compare = on_start_compare

        self.current_project = None
        self.current_job = None
        self._checked_files = []  # ordered - at most 2

        self._populate_job_picker()
        self.cbbJobPicker.currentIndexChanged.connect(self._on_job_picker_changed)
        self.btnStartCompare.clicked.connect(self._start_compare)

        self._select_default_job()

    def showEvent(self, event):
        # Job list / active-jobs / files can all change while this page
        # isn't visible (JOB tab, another tablet via Syncthing) - refresh
        # whenever the operator actually switches to this tab instead of
        # only once at construction.
        super().showEvent(event)
        self._populate_job_picker()

    # =========================
    #      JOB SELECTION
    # =========================
    def _index_for(self, project, job):
        """QComboBox.findData() compares itemData via QVariant equality,
        which for an opaque Python object (our (project, job) tuples)
        falls back to identity rather than value equality in PyQt - it
        never matches a freshly-built tuple even when equal by value.
        Look it up by hand instead."""
        for i in range(self.cbbJobPicker.count()):
            if self.cbbJobPicker.itemData(i) == (project, job):
                return i
        return -1

    def _populate_job_picker(self):
        previous = (self.current_project, self.current_job)
        self.cbbJobPicker.blockSignals(True)
        self.cbbJobPicker.clear()
        for project in repo.list_projects():
            for job in repo.list_jobs(project):
                self.cbbJobPicker.addItem(f"{project}/{job}", (project, job))
        self.cbbJobPicker.blockSignals(False)

        if previous[0] is not None:
            idx = self._index_for(*previous)
            if idx >= 0:
                self.cbbJobPicker.setCurrentIndex(idx)
                self._checked_files = []
                self.render_files()
                return
        self._select_default_job()

    def _select_default_job(self):
        current = self.job_store.get_current_job()
        project, job = (None, None)
        if current:
            project, job = repo.parse_job_ref(current)
        idx = self._index_for(project, job) if project else -1
        if idx < 0:
            idx = 0 if self.cbbJobPicker.count() else -1
        if idx >= 0:
            self.cbbJobPicker.setCurrentIndex(idx)
        self._on_job_picker_changed(idx)

    def _on_job_picker_changed(self, index):
        data = self.cbbJobPicker.itemData(index) if index is not None and index >= 0 else None
        self.current_project, self.current_job = data if data else (None, None)
        self._checked_files = []
        self.render_files()

    # =========================
    #        RENDERING
    # =========================
    def _clear_dynamic_rows(self, layout):
        while layout.count() > 1:
            item = layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

    def _empty_label(self, text):
        lbl = QLabel(text)
        lbl.setObjectName("emptyStateLabel")
        return lbl

    def render_files(self):
        layout = self.filesListLayout
        self._clear_dynamic_rows(layout)

        if not self.current_project:
            layout.insertWidget(0, self._empty_label("No project/job available - create one in the JOB tab first."))
            self._update_compare_bar()
            return

        job_path = repo.job_path(self.current_project, self.current_job)
        try:
            files = sorted(
                f for f in os.listdir(job_path)
                if f.lower().endswith(".ply") and not is_sync_junk(f)
            )
        except OSError:
            files = []

        if not files:
            layout.insertWidget(0, self._empty_label("No point clouds for this job yet."))
        for filename in files:
            layout.insertWidget(layout.count() - 1, self._build_file_row(job_path, filename))

        self._update_compare_bar()

    def _build_file_row(self, job_path, filename):
        filepath = os.path.join(job_path, filename)
        parsed = parse_filename(filename)

        row = QFrame()
        row.setObjectName("fileRow")
        h = QHBoxLayout(row)
        h.setContentsMargins(20, 14, 20, 14)
        h.setSpacing(14)

        chk = QCheckBox()
        chk.setChecked(filepath in self._checked_files)
        chk.toggled.connect(lambda checked, p=filepath: self._on_row_checked(p, checked))
        h.addWidget(chk)

        info = QVBoxLayout()
        info.setSpacing(4)
        title = QLabel(f"{parsed['type'].upper()} #{parsed['index']} (SCAN {parsed['scan_id']})")
        title.setObjectName("fileRowTitle")
        info.addWidget(title)
        subtext = QLabel(f"Timestamp: {parsed['timestamp']}")
        subtext.setObjectName("fileRowSubtext")
        info.addWidget(subtext)
        h.addLayout(info)
        h.addStretch(1)

        btn_view3d = QPushButton("3D View")
        btn_view3d.setProperty("cssClass", "rowActionBtn")
        btn_view3d.clicked.connect(lambda _checked, p=filepath: self.on_view_3d(p))
        h.addWidget(btn_view3d)

        report_path = _find_matching_report(filepath)
        btn_report = QPushButton("View Report")
        btn_report.setProperty("cssClass", "rowActionBtn")
        btn_report.setEnabled(report_path is not None)
        btn_report.clicked.connect(lambda _checked, p=report_path: self._open_report(p))
        h.addWidget(btn_report)

        btn_delete = QPushButton("Delete")
        btn_delete.setProperty("cssClass", "rowDangerBtn")
        btn_delete.clicked.connect(lambda _checked, p=filepath: self._delete_file(p))
        h.addWidget(btn_delete)

        return row

    # =========================
    #    COMPARE SELECTION
    # =========================
    def _on_row_checked(self, filepath, checked):
        if checked:
            if filepath not in self._checked_files:
                if len(self._checked_files) >= 2:
                    # Third checkbox checked - refresh drops the oldest
                    # selection instead of silently allowing 3.
                    self._checked_files.pop(0)
                self._checked_files.append(filepath)
        else:
            if filepath in self._checked_files:
                self._checked_files.remove(filepath)
        self.render_files()

    def _update_compare_bar(self):
        count = len(self._checked_files)
        self.compareBarLabel.setText(f"Select 2 point clouds to compare ({count}/2 selected)")
        self.btnStartCompare.setEnabled(count == 2)

    def _start_compare(self):
        if len(self._checked_files) != 2:
            return
        # Same rule the old CompareManager.get_result() used: whichever
        # selected file has "pre" in its name is treated as the prescan,
        # the other as the postscan - regardless of its actual type.
        files = sorted(self._checked_files, key=lambda p: 0 if "pre" in os.path.basename(p).lower() else 1)
        self._checked_files = []
        self.render_files()
        self.on_start_compare(files[0], files[1])

    # =========================
    #    REPORT / DELETE
    # =========================
    def _open_report(self, pdf_path):
        if not pdf_path or not os.path.exists(pdf_path):
            QMessageBox.warning(self, "Not Found", "Report file not found.")
            return
        subprocess.Popen(["qpdfview", "--unique", pdf_path])

    def _delete_file(self, filepath):
        if QMessageBox.question(
            self, "Confirm", f"Delete file:\n{os.path.basename(filepath)} ?",
            QMessageBox.Yes | QMessageBox.No
        ) != QMessageBox.Yes:
            return
        try:
            if os.path.exists(filepath):
                os.remove(filepath)
        except OSError as e:
            QMessageBox.critical(self, "Error", f"Failed to delete file:\n{e}")
            return
        if filepath in self._checked_files:
            self._checked_files.remove(filepath)
        self.render_files()
