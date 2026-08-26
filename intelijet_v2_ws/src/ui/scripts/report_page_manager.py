# -*- coding: utf-8 -*-
"""ReportPageManager - the new merged "Reports" screen (REPORT side-nav
tab). Replaces the old CompareManager/ReportViewManager modal dialogs'
functionality with a single always-visible page: pick a project then a
job (or jump straight to the header's CURRENT JOB), see every scan for
that job grouped into segments (one Pre-Scan + the Post-Scans taken
against it, i.e. sharing its scan_id), and per-row:
  - "3D View"  - load that raw scan cloud into the 3D MAIN viewport
  - "Heatmap"  - (Post-Scan rows only) find the compared/result cloud
                 produced from this scan and load THAT into the 3D MAIN
                 viewport instead
  - "Report"   - open the matching PDF report in qpdfview, if one
                 exists, and raise/activate its window
  - "Delete"   - remove the file
Compared-cloud files themselves are not listed as rows (too much noise -
they're reached via "Heatmap" on their source Post-Scan row).
Checking exactly 2 rows enables "Compare Selected", which runs the same
manual-compare pipeline the old Compare dialog used (CompareWorker).

App wires the two things this page can't reasonably do itself (touch
the 3D viewport, start a ROS action worker) via the on_view_3d/
on_start_compare callbacks passed into the constructor - this file has
no other dependency on App's internals, same spirit as ProjectManager's
job_store injection.

The old compare_dlg_manager.py/report_view_dlg_manager.py/compare_dlg_ui.py
(and the header's Compare/Report buttons that opened them) have been
deleted - this page fully replaces them.
"""
import os
import subprocess

from PyQt5.QtWidgets import (
    QWidget, QFrame, QMessageBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton,
)

from ui.report_page_ui import Ui_frm_ReportPage
from ui.models.file_name import is_sync_junk, parse_filename
from ui.services import project_repository as repo
from ui.widgets.picker_item_delegate import PickerItemDelegate


def _matches(parsed, candidate_parsed):
    """A compared-cloud/report file is considered "produced from" a given
    scan when it shares the same scan_id + index - NOT timestamp: auto
    compare re-stamps a fresh timestamp when it writes the compared
    cloud (only manual compare reuses the source file's exact timestamp,
    see scan_pipeline_worker.py's filepath= branch), so timestamp can't
    be part of the identity check here.
    """
    return (candidate_parsed["scan_id"] == parsed["scan_id"]
            and candidate_parsed["index"] == parsed["index"])


def _find_latest(folder, parsed, ext, type_predicate):
    try:
        candidates = os.listdir(folder)
    except OSError:
        return None
    matches = []
    for name in candidates:
        if not name.lower().endswith(f".{ext}") or is_sync_junk(name):
            continue
        p = parse_filename(name)
        if type_predicate(p["type"]) and _matches(parsed, p):
            path = os.path.join(folder, name)
            matches.append(path)
    if not matches:
        return None
    matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return matches[0]


def _find_matching_compared(ply_path):
    """Given a Post-Scan .ply path, return the newest compared/result
    cloud produced from it, if one exists."""
    parsed = parse_filename(ply_path)
    folder = os.path.dirname(ply_path)
    return _find_latest(folder, parsed, "ply", lambda t: "compared" in t)


def _find_matching_report(ply_path):
    """Given any scan .ply path, return the path of its matching PDF
    report if one exists (report_service.py names the report file by
    swapping .ply for .pdf on the compared cloud's own name, so we
    match the same way _find_matching_compared does: scan_id + index,
    on a file whose type contains "compared")."""
    parsed = parse_filename(ply_path)
    folder = os.path.dirname(ply_path)
    return _find_latest(folder, parsed, "pdf", lambda t: "compared" in t)


def _is_prescan(scan_type):
    return "pre" in scan_type.lower()


def _find_matching_report_for_segment(job_path, scan_id):
    """For a Pre-Scan row: the newest report PDF belonging to this
    segment (same scan_id), regardless of index - a Pre-Scan's own index
    carries no real relationship to any compared cloud/report (see
    _build_file_row), but every Post-Scan/report generated against this
    Pre-Scan does share its scan_id, so that's the right key here."""
    try:
        candidates = os.listdir(job_path)
    except OSError:
        return None
    matches = []
    for name in candidates:
        if not name.lower().endswith(".pdf") or is_sync_junk(name):
            continue
        p = parse_filename(name)
        if "compared" in p["type"] and p["scan_id"] == scan_id:
            matches.append(os.path.join(job_path, name))
    if not matches:
        return None
    matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return matches[0]


class ReportPageManager(QWidget, Ui_frm_ReportPage):
    def __init__(self, job_store, on_view_3d, on_start_compare):
        super().__init__()
        self.setupUi(self)

        self.job_store = job_store
        self.on_view_3d = on_view_3d
        self.on_start_compare = on_start_compare

        self.current_project = None
        self.current_job = None
        self.current_segment_filter = None  # None = "All Segments"
        self._checked_files = []  # ordered - at most 2
        self._all_projects = []  # full, unfiltered - see _load_project_and_job
        self._all_jobs = []

        self._enlarge_popup_items(self.cbbProjectPicker)
        self._enlarge_popup_items(self.cbbJobPicker)
        self._enlarge_popup_items(self.cbbSegmentFilter)

        self._wire_search_filter(self.searchProjectBox, self.cbbProjectPicker, lambda: self._all_projects, self._on_project_picker_changed)
        self._wire_search_filter(self.searchJobBox, self.cbbJobPicker, lambda: self._all_jobs, self._on_job_picker_changed)

        self.cbbProjectPicker.currentIndexChanged.connect(self._on_project_picker_changed)
        self.cbbJobPicker.currentIndexChanged.connect(self._on_job_picker_changed)
        self.cbbSegmentFilter.currentIndexChanged.connect(self._on_segment_filter_changed)
        self.btnCurrentJob.clicked.connect(self._jump_to_current_job)
        self.btnStartCompare.clicked.connect(self._start_compare)

        self._jump_to_current_job()

    def showEvent(self, event):
        # Job list / active-jobs / files can all change while this page
        # isn't visible (JOB tab, another tablet via Syncthing) - refresh
        # whenever the operator actually switches to this tab instead of
        # only once at construction.
        super().showEvent(event)
        self._populate_project_picker()

    # =========================
    #   PROJECT / JOB SELECTION
    # =========================
    def _enlarge_popup_items(self, combo):
        """Real per-row height/selection styling for the popup list -
        see PickerItemDelegate for why this has to be a delegate rather
        than QSS ::item rules."""
        combo.view().setItemDelegate(PickerItemDelegate(combo))

    def _wire_search_filter(self, search_box, combo, get_all_items, on_changed):
        """The Project/Job pickers are plain selection-only comboboxes
        (see report_page_ui.py's _build_searchable_field) - each has its
        own dedicated search QLineEdit above it instead of being made
        editable itself, so typing to filter and picking an item stay
        two clearly separate actions (an editable combobox's internal
        QLineEdit was also the thing that turned out to not reliably
        raise the on-screen keyboard). Typing here just narrows what
        combo currently lists; if that causes the selection itself to
        change (the previously-selected item got filtered out), run
        on_changed same as a real user pick would.
        """
        def apply_filter():
            previous = combo.currentData()
            self._populate_picker(combo, get_all_items(), previous, search_box.text())
            if combo.currentData() != previous:
                on_changed(combo.currentIndex())
        search_box.textChanged.connect(lambda _text: apply_filter())

    def _populate_picker(self, combo, all_items, desired_value, filter_text=""):
        """Repopulate combo from all_items, narrowed to filter_text (case
        -insensitive substring) if given, and select desired_value if
        it's still present (else the first item, if any) - signals
        blocked throughout so this never itself fires currentIndexChanged;
        callers that need to react to a resulting selection change do so
        explicitly (see _wire_search_filter)."""
        filter_text = (filter_text or "").strip().lower()
        items = [v for v in all_items if filter_text in v.lower()] if filter_text else list(all_items)
        combo.blockSignals(True)
        combo.clear()
        for v in items:
            combo.addItem(v, v)
        idx = combo.findData(desired_value) if desired_value else -1
        if idx < 0:
            idx = 0 if combo.count() else -1
        if idx >= 0:
            combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    def _populate_project_picker(self):
        """Only called on user-driven changes (user typed/picked a project)
        - never for programmatic init/refresh, see _load_project_and_job.
        """
        self._load_project_and_job(self.current_project, self.current_job)

    def _on_project_picker_changed(self, index):
        # User picked a different project from the dropdown - reload jobs
        # for it and default to that project's first job.
        project = self.cbbProjectPicker.itemData(index) if index is not None and index >= 0 else None
        self._load_project_and_job(project, None)

    def _on_job_picker_changed(self, index):
        # User picked a different job for the already-selected project.
        self.current_job = self.cbbJobPicker.itemData(index) if index is not None and index >= 0 else None
        self._checked_files = []
        self.current_segment_filter = None
        self.render_files()

    def _on_segment_filter_changed(self, index):
        self.current_segment_filter = self.cbbSegmentFilter.itemData(index) if index is not None and index >= 0 else None
        self.render_files(rebuild_filter=False)

    def _jump_to_current_job(self):
        current = self.job_store.get_current_job()
        project, job = repo.parse_job_ref(current) if current else (None, None)
        self._load_project_and_job(project, job)

    def _load_project_and_job(self, project, job):
        """Refresh both comboboxes' full item list and selection in one
        deterministic pass, then render - used for every non-interactive
        (re)population: construction, showEvent refresh, and "Current
        Job". Signals are blocked throughout (via _populate_picker) so
        this never itself fires currentIndexChanged; only genuine user
        picks go through the signal-connected _on_..._changed slots.
        Re-applies whatever each search box currently has typed, so a
        background refresh doesn't silently clear an active filter.
        """
        self._all_projects = repo.list_projects()
        self._populate_picker(self.cbbProjectPicker, self._all_projects, project, self.searchProjectBox.text())
        self.current_project = self.cbbProjectPicker.currentData()

        self._all_jobs = repo.list_jobs(self.current_project) if self.current_project else []
        self._populate_picker(self.cbbJobPicker, self._all_jobs, job, self.searchJobBox.text())
        self.current_job = self.cbbJobPicker.currentData()

        self._checked_files = []
        self.current_segment_filter = None
        self.render_files()

    # =========================
    #        RENDERING
    # =========================
    def _clear_dynamic_rows(self, layout):
        while layout.count() > 1:
            item = layout.takeAt(0)
            widget = item.widget()
            if widget:
                # Removing from the layout alone doesn't stop it from
                # painting - it stays a visible (just unmanaged) child of
                # filesScrollContent until deleteLater()'s deferred
                # deletion actually runs, which briefly overlapped the
                # next segment card's widgets when re-rendering
                # back-to-back (e.g. picking a new project). hide() takes
                # effect immediately, deleteLater() still reclaims it.
                widget.hide()
                widget.deleteLater()

    def _empty_label(self, text):
        lbl = QLabel(text)
        lbl.setObjectName("emptyStateLabel")
        return lbl

    def render_files(self, rebuild_filter=True):
        layout = self.filesListLayout
        self._clear_dynamic_rows(layout)

        if not self.current_project or not self.current_job:
            if rebuild_filter:
                self._populate_segment_filter({})
            layout.insertWidget(0, self._empty_label("No project/job available - create one in the JOB tab first."))
            self._update_compare_bar()
            return

        job_path = repo.job_path(self.current_project, self.current_job)
        try:
            all_files = sorted(
                f for f in os.listdir(job_path)
                if f.lower().endswith(".ply") and not is_sync_junk(f)
            )
        except OSError:
            all_files = []

        # Compared/result clouds are reached via "Heatmap" on their
        # source Post-Scan row, not listed as their own rows.
        parsed_files = [(f, parse_filename(f)) for f in all_files]
        parsed_files = [(f, p) for f, p in parsed_files if "compared" not in p["type"]]

        # Group into segments: one Pre-Scan opens a segment, every
        # Post-Scan up to (not including) the next Pre-Scan shares its
        # scan_id and belongs to that same segment.
        segments = {}
        for f, p in parsed_files:
            segments.setdefault(p["scan_id"], []).append((f, p))

        if rebuild_filter:
            self._populate_segment_filter(segments)

        if not segments:
            layout.insertWidget(0, self._empty_label("No pre-scan/post-scan point clouds for this job yet."))
            self._update_compare_bar()
            return

        shown_scan_ids = (
            [self.current_segment_filter] if self.current_segment_filter is not None and self.current_segment_filter in segments
            else sorted(segments.keys())
        )

        for scan_id in shown_scan_ids:
            entries = segments[scan_id]
            entries.sort(key=lambda fp: (0 if _is_prescan(fp[1]["type"]) else 1, fp[1]["index"]))
            layout.insertWidget(layout.count() - 1, self._build_segment_card(job_path, scan_id, entries))

        self._update_compare_bar()

    def _populate_segment_filter(self, segments):
        previous = self.current_segment_filter
        self.cbbSegmentFilter.blockSignals(True)
        self.cbbSegmentFilter.clear()
        self.cbbSegmentFilter.addItem("All Segments", None)
        for scan_id in sorted(segments.keys()):
            self.cbbSegmentFilter.addItem(f"Segment - Scan {scan_id}", scan_id)
        idx = 0
        if previous is not None:
            for i in range(self.cbbSegmentFilter.count()):
                if self.cbbSegmentFilter.itemData(i) == previous:
                    idx = i
                    break
            else:
                previous = None  # previously-selected segment no longer exists
        self.current_segment_filter = previous
        self.cbbSegmentFilter.setCurrentIndex(idx)
        self.cbbSegmentFilter.blockSignals(False)

    def _build_segment_card(self, job_path, scan_id, entries):
        prescan_entry = next((p for f, p in entries if _is_prescan(p["type"])), None)
        post_count = sum(1 for f, p in entries if not _is_prescan(p["type"]))

        card = QFrame()
        card.setObjectName("segmentCard")
        v = QVBoxLayout(card)
        v.setContentsMargins(20, 18, 20, 20)
        v.setSpacing(14)

        header = QLabel(f"Segment - Scan {scan_id}")
        header.setObjectName("segmentHeader")
        v.addWidget(header)

        subtitle = (f"Pre-Scan at {prescan_entry['timestamp']}  ·  {post_count} Post-Scan(s)"
                    if prescan_entry else f"No Pre-Scan recorded  ·  {post_count} Post-Scan(s)")
        sub = QLabel(subtitle)
        sub.setObjectName("segmentSubtext")
        v.addWidget(sub)

        for filename, parsed in entries:
            v.addWidget(self._build_file_row(job_path, filename, parsed))

        return card

    def _build_file_row(self, job_path, filename, parsed):
        filepath = os.path.join(job_path, filename)
        is_prescan = _is_prescan(parsed["type"])

        row = QFrame()
        row.setObjectName("fileRow")
        h = QHBoxLayout(row)
        # Vertical margin kept small (not zero - the row still needs a
        # little breathing room from its neighbors) so the row's height
        # is mostly however tall the buttons' own (generous) padding
        # makes them, not fought over between the two.
        h.setContentsMargins(20, 8, 20, 8)
        h.setSpacing(14)

        info = QVBoxLayout()
        info.setSpacing(4)
        label = "PRE SCAN" if is_prescan else "POST SCAN"
        title = QLabel(f"{label} #{parsed['index']} (SCAN {parsed['scan_id']})")
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

        if not is_prescan:
            result_path = _find_matching_compared(filepath)
            btn_result = QPushButton("Heatmap")
            btn_result.setProperty("cssClass", "rowActionBtn")
            btn_result.setEnabled(result_path is not None)
            btn_result.clicked.connect(lambda _checked, p=result_path: self.on_view_3d(p))
            h.addWidget(btn_result)

        # Pre-Scan rows match by scan_id only (see
        # _find_matching_report_for_segment) since generate_filename()
        # gives a prescan file whatever post-index happened to be current
        # at scan time - that index carries no real relationship to any
        # compared cloud/report, so index can't be part of the match here.
        if is_prescan:
            report_path = _find_matching_report_for_segment(job_path, parsed["scan_id"])
        else:
            report_path = _find_matching_report(filepath)
        btn_report = QPushButton("Report")
        btn_report.setProperty("cssClass", "rowActionBtn")
        btn_report.setEnabled(report_path is not None)
        btn_report.clicked.connect(lambda _checked, p=report_path: self._open_report(p))
        h.addWidget(btn_report)

        # Delete sits with extra breathing room on both sides - it's the
        # only destructive action in the row, so it must not be adjacent
        # to the scrollbar (easy to hit by accident while scrolling) nor
        # to the Compare toggle (easy to hit while selecting for compare).
        h.addSpacing(28)
        btn_delete = QPushButton("Delete")
        btn_delete.setProperty("cssClass", "rowDangerBtn")
        btn_delete.clicked.connect(lambda _checked, p=filepath: self._delete_file(p))
        h.addWidget(btn_delete)
        h.addSpacing(28)

        # Compare toggle is the rightmost element, right against the
        # scrollbar edge - it's non-destructive (just marks/unmarks this
        # file for the Compare Selected action below), so it's safe to
        # place where an accidental tap is most likely.
        compare_toggle = QPushButton("Compare")
        compare_toggle.setProperty("cssClass", "compareToggle")
        compare_toggle.setCheckable(True)
        compare_toggle.setChecked(filepath in self._checked_files)
        compare_toggle.toggled.connect(lambda checked, p=filepath: self._on_row_checked(p, checked))
        h.addWidget(compare_toggle)

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
        # qpdfview --unique reuses its existing instance's window if one is
        # already running rather than opening a new one, so wmctrl (by
        # window class, not PID - the new process may just be a client
        # signalling the running instance) is used to raise+focus it
        # instead of relying on the Popen'd process being the visible window.
        try:
            subprocess.Popen(["wmctrl", "-a", "qpdfview"])
        except OSError:
            pass  # wmctrl not installed - report still opened, just not raised

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
