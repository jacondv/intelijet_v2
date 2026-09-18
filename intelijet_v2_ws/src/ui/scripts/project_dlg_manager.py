# -*- coding: utf-8 -*-
from datetime import datetime

from PyQt5 import QtCore
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QSize
from PyQt5.QtWidgets import (
    QApplication, QWidget, QFrame, QMessageBox, QHBoxLayout, QVBoxLayout, QGridLayout,
    QLabel, QPushButton, QLineEdit, QSpinBox, QComboBox, QStyledItemDelegate,
    QSizePolicy,
)

from ui.project_dlg_ui import Ui_frm_ProjectPage

from ui.models.job_info import JobInfo
from ui.services import project_repository as repo
from ui.services.job_store import JobStore
from ui.style_tokens import DELETE_ICON_PATH, UNSCHEDULE_ICON_PATH, FOLDER_ICON_PATH, FONT_SUBTEXT

ROW_ICON_SIZE = QSize(40, 40)
DELETE_ICON_SIZE = QSize(48, 48)
# All row-family buttons (Schedule/Edit/Delete/Remove/Rename/...) share this
# fixed height so icon-only buttons (whose padding alone can't reproduce a
# text button's font-driven height) always end up the same size as the rest
# of their row instead of relying on padding math to happen to match.
ROW_BUTTON_HEIGHT = 104

PROJECT_DIR = repo.PROJECT_DIR
ACTIVE_JOB_FILE = repo.ACTIVE_JOB_FILE


class _ComboRowDelegate(QStyledItemDelegate):
    """Fixed popup row height for a QComboBox's dropdown - QSS ::item
    padding/min-height was confirmed to have no effect on row height in
    this Qt build (see report_page_manager.py's _PickerItemDelegate,
    which hit the same thing first), so real control has to go through a
    delegate's sizeHint instead of a stylesheet, or rows overlap."""
    ROW_HEIGHT = 60

    def sizeHint(self, option, index):
        size = super().sizeHint(option, index)
        size.setHeight(self.ROW_HEIGHT)
        return size


class ProjectManager(QWidget, Ui_frm_ProjectPage):
    """2-column JOB tab (docs/ui_sample/App.html TAB 2): 'Projects & Jobs
    List' (left, always-expanded project cards - no collapse, matching
    the mockup) and 'Work Schedule' (right). Project/job rows are plain
    widgets built and rebuilt in render_projects()/render_schedule() -
    there's no QListWidget selection model here, every row carries its
    own project/job directly via closures on its buttons.

    New Project / New Job / Edit Job / Rename Project used to open as
    separate QDialog popups. On this touchscreen, a popup centered on
    screen gets covered by the nam72 on-screen keyboard the moment a text
    field inside it takes focus - repositioning the popup near the top of
    the screen never reliably stuck (Qt re-centers a QDialog on its
    parent internally the instant show()/exec_() runs, no matter what
    position was set beforehand). Inline editing sidesteps the problem
    entirely: there's no floating window to fight the keyboard over - the
    input fields are part of this page's own scroll area, and opening one
    just scrolls it into view above where the keyboard will dock.
    """

    # Emitted right after add_job_to_active/remove_job_from_active change
    # active_jobs.json - App connects this to refresh the header's
    # CURRENT JOB combobox immediately, instead of that combobox only
    # ever catching up on its own 30s polling timer (_refresh_active_jobs
    # in app.py), which made Schedule/Remove feel like it did nothing for
    # up to half a minute.
    active_jobs_changed = QtCore.pyqtSignal()

    def __init__(self, job_store=None):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Project Manager")

        # Shared with App (same active_jobs.json cache) instead of each
        # opening an independent JobStore - see project_repository.py.
        self.job_store = job_store or JobStore(repo.ACTIVE_JOB_FILE, repo.CURRENT_JOB_FILE)

        # Inline-edit state - at most one of these is "open" at a time.
        # None/False means "not editing anything right now".
        self._creating_project = False          # True while the New Project card is open
        self._renaming_project = None           # project name being renamed, or None
        self._job_panel = None                  # (project, job_or_None) - None job = "new job"
        self._scroll_target = None              # widget to scroll into view after the next render
        self._focus_target = None               # input field to focus after the next render

        self.btnNewProject.clicked.connect(self.new_project)
        self.txtSearchProject.textChanged.connect(lambda _text: self.render_projects())
        self.cmbSortProject.setItemDelegate(_ComboRowDelegate(self.cmbSortProject))
        self.cmbSortProject.currentIndexChanged.connect(lambda _idx: self.render_projects())

        self.render_projects()
        self.render_schedule()

    # =========================
    #        RENDERING
    # =========================
    def _clear_dynamic_rows(self, layout):
        """Remove every item except the trailing stretch (always last -
        every insert in this file uses insertWidget(count()-1, ...))."""
        while layout.count() > 1:
            item = layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

    def _empty_label(self, text):
        lbl = QLabel(text)
        lbl.setObjectName("emptyStateLabel")
        lbl.setAlignment(QtCore.Qt.AlignCenter)
        return lbl

    # (badge background, border, text, dot) - pill-with-dot style matching
    # the "Scheduled"/"Pending"/"Finished" status chips in the mock-up.
    _STATUS_BADGE_COLORS = {
        JobInfo.PENDING: ("#fffbeb", "#fde68a", "#b45309", "#f59e0b"),   # amber
        JobInfo.ACTIVE: ("#f0f9ff", "#bae6fd", "#0369a1", "#0ea5e9"),     # sky ("Scheduled")
        JobInfo.FINISHED: ("#ecfdf5", "#a7f3d0", "#047857", "#10b981"),  # emerald
    }

    def _divider(self):
        """A faint horizontal rule - used under a job/schedule card's title
        to visually separate it from the info+actions area below."""
        line = QFrame()
        line.setObjectName("rowDivider")
        line.setFixedHeight(1)
        return line

    def _status_badge(self, status):
        bg, border, fg, dot = self._STATUS_BADGE_COLORS.get(status, ("#f1f5f9", "#e2e8f0", "#334155", "#94a3b8"))
        badge = QWidget()
        badge.setStyleSheet(
            f"background-color: {bg}; border: 1px solid {border}; border-radius: 8px;"
        )
        layout = QHBoxLayout(badge)
        layout.setContentsMargins(10, 3, 12, 3)
        layout.setSpacing(6)

        dot_label = QLabel()
        dot_label.setFixedSize(9, 9)
        dot_label.setStyleSheet(f"background-color: {dot}; border-radius: 4px; border: none;")
        layout.addWidget(dot_label)

        # Matches jobRowSubtext/scheduleSubtext's font size (FONT_SUBTEXT)
        # so the badge reads at the same scale as the Created/Target lines
        # right next to it, instead of looking like fine print.
        text_label = QLabel((status or "Unknown").capitalize())
        text_label.setStyleSheet(f"background: transparent; border: none; color: {fg}; font-size: {FONT_SUBTEXT}; font-weight: 700;")
        layout.addWidget(text_label)

        badge.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        return badge

    def render_projects(self):
        layout = self.projectsListLayout
        self._clear_dynamic_rows(layout)
        self._scroll_target = None
        self._focus_target = None

        if self._creating_project:
            layout.insertWidget(0, self._build_new_project_card())

        search = self.txtSearchProject.text().strip().lower()
        sort_mode = self.cmbSortProject.currentData() or repo.SORT_DATE_DESC
        any_shown = False
        for project in repo.list_projects(sort_mode=sort_mode):
            jobs = repo.list_jobs(project)
            project_matches = not search or search in project.lower()
            jobs_to_show = jobs if project_matches else [j for j in jobs if search in j.lower()]
            if search and not project_matches and not jobs_to_show:
                continue
            any_shown = True
            layout.insertWidget(layout.count() - 1, self._build_project_card(project, jobs_to_show))

        if not any_shown and not self._creating_project:
            text = "No projects found." if search else "No projects yet - create one with \"+ New Project\"."
            layout.insertWidget(0, self._empty_label(text))

        # Scroll whatever inline editor just opened into view, above where
        # the on-screen keyboard will dock - deferred via singleShot(0) so
        # it runs after this layout pass has actually sized/placed the
        # new widget (ensureWidgetVisible on a not-yet-laid-out widget is
        # a no-op).
        if self._scroll_target is not None:
            target = self._scroll_target
            QtCore.QTimer.singleShot(0, lambda: self.projectsScroll.ensureWidgetVisible(target, 0, 80))

        # Same deferral reason as the scroll above - setFocus() called
        # while the widget was being built (before it's actually part of
        # the visible layout/window) is silently ignored by Qt, which is
        # why the field wasn't visibly focused/getting the on-screen
        # keyboard before this.
        if self._focus_target is not None:
            field = self._focus_target
            QtCore.QTimer.singleShot(0, field.setFocus)
        else:
            # Rebuilding this list (e.g. after Schedule/Remove) deletes
            # whatever row widget currently held focus - Qt's automatic
            # focus succession then lands on some other newly-built
            # focusable field (often the search box) even though nothing
            # here asked to edit text, which pops the on-screen keyboard
            # for no reason. Explicitly drop focus off of it when we're
            # not the ones opening an editor.
            def _clear_stray_focus():
                w = QApplication.focusWidget()
                if w is not None:
                    w.clearFocus()
            QtCore.QTimer.singleShot(0, _clear_stray_focus)

    def _build_project_card(self, project, jobs):
        card = QFrame()
        card.setObjectName("projectCard")
        v = QVBoxLayout(card)
        v.setContentsMargins(24, 20, 24, 20)
        v.setSpacing(12)

        if self._renaming_project == project:
            v.addLayout(self._build_rename_project_header(project))
        else:
            header = QHBoxLayout()
            header.setSpacing(12)
            folder_icon = QLabel()
            folder_icon.setStyleSheet("background: transparent; border: none;")
            # QIcon(...).pixmap(size) rasterizes the SVG at the target size
            # directly (sharp) - QPixmap(path).scaled(...) would instead
            # scale up the SVG's native small bitmap and come out blurry.
            folder_icon.setPixmap(QIcon(FOLDER_ICON_PATH).pixmap(72, 72))
            title = QLabel(f"Project: {project}")
            title.setObjectName("projectHeaderLabel")
            # Icon and title share their own tight-spaced group so the
            # bigger icon doesn't push the title away from it - header's
            # own spacing (12) still applies between this group and the
            # buttons on the right.
            icon_title = QHBoxLayout()
            icon_title.setSpacing(4)
            icon_title.addWidget(folder_icon)
            icon_title.addWidget(title)
            header.addLayout(icon_title)
            header.addStretch(1)

            btn_add_job = QPushButton("+ Add Job")
            btn_add_job.setProperty("cssClass", "rowPrimaryBtn")
            btn_add_job.setFixedHeight(ROW_BUTTON_HEIGHT)
            btn_add_job.clicked.connect(lambda _checked, p=project: self.new_job(p))
            header.addWidget(btn_add_job)

            btn_rename = QPushButton("Rename")
            btn_rename.setProperty("cssClass", "rowActionBtn")
            btn_rename.setFixedHeight(ROW_BUTTON_HEIGHT)
            btn_rename.clicked.connect(lambda _checked, p=project: self.rename_project(p))
            header.addWidget(btn_rename)

            btn_delete = QPushButton()
            btn_delete.setIcon(QIcon(DELETE_ICON_PATH))
            btn_delete.setIconSize(DELETE_ICON_SIZE)
            btn_delete.setToolTip("Delete Project")
            btn_delete.setProperty("cssClass", "rowDangerIconBtn")
            btn_delete.setFixedHeight(ROW_BUTTON_HEIGHT)
            btn_delete.clicked.connect(lambda _checked, p=project: self.delete_project(p))
            header.addWidget(btn_delete)

            v.addLayout(header)

        if self._job_panel == (project, None):
            v.addWidget(self._build_job_edit_panel(project, None))

        if not jobs:
            v.addWidget(self._empty_label("No jobs yet."))
        for job in jobs:
            if self._job_panel == (project, job):
                v.addWidget(self._build_job_edit_panel(project, job))
            else:
                v.addWidget(self._build_job_row(project, job))

        return card

    def _build_rename_project_header(self, project):
        header = QHBoxLayout()
        header.setSpacing(12)

        edit = QLineEdit(project)
        edit.setObjectName("jobEditField")
        edit.setMinimumHeight(50)
        header.addWidget(edit, 1)

        btn_cancel = QPushButton("Cancel")
        btn_cancel.setProperty("cssClass", "rowActionBtn")
        btn_cancel.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_cancel.clicked.connect(self._cancel_rename_project)
        header.addWidget(btn_cancel)

        btn_save = QPushButton("Save")
        btn_save.setProperty("cssClass", "rowPrimaryBtn")
        btn_save.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_save.clicked.connect(lambda _checked, p=project, e=edit: self._commit_rename_project(p, e.text()))
        header.addWidget(btn_save)

        edit.returnPressed.connect(lambda p=project, e=edit: self._commit_rename_project(p, e.text()))
        self._scroll_target = edit
        self._focus_target = edit
        return header

    def _build_new_project_card(self):
        # Same "jobEditPanel" look as the New/Edit Job form - a visible
        # accent border around the whole card so it's obvious this is the
        # thing currently being edited, not just another project row.
        card = QFrame()
        card.setObjectName("jobEditPanel")
        v = QVBoxLayout(card)
        v.setContentsMargins(28, 24, 28, 24)
        v.setSpacing(12)

        title = QLabel("New Project")
        title.setObjectName("jobEditPanelTitle")
        v.addWidget(title)

        edit = QLineEdit()
        edit.setObjectName("jobEditField")
        edit.setPlaceholderText("Enter project name...")
        edit.setMinimumHeight(50)
        v.addWidget(edit)

        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        btn_cancel = QPushButton("Cancel")
        btn_cancel.setProperty("cssClass", "rowActionBtn")
        btn_cancel.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_cancel.clicked.connect(self._cancel_new_project)
        btn_row.addWidget(btn_cancel)

        btn_save = QPushButton("Save")
        btn_save.setProperty("cssClass", "rowPrimaryBtn")
        btn_save.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_save.clicked.connect(lambda _checked, e=edit: self._commit_new_project(e.text()))
        btn_row.addWidget(btn_save)
        v.addLayout(btn_row)

        edit.returnPressed.connect(lambda e=edit: self._commit_new_project(e.text()))
        self._scroll_target = card
        self._focus_target = edit
        return card

    def _build_job_row(self, project, job):
        row = QFrame()
        row.setObjectName("jobRow")
        v = QVBoxLayout(row)
        v.setContentsMargins(24, 16, 24, 16)
        v.setSpacing(8)

        # Title always gets its own full-width line on top, so a long job
        # name wraps/elides in place instead of squeezing the info or
        # button columns below it.
        title = QLabel(job)
        title.setObjectName("jobRowTitle")
        title.setWordWrap(True)
        v.addWidget(title)
        v.addWidget(self._divider())

        job_info = repo.load_job_info(project, job)

        bottom = QHBoxLayout()
        bottom.setSpacing(12)

        info = QVBoxLayout()
        info.setSpacing(2)

        badge_row = QHBoxLayout()
        badge_row.addWidget(self._status_badge(job_info.status if job_info else None), 0)
        badge_row.addStretch(1)
        info.addLayout(badge_row)

        created = job_info.created if job_info else "--"
        subtext = QLabel(f"Created: {created}")
        subtext.setObjectName("jobRowSubtext")
        info.addWidget(subtext)

        params = job_info.parameters if job_info else {}
        target_thickness = params.get("target_thickness", 60)
        tolerance = params.get("tolerance", 10)
        target_label = QLabel(f"Target: {target_thickness}±{tolerance} mm")
        target_label.setObjectName("jobRowSubtext")
        info.addWidget(target_label)
        bottom.addLayout(info, 1)

        buttons = QHBoxLayout()
        buttons.setSpacing(12)

        is_scheduled = any(
            a["project"] == project and a["job"] == job
            for a in self.job_store.list_active_jobs()
        )
        btn_schedule = QPushButton("Scheduled" if is_scheduled else "Schedule")
        btn_schedule.setProperty("cssClass", "rowScheduledBtn" if is_scheduled else "rowPrimaryBtn")
        btn_schedule.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_schedule.clicked.connect(lambda _checked, p=project, j=job: self.add_job_to_active(p, j))
        buttons.addWidget(btn_schedule)

        btn_edit = QPushButton("Edit")
        btn_edit.setProperty("cssClass", "rowActionBtn")
        btn_edit.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_edit.clicked.connect(lambda _checked, p=project, j=job: self.edit_job(p, j))
        buttons.addWidget(btn_edit)

        btn_delete = QPushButton()
        btn_delete.setIcon(QIcon(DELETE_ICON_PATH))
        btn_delete.setIconSize(DELETE_ICON_SIZE)
        btn_delete.setToolTip("Delete")
        btn_delete.setProperty("cssClass", "rowDangerIconBtn")
        btn_delete.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_delete.clicked.connect(lambda _checked, p=project, j=job: self.delete_job(p, j))
        buttons.addWidget(btn_delete)

        bottom.addLayout(buttons, 0)
        v.addLayout(bottom)

        return row

    def _build_job_edit_panel(self, project, job):
        """Inline replacement for the old JobInfoDialog popup - used for
        both "New Job" (job=None) and "Edit Job" (job=an existing job
        name), same fields either way."""
        job_info = repo.load_job_info(project, job) if job else None

        panel = QFrame()
        panel.setObjectName("jobEditPanel")
        v = QVBoxLayout(panel)
        v.setContentsMargins(28, 24, 28, 24)
        v.setSpacing(18)

        title = QLabel("Edit Job" if job else "New Job")
        title.setObjectName("jobEditPanelTitle")
        v.addWidget(title)

        name_edit = QLineEdit(job_info.name if job_info else "")
        name_edit.setObjectName("jobEditField")
        name_edit.setMinimumHeight(50)

        status_combo = QComboBox()
        status_combo.addItems([JobInfo.PENDING, JobInfo.ACTIVE, JobInfo.FINISHED])
        status_combo.setMinimumHeight(50)
        status_combo.setItemDelegate(_ComboRowDelegate(status_combo))
        if job_info:
            status_combo.setCurrentText(job_info.status)

        description_edit = QLineEdit(job_info.description if job_info else "")
        description_edit.setObjectName("jobEditField")
        description_edit.setMinimumHeight(50)

        thickness_spin = QSpinBox()
        thickness_spin.setRange(0, 1000)
        thickness_spin.setMinimumHeight(50)
        thickness_spin.setValue(job_info.parameters.get("target_thickness", 60) if job_info else 60)

        tolerance_spin = QSpinBox()
        tolerance_spin.setRange(0, 100)
        tolerance_spin.setMinimumHeight(50)
        tolerance_spin.setValue(job_info.parameters.get("tolerance", 10) if job_info else 10)

        # Right-aligned label / field columns (a real 2-column form) instead
        # of the previous label-above-field stack - this panel has plenty
        # of width at its position in the layout for a label column without
        # squeezing the fields, and it matches the reference mock-up.
        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(14)
        grid.setColumnStretch(1, 1)
        for row, (caption, field) in enumerate((
            ("Job Name:", name_edit),
            ("Status:", status_combo),
            ("Description:", description_edit),
            ("Target Thickness:", thickness_spin),
            ("Tolerance:", tolerance_spin),
        )):
            label = QLabel(caption)
            label.setObjectName("jobEditFieldLabel")
            label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            grid.addWidget(label, row, 0)
            grid.addWidget(field, row, 1)
        v.addLayout(grid)

        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        btn_cancel = QPushButton("Cancel")
        btn_cancel.setProperty("cssClass", "rowActionBtn")
        btn_cancel.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_cancel.clicked.connect(self._cancel_job_panel)
        btn_row.addWidget(btn_cancel)

        btn_save = QPushButton("Save")
        btn_save.setProperty("cssClass", "rowPrimaryBtn")
        btn_save.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_save.clicked.connect(
            lambda _checked, p=project, j=job, ne=name_edit, sc=status_combo, de=description_edit,
                   ts=thickness_spin, tl=tolerance_spin:
            self._commit_job_panel(p, j, ne, sc, de, ts, tl)
        )
        btn_row.addWidget(btn_save)
        v.addLayout(btn_row)

        self._scroll_target = panel
        self._focus_target = name_edit
        return panel

    def render_schedule(self):
        layout = self.scheduleListLayout
        self._clear_dynamic_rows(layout)

        jobs = self.job_store.list_active_jobs()
        if not jobs:
            layout.insertWidget(0, self._empty_label("No scheduled jobs."))
            return
        for j in jobs:
            layout.insertWidget(layout.count() - 1, self._build_schedule_card(j["project"], j["job"]))

    def _build_schedule_card(self, project, job):
        # Same layout as a job row (_build_job_row): title on its own full
        # width line on top, then a bottom strip split into info (left) and
        # actions (right) - kept consistent across both lists instead of
        # this card using its own header/footer-with-dividers arrangement.
        card = QFrame()
        card.setObjectName("scheduleCard")
        v = QVBoxLayout(card)
        v.setContentsMargins(20, 12, 20, 12)
        v.setSpacing(6)

        job_info = repo.load_job_info(project, job)

        title = QLabel(f"{project} / {job}")
        title.setObjectName("scheduleTitle")
        title.setWordWrap(True)
        v.addWidget(title)
        v.addWidget(self._divider())

        bottom = QHBoxLayout()
        bottom.setSpacing(12)

        info = QVBoxLayout()
        info.setSpacing(2)

        badge_row = QHBoxLayout()
        badge_row.addWidget(self._status_badge(job_info.status if job_info else None), 0)
        badge_row.addStretch(1)
        info.addLayout(badge_row)

        params = job_info.parameters if job_info else {}
        target_thickness = params.get("target_thickness", 60)
        tolerance = params.get("tolerance", 10)
        target_label = QLabel(f"Target: {target_thickness}±{tolerance} mm")
        target_label.setObjectName("scheduleSubtext")
        info.addWidget(target_label)
        bottom.addLayout(info, 1)
        # Info here is just one short line, much shorter than the fixed-
        # height buttons next to it - vertically center it in the row
        # instead of top-aligning, so it doesn't read as floating in a
        # mostly-empty column next to the buttons.
        bottom.setAlignment(info, QtCore.Qt.AlignVCenter)

        buttons = QHBoxLayout()
        buttons.setSpacing(12)

        btn_finish = QPushButton("Done")
        btn_finish.setProperty("cssClass", "rowActionBtn")
        btn_finish.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_finish.clicked.connect(lambda _checked, p=project, j=job: self.finish_job(p, j))
        buttons.addWidget(btn_finish)

        btn_remove = QPushButton()
        btn_remove.setIcon(QIcon(UNSCHEDULE_ICON_PATH))
        btn_remove.setIconSize(ROW_ICON_SIZE)
        btn_remove.setToolTip("Remove from Schedule")
        btn_remove.setProperty("cssClass", "rowActionIconBtn")
        btn_remove.setFixedHeight(ROW_BUTTON_HEIGHT)
        btn_remove.clicked.connect(lambda _checked, p=project, j=job: self.remove_job_from_active(p, j))
        buttons.addWidget(btn_remove)

        bottom.addLayout(buttons, 0)
        v.addLayout(bottom)

        return card

    # =========================
    #      PROJECT ACTIONS
    # =========================
    def new_project(self):
        self._creating_project = True
        self.render_projects()

    def _cancel_new_project(self):
        self._creating_project = False
        self.render_projects()

    def _commit_new_project(self, name):
        name = name.strip()
        if not name:
            return

        try:
            repo.create_project(name)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self._creating_project = False
        self.render_projects()

    def rename_project(self, old_name):
        for job in self.job_store.list_active_jobs():
            if job["project"] == old_name:
                QMessageBox.warning(self, "Active Job", "Cannot rename a project with active jobs. Please remove its jobs from active jobs first.")
                return

        self._renaming_project = old_name
        self.render_projects()

    def _cancel_rename_project(self):
        self._renaming_project = None
        self.render_projects()

    def _commit_rename_project(self, old_name, new_name):
        new_name = new_name.strip()
        if not new_name or new_name == old_name:
            self._renaming_project = None
            self.render_projects()
            return

        try:
            repo.rename_project(old_name, new_name)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self.job_store.rename_active_job_project(old_name, new_name)
        self._renaming_project = None
        self.render_projects()
        self.render_schedule()

    def delete_project(self, name):
        for job in self.job_store.list_active_jobs():
            if job["project"] == name:
                QMessageBox.warning(self, "Active Job", "Cannot delete a project with active jobs. Please remove its jobs from [Active Work Orders].")
                return

        if QMessageBox.question(self, "Confirm", f"Delete project '{name}' and all its jobs?") != QMessageBox.Yes:
            return

        try:
            repo.delete_project(name)
        except OSError as e:
            QMessageBox.critical(self, "Error", f"Failed to delete project folder, some files may have already been removed : {e}")
            return

        self.render_projects()

    # =========================
    #        JOB ACTIONS
    # =========================
    def new_job(self, project):
        """Always called from a project card's "+ Add Job" button, which
        carries its own project context - no top-bar "+ New Job" button
        exists any more (a job with no project to belong to doesn't make
        sense in this card layout)."""
        self._job_panel = (project, None)
        self.render_projects()

    def edit_job(self, project, job):
        for j in self.job_store.list_active_jobs():
            if j["project"] == project and j["job"] == job:
                QMessageBox.warning(self, "Active Job", "Cannot edit an active job. Please remove it from active jobs first.")
                return

        self._job_panel = (project, job)
        self.render_projects()

    def _cancel_job_panel(self):
        self._job_panel = None
        self.render_projects()

    def _commit_job_panel(self, project, job, name_edit, status_combo, description_edit, thickness_spin, tolerance_spin):
        name = name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Invalid Name", "Job name cannot be empty.")
            return

        status = status_combo.currentText()
        description = description_edit.text().strip()
        parameters = {
            "target_thickness": thickness_spin.value(),
            "tolerance": tolerance_spin.value(),
        }

        if job is None:
            job_info = JobInfo(
                name=name,
                created=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                status=status,
                description=description,
                parameters=parameters,
            )
            try:
                repo.create_job(project, job_info)
            except repo.ProjectError as e:
                QMessageBox.warning(self, "Exists", str(e))
                return
        else:
            job_info = repo.load_job_info(project, job)
            if not job_info:
                QMessageBox.critical(self, "Error", f"Cannot load job_info.json for {job}")
                return

            if name != job:
                try:
                    repo.rename_job(project, job, name)
                except repo.ProjectError as e:
                    QMessageBox.warning(self, "Exists", str(e))
                    return
                self.job_store.rename_active_job(project, job, name)

            job_info.name = name
            job_info.status = status
            job_info.description = description
            job_info.parameters = parameters

            try:
                repo.save_job_info(project, job_info)
            except OSError as e:
                QMessageBox.critical(self, "Error", f"Unable to save job info:\n{e}")
                return

        self._job_panel = None
        self.render_projects()

    def delete_job(self, project, job):
        for j in self.job_store.list_active_jobs():
            if j["project"] == project and j["job"] == job:
                QMessageBox.warning(self, "Active Job", f"Cannot delete {job}.\nPlease cancel it from Active Work Orders first.")
                return

        if QMessageBox.question(self, "Confirm", f"Do you really want to delete '{job}'?") != QMessageBox.Yes:
            return

        repo.delete_job(project, job)
        self.render_projects()

    # =========================
    #     ACTIVE JOB SECTION
    # =========================
    def add_job_to_active(self, project, job):
        job_info = repo.load_job_info(project, job)
        if job_info and job_info.status == JobInfo.FINISHED:
            if QMessageBox.question(
                self,
                "Job Finished",
                f"Job '{job}' is already Finished.\nDo you want to reset it to Pending and schedule it again?",
            ) != QMessageBox.Yes:
                return
            job_info.status = JobInfo.PENDING
            repo.save_job_info(project, job_info)

        if not self.job_store.add_active_job(project, job):
            QMessageBox.information(self, "Exists", f"Job '{job}' is already active.")
            return

        job_info = repo.load_job_info(project, job)
        if job_info and job_info.status != JobInfo.FINISHED:
            job_info.status = JobInfo.ACTIVE
            repo.save_job_info(project, job_info)

        self.render_schedule()
        self.render_projects()  # job's own "Schedule" button needs to flip to "Scheduled"
        self.active_jobs_changed.emit()

    def remove_job_from_active(self, project, job):
        self.job_store.remove_active_job(project, job)

        job_info = repo.load_job_info(project, job)
        if job_info and job_info.status == JobInfo.ACTIVE:
            job_info.status = JobInfo.PENDING
            repo.save_job_info(project, job_info)

        self.render_schedule()
        self.render_projects()  # job's own "Schedule" button needs to flip back
        self.active_jobs_changed.emit()

    def finish_job(self, project, job):
        job_info = repo.load_job_info(project, job)
        if job_info:
            job_info.status = JobInfo.FINISHED
            repo.save_job_info(project, job_info)

        self.job_store.remove_active_job(project, job)
        self.render_schedule()
        self.render_projects()
        self.active_jobs_changed.emit()
