# -*- coding: utf-8 -*-
from datetime import datetime

from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication, QWidget, QFrame, QMessageBox, QHBoxLayout, QVBoxLayout,
    QLabel, QPushButton, QLineEdit, QSpinBox, QComboBox, QStyledItemDelegate,
    QSizePolicy,
)

from ui.project_dlg_ui import Ui_frm_ProjectPage

from ui.models.job_info import JobInfo
from ui.services import project_repository as repo
from ui.services.job_store import JobStore

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

    _STATUS_BADGE_COLORS = {
        JobInfo.PENDING: ("#fef3c7", "#92400e"),   # amber
        JobInfo.ACTIVE: ("#dbeafe", "#1e40af"),     # blue ("Scheduled")
        JobInfo.FINISHED: ("#dcfce7", "#166534"),   # green
    }

    def _status_badge(self, status):
        bg, fg = self._STATUS_BADGE_COLORS.get(status, ("#e2e8f0", "#334155"))
        badge = QLabel((status or "Unknown").capitalize())
        badge.setAlignment(QtCore.Qt.AlignCenter)
        badge.setStyleSheet(
            f"background-color: {bg}; color: {fg}; font-weight: 700; "
            "border-radius: 8px; padding: 2px 12px;"
        )
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
        any_shown = False
        for project in repo.list_projects():
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
        v.setContentsMargins(28, 24, 28, 24)
        v.setSpacing(16)

        if self._renaming_project == project:
            v.addLayout(self._build_rename_project_header(project))
        else:
            header = QHBoxLayout()
            header.setSpacing(12)
            title = QLabel(f"Project: {project}")
            title.setObjectName("projectHeaderLabel")
            header.addWidget(title)
            header.addStretch(1)

            btn_add_job = QPushButton("+ Add Job")
            btn_add_job.setProperty("cssClass", "rowPrimaryBtn")
            btn_add_job.clicked.connect(lambda _checked, p=project: self.new_job(p))
            header.addWidget(btn_add_job)

            btn_rename = QPushButton("Rename")
            btn_rename.setProperty("cssClass", "rowActionBtn")
            btn_rename.clicked.connect(lambda _checked, p=project: self.rename_project(p))
            header.addWidget(btn_rename)

            btn_delete = QPushButton("Delete")
            btn_delete.setProperty("cssClass", "rowDangerBtn")
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
        btn_cancel.clicked.connect(self._cancel_rename_project)
        header.addWidget(btn_cancel)

        btn_save = QPushButton("Save")
        btn_save.setProperty("cssClass", "rowPrimaryBtn")
        btn_save.clicked.connect(lambda _checked, p=project, e=edit: self._commit_rename_project(p, e.text()))
        header.addWidget(btn_save)

        edit.returnPressed.connect(lambda p=project, e=edit: self._commit_rename_project(p, e.text()))
        self._scroll_target = edit
        self._focus_target = edit
        return header

    def _build_new_project_card(self):
        card = QFrame()
        card.setObjectName("projectCard")
        v = QVBoxLayout(card)
        v.setContentsMargins(28, 24, 28, 24)
        v.setSpacing(12)

        title = QLabel("New Project")
        title.setObjectName("projectHeaderLabel")
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
        btn_cancel.clicked.connect(self._cancel_new_project)
        btn_row.addWidget(btn_cancel)

        btn_save = QPushButton("Save")
        btn_save.setProperty("cssClass", "rowPrimaryBtn")
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
        h = QHBoxLayout(row)
        h.setContentsMargins(24, 16, 24, 16)
        h.setSpacing(12)

        info = QVBoxLayout()
        info.setSpacing(4)
        title = QLabel(job)
        title.setObjectName("jobRowTitle")
        info.addWidget(title)

        job_info = repo.load_job_info(project, job)
        created = job_info.created if job_info else "--"
        subtext = QLabel(f"Created: {created}")
        subtext.setObjectName("jobRowSubtext")
        info.addWidget(subtext)

        status_row = QHBoxLayout()
        status_row.setSpacing(6)
        status_row.addWidget(QLabel("Status:"), 0)
        status_row.itemAt(0).widget().setObjectName("jobRowSubtext")
        status_row.addWidget(self._status_badge(job_info.status if job_info else None), 0)
        status_row.addStretch(1)
        info.addLayout(status_row)
        h.addLayout(info)
        h.addStretch(1)

        is_scheduled = any(
            a["project"] == project and a["job"] == job
            for a in self.job_store.list_active_jobs()
        )
        btn_schedule = QPushButton("Scheduled" if is_scheduled else "Schedule")
        btn_schedule.setProperty("cssClass", "rowScheduledBtn" if is_scheduled else "rowPrimaryBtn")
        btn_schedule.clicked.connect(lambda _checked, p=project, j=job: self.add_job_to_active(p, j))
        h.addWidget(btn_schedule)

        btn_edit = QPushButton("Edit")
        btn_edit.setProperty("cssClass", "rowActionBtn")
        btn_edit.clicked.connect(lambda _checked, p=project, j=job: self.edit_job(p, j))
        h.addWidget(btn_edit)

        btn_delete = QPushButton("Delete")
        btn_delete.setProperty("cssClass", "rowDangerBtn")
        btn_delete.clicked.connect(lambda _checked, p=project, j=job: self.delete_job(p, j))
        h.addWidget(btn_delete)

        return row

    def _build_job_edit_panel(self, project, job):
        """Inline replacement for the old JobInfoDialog popup - used for
        both "New Job" (job=None) and "Edit Job" (job=an existing job
        name), same fields either way."""
        job_info = repo.load_job_info(project, job) if job else None

        panel = QFrame()
        panel.setObjectName("jobEditPanel")
        v = QVBoxLayout(panel)
        v.setContentsMargins(24, 20, 24, 20)
        v.setSpacing(10)

        title = QLabel("Edit Job" if job else "New Job")
        title.setObjectName("jobRowTitle")
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

        # Each field stacked as its own label-above-field block (not a
        # QFormLayout's side-by-side columns) - at this panel's width, a
        # 2-column form squeezed the field column too narrow and the
        # combo/spinbox text overlapped its own label. Full-width fields
        # stacked vertically can't collide with anything next to them.
        for caption, field in (
            ("Job Name:", name_edit),
            ("Status:", status_combo),
            ("Description:", description_edit),
            ("Target Thickness:", thickness_spin),
            ("Tolerance:", tolerance_spin),
        ):
            label = QLabel(caption)
            label.setObjectName("jobEditFieldLabel")
            v.addWidget(label)
            v.addWidget(field)

        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        btn_cancel = QPushButton("Cancel")
        btn_cancel.setProperty("cssClass", "rowActionBtn")
        btn_cancel.clicked.connect(self._cancel_job_panel)
        btn_row.addWidget(btn_cancel)

        btn_save = QPushButton("Save")
        btn_save.setProperty("cssClass", "rowPrimaryBtn")
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
        card = QFrame()
        card.setObjectName("scheduleCard")
        h = QHBoxLayout(card)
        h.setContentsMargins(28, 24, 28, 24)
        h.setSpacing(12)

        info = QVBoxLayout()
        info.setSpacing(4)
        title = QLabel(f"{project} / {job}")
        title.setObjectName("scheduleTitle")
        info.addWidget(title)

        job_info = repo.load_job_info(project, job)
        status_row = QHBoxLayout()
        status_row.setSpacing(6)
        status_row.addWidget(QLabel("Status:"), 0)
        status_row.itemAt(0).widget().setObjectName("scheduleSubtext")
        status_row.addWidget(self._status_badge(job_info.status if job_info else None), 0)
        status_row.addStretch(1)
        info.addLayout(status_row)
        h.addLayout(info)
        h.addStretch(1)

        btn_finish = QPushButton("Finish")
        btn_finish.setProperty("cssClass", "rowPrimaryBtn")
        btn_finish.clicked.connect(lambda _checked, p=project, j=job: self.finish_job(p, j))
        h.addWidget(btn_finish)

        btn_remove = QPushButton("Remove")
        btn_remove.setProperty("cssClass", "rowDangerBtn")
        btn_remove.clicked.connect(lambda _checked, p=project, j=job: self.remove_job_from_active(p, j))
        h.addWidget(btn_remove)

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
