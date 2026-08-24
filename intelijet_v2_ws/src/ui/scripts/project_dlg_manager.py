# -*- coding: utf-8 -*-
from datetime import datetime

from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QWidget, QFrame, QInputDialog, QMessageBox, QHBoxLayout, QVBoxLayout,
    QLabel, QPushButton, QLineEdit,
)

from ui.project_dlg_ui import Ui_frm_ProjectPage

from ui.models.job_info import JobInfo
from ui.services import project_repository as repo
from ui.services.job_store import JobStore

PROJECT_DIR = repo.PROJECT_DIR
ACTIVE_JOB_FILE = repo.ACTIVE_JOB_FILE


from PyQt5.QtWidgets import QDialog, QFormLayout, QSpinBox, QComboBox, QDialogButtonBox, QDesktopWidget


def _position_dialog_near_top(dlg, margin_top=30, use_size_hint=True):
    """Popups (Rename Project, New/Edit Job) default to opening centered
    on screen, which the nam72 on-screen keyboard then sits right on top
    of/overlaps once it appears for one of the dialog's text fields -
    push the dialog up near the top of the screen instead, well clear of
    where the keyboard will dock at the bottom. use_size_hint=False for a
    dialog that already sets its own explicit size via resize() - calling
    adjustSize() on top of that would just discard it."""
    if use_size_hint:
        dlg.adjustSize()
    screen = QDesktopWidget().availableGeometry(dlg)
    x = screen.x() + (screen.width() - dlg.width()) // 2
    dlg.move(x, screen.y() + margin_top)


class NewProjectDlg(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("New Project")

        layout = QVBoxLayout(self)

        self.edit = QLineEdit(self)
        self.edit.setPlaceholderText("Enter project name...")
        self.edit.setMinimumHeight(50)  # dễ bấm trên tablet
        self.edit.setFocus()            # bắt focus -> bật bàn phím
        layout.addWidget(self.edit)

        btn_ok = QPushButton("OK")
        btn_cancel = QPushButton("Cancel")
        btn_ok.setDefault(True)  # so Enter (incl. from the on-screen keyboard) confirms
        btn_ok.clicked.connect(self.accept)
        btn_cancel.clicked.connect(self.reject)

        h = QHBoxLayout()
        h.addWidget(btn_cancel)
        h.addWidget(btn_ok)
        layout.addLayout(h)



    def get_text(self):
        return self.edit.text()


class JobInfoDialog(QDialog):
    """Dialog để nhập tất cả thông tin cho JobInfo"""
    def __init__(self, parent=None, default_name=""):
        super().__init__(parent)
        self.setWindowTitle("New Job Info")
        self.resize(600, 400)
        # ===== Set FONT 24px cho toàn dialog =====
        self.setStyleSheet("""
            QLineEdit, QComboBox, QSpinBox, QTextEdit {
                font-size: 24px;
                min-height: 60px;
                min-width: 120px;
            }
            QLabel {
                font-size: 24px;
            }
            QDialogButtonBox QPushButton {
                font-size: 24px;
                min-height: 60px;
                min-width: 120px;
                padding: 10px;
            }
        """)
        # ===== Set font chung =====

        self.name_edit = QLineEdit(default_name)
        self.status_combo = QComboBox()
        self.status_combo.addItems([JobInfo.PENDING, JobInfo.ACTIVE, JobInfo.FINISHED])
        self.description_edit = QLineEdit()

        self.target_thickness_spin = QSpinBox()
        self.target_thickness_spin.setRange(0, 1000)
        self.target_thickness_spin.setValue(60)

        self.tolerance_spin = QSpinBox()
        self.tolerance_spin.setRange(0, 100)
        self.tolerance_spin.setValue(10)

        layout = QFormLayout()
        layout.addRow("Job Name:", self.name_edit)
        layout.addRow("Status:", self.status_combo)
        layout.addRow("Description:", self.description_edit)
        layout.addRow("Target Thickness:", self.target_thickness_spin)
        layout.addRow("Tolerance:", self.tolerance_spin)

        self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        layout.addRow(self.buttons)


        self.setLayout(layout)
        _position_dialog_near_top(self, use_size_hint=False)

    def get_data(self):
        """Trả về dict chứa tất cả dữ liệu"""
        return {
            "name": self.name_edit.text().strip(),
            "status": self.status_combo.currentText(),
            "description": self.description_edit.text().strip(),
            "parameters": {
                "target_thickness": self.target_thickness_spin.value(),
                "tolerance": self.tolerance_spin.value()
            }
        }


class ProjectManager(QWidget, Ui_frm_ProjectPage):
    """2-column JOB tab (docs/ui_sample/App.html TAB 2): 'Projects & Jobs
    List' (left, always-expanded project cards - no collapse, matching
    the mockup) and 'Work Schedule' (right). Project/job rows are plain
    widgets built and rebuilt in render_projects()/render_schedule() -
    there's no QListWidget selection model here, every row carries its
    own project/job directly via closures on its buttons."""

    def __init__(self, job_store=None):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Project Manager")

        # Shared with App (same active_jobs.json cache) instead of each
        # opening an independent JobStore - see project_repository.py.
        self.job_store = job_store or JobStore(repo.ACTIVE_JOB_FILE, repo.CURRENT_JOB_FILE)

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

    def render_projects(self):
        layout = self.projectsListLayout
        self._clear_dynamic_rows(layout)

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

        if not any_shown:
            text = "No projects found." if search else "No projects yet - create one with \"+ New Project\"."
            layout.insertWidget(0, self._empty_label(text))

    def _build_project_card(self, project, jobs):
        card = QFrame()
        card.setObjectName("projectCard")
        v = QVBoxLayout(card)
        v.setContentsMargins(28, 24, 28, 24)
        v.setSpacing(16)

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

        if not jobs:
            v.addWidget(self._empty_label("No jobs yet."))
        for job in jobs:
            v.addWidget(self._build_job_row(project, job))

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
        status = job_info.status.capitalize() if job_info else "Unknown"
        subtext = QLabel(f"Status: {status}")
        subtext.setObjectName("scheduleSubtext")
        info.addWidget(subtext)
        h.addLayout(info)
        h.addStretch(1)

        btn_remove = QPushButton("Remove")
        btn_remove.setProperty("cssClass", "rowDangerBtn")
        btn_remove.clicked.connect(lambda _checked, p=project, j=job: self.remove_job_from_active(p, j))
        h.addWidget(btn_remove)

        return card

    # =========================
    #      PROJECT ACTIONS
    # =========================
    def new_project(self):
        dlg = NewProjectDlg(self)
        if dlg.exec_() != QDialog.Accepted:
            return

        name = dlg.get_text().strip()
        if not name:
            return

        try:
            repo.create_project(name)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self.render_projects()

    def rename_project(self, old_name):
        for job in self.job_store.list_active_jobs():
            if job["project"] == old_name:
                QMessageBox.warning(self, "Active Job", "Cannot rename a project with active jobs. Please remove its jobs from active jobs first.")
                return

        dlg = QInputDialog(self)
        dlg.setWindowTitle("Rename Project")
        dlg.setLabelText("Enter new name:")
        dlg.setTextValue(old_name)
        _position_dialog_near_top(dlg)
        ok = dlg.exec_() == QDialog.Accepted
        new_name = dlg.textValue()
        if not ok or not new_name.strip() or new_name == old_name:
            return
        new_name = new_name.strip()

        try:
            repo.rename_project(old_name, new_name)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self.job_store.rename_active_job_project(old_name, new_name)
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
        dlg = JobInfoDialog(self)
        if dlg.exec() != QDialog.Accepted:
            return  # user cancel

        data = dlg.get_data()
        name = data["name"]
        if not name:
            QMessageBox.warning(self, "Invalid Name", "Job name cannot be empty.")
            return

        job_info = JobInfo(
            name=name,
            created=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            status=data["status"],
            description=data["description"],
            parameters=data["parameters"]
        )

        try:
            repo.create_job(project, job_info)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self.render_projects()

    def edit_job(self, project, job):
        for j in self.job_store.list_active_jobs():
            if j["project"] == project and j["job"] == job:
                QMessageBox.warning(self, "Active Job", "Cannot edit an active job. Please remove it from active jobs first.")
                return

        job_info = repo.load_job_info(project, job)
        if not job_info:
            QMessageBox.critical(self, "Error", f"Cannot load job_info.json for {job}")
            return

        dlg = JobInfoDialog(self, default_name=job_info.name)
        dlg.name_edit.setText(job_info.name)
        dlg.status_combo.setCurrentText(job_info.status)
        dlg.description_edit.setText(job_info.description)
        dlg.target_thickness_spin.setValue(job_info.parameters.get("target_thickness", 60))
        dlg.tolerance_spin.setValue(job_info.parameters.get("tolerance", 17))

        if dlg.exec() != QDialog.Accepted:
            return  # user cancel

        data = dlg.get_data()
        new_name = data["name"]

        if new_name != job:
            try:
                repo.rename_job(project, job, new_name)
            except repo.ProjectError as e:
                QMessageBox.warning(self, "Exists", str(e))
                return
            self.job_store.rename_active_job(project, job, new_name)
            job = new_name

        job_info.name = new_name
        job_info.status = data["status"]
        job_info.description = data["description"]
        job_info.parameters = data["parameters"]

        try:
            repo.save_job_info(project, job_info)
        except OSError as e:
            QMessageBox.critical(self, "Error", f"Unable to save job info:\n{e}")
            return

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
        if not self.job_store.add_active_job(project, job):
            QMessageBox.information(self, "Exists", f"Job '{job}' is already active.")
            return
        self.render_schedule()
        self.render_projects()  # job's own "Schedule" button needs to flip to "Scheduled"

    def remove_job_from_active(self, project, job):
        self.job_store.remove_active_job(project, job)
        self.render_schedule()
        self.render_projects()  # job's own "Schedule" button needs to flip back
