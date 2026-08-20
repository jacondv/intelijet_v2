# -*- coding: utf-8 -*-
from datetime import datetime
from PyQt5 import QtCore
from PyQt5.QtCore import QEvent

from PyQt5.QtWidgets import QWidget, QInputDialog, QMessageBox, QListWidgetItem, QHBoxLayout, QPushButton, QVBoxLayout,QTextEdit, QLineEdit, QPlainTextEdit

from ui.project_dlg_ui import Ui_frm_ProjectPage

from ui.models.job_info import JobInfo
from ui.services import project_repository as repo
from ui.services.job_store import JobStore

PROJECT_DIR = repo.PROJECT_DIR
ACTIVE_JOB_FILE = repo.ACTIVE_JOB_FILE


from PyQt5.QtWidgets import QDialog, QFormLayout, QLineEdit, QSpinBox, QComboBox, QDialogButtonBox

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
    def __init__(self, job_store=None):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Project Manager")

        # All Projects/Jobs filesystem access goes through project_repository
        # (single source of truth for PROJECT_DIR's layout - see that
        # module's docstring). active_jobs.json goes through JobStore
        # (atomic writes) - accept an existing instance so this shares
        # the same cache App uses for cbbJobSelect instead of each
        # keeping an out-of-sync copy of the same file.
        self.job_store = job_store or JobStore(repo.ACTIVE_JOB_FILE, repo.CURRENT_JOB_FILE)

        self.current_project = None
        self.current_job = None

        # ====== CONNECT SIGNALS ======
        self.btnNewProject.clicked.connect(self.new_project)
        self.btnRenameProject.clicked.connect(self.rename_project)
        self.btnDeleteProject.clicked.connect(self.delete_project)
        self.lstProject.itemClicked.connect(self.select_project)
        self.txtSearchProject.textChanged.connect(self.filter_projects)

        self.btnNewJob.clicked.connect(self.new_job)
        self.btnEditJob.clicked.connect(self.edit_job)
        self.btnDeleteJob.clicked.connect(self.delete_job)
        self.lstJob.itemClicked.connect(self.select_job)
        self.txtSearchJob.textChanged.connect(self.filter_jobs)


        self.btnAdd.clicked.connect(self.add_job_to_active)
        self.btnRemove.clicked.connect(self.remove_job_from_active)

        self.update_project_list()
        self.load_active_jobs()


    # =========================
    #      PROJECT SECTION
    # =========================
    def new_project(self):
        """Tạo mới project (thư mục con trong ROOT_DIR)."""
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

        self.update_project_list()

        # Alway select new Item
        items = self.lstProject.findItems(name, QtCore.Qt.MatchExactly)
        if items:
            self.lstProject.setCurrentItem(items[0])

    def rename_project(self):
        """Đổi tên thư mục project."""
        item = self.lstProject.currentItem()
        if not item:
            QMessageBox.warning(self, "No selection", "Please select a project to rename.")
            return

        old_name = item.text()
        for job in self.job_store.list_active_jobs():
            if job["project"] == old_name:
                QMessageBox.warning(self, "Active Job", "Cannot rename a project with active jobs. Please remove its jobs from active jobs first.")
                return

        new_name, ok = QInputDialog.getText(self, "Rename Project", "Enter new name:", text=old_name)
        if not ok or not new_name.strip() or new_name == old_name:
            return

        new_name = new_name.strip()
        try:
            repo.rename_project(old_name, new_name)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self.update_project_list()

    def delete_project(self):
        """Xóa project (thư mục + job con)."""
        item = self.lstProject.currentItem()
        if not item:
            QMessageBox.warning(self, "No selection", "Please select a project to delete.")
            return

        name = item.text()
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

        self.update_project_list()
        self.lstJob.clear()
        self.lblProjectName.setText("#CurrentProject")
        self.lblJobName.setText("#CurrentJob")

    def select_project(self, item):
        """Chọn 1 project để hiển thị job bên phải."""
        name = item.text()
        self.current_project = name
        self.lblProjectName.setText(name)
        self.update_job_list()

    # =========================
    #         JOB SECTION
    # =========================

    def new_job(self):
        """Tạo job mới trong project hiện tại và tạo file job_info.json."""

        if not self.current_project:
            QMessageBox.warning(self, "No Project", "Please select a project first.")
            return

        # Hiển thị dialog nhập thông tin job
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
            repo.create_job(self.current_project, job_info)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self.update_job_list()


    def rename_job(self):
        """Đổi tên thư mục job."""
        if not self.current_project:
            QMessageBox.warning(self, "No Project", "Please select a project first.")
            return

        item = self.lstJob.currentItem()
        if not item:
            QMessageBox.warning(self, "No selection", "Please select a job to rename.")
            return

        old_name = item.text()
        new_name, ok = QInputDialog.getText(self, "Rename Job", "Enter new name:", text=old_name)
        if not ok or not new_name.strip() or new_name == old_name:
            return

        new_name = new_name.strip()
        try:
            repo.rename_job(self.current_project, old_name, new_name)
        except repo.ProjectError as e:
            QMessageBox.warning(self, "Exists", str(e))
            return

        self.job_store.rename_active_job(self.current_project, old_name, new_name)
        if self.current_job == old_name:
            self.current_job = new_name
        self.update_job_list()


    def delete_job(self):
        """Xóa job khỏi project."""
        if not self.current_project:
            QMessageBox.warning(self, "No Project", "Please select a project first.")
            return

        item = self.lstJob.currentItem()
        if not item:
            QMessageBox.warning(self, "No selection", "Please select a job to delete.")
            return

        name = item.text()
        for job in self.job_store.list_active_jobs():
            if job["project"] == self.current_project and job["job"] == name:
                QMessageBox.warning(self, "Active Job", f"⚠️ Cannot delete {name}.\nPlease cancel it from Active Work Orders first.")
                return

        if QMessageBox.question(self, "Confirm", f"⚠️ Do you really want to delete '{name}'?") != QMessageBox.Yes:
            return

        repo.delete_job(self.current_project, name)

        self.update_job_list()
        self.lblJobName.setText("#CurrentJob")


    def edit_job(self):
        """Chỉnh sửa thông tin job hiện tại"""
        if not self.current_project or not self.current_job:
            QMessageBox.warning(self, "No selection", "Please select a job to edit.")
            return

        for job in self.job_store.list_active_jobs():
            if job["project"] == self.current_project and job["job"] == self.current_job:
                QMessageBox.warning(self, "Active Job", "Cannot edit an active job. Please remove it from active jobs first.")
                return

        job_info = repo.load_job_info(self.current_project, self.current_job)
        if not job_info:
            QMessageBox.critical(self, "Error", f"Cannot load job_info.json for {self.current_job}")
            return

        # Tạo dialog, load dữ liệu hiện tại
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

        # Nếu đổi tên folder
        if new_name != self.current_job:
            try:
                repo.rename_job(self.current_project, self.current_job, new_name)
            except repo.ProjectError as e:
                QMessageBox.warning(self, "Exists", str(e))
                return
            self.job_store.rename_active_job(self.current_project, self.current_job, new_name)
            self.current_job = new_name

        # Update thông tin job_info
        job_info.name = new_name
        job_info.status = data["status"]
        job_info.description = data["description"]
        job_info.parameters = data["parameters"]

        try:
            repo.save_job_info(self.current_project, job_info)
        except OSError as e:
            QMessageBox.critical(self, "Error", f"Unable to save job info:\n{e}")
            return

        self.update_job_list()
        item = self.lstJob.findItems(self.current_job, QtCore.Qt.MatchExactly)[0]
        self.select_job(item)  # reload detail


    def select_job(self, item: QListWidgetItem):
        """Hiển thị chi tiết job khi người dùng click chọn."""
        if not item:
            return
        name = item.text()
        self.current_job = name
        self.lblJobName.setText(name)
        self.lstJobDetail.clear()

        job_info = repo.load_job_info(self.current_project, self.current_job)
        if not job_info:
            QMessageBox.warning(self, "Error", "job_info.json not found or invalid")
            return

        # Hiển thị nội dung ra lstJobDetail từ job_info
        self.lstJobDetail.addItem(f"• Job: {job_info.name}")
        self.lstJobDetail.addItem(f"• Created: {job_info.created}")
        self.lstJobDetail.addItem(f"• Status: {job_info.status}")
        self.lstJobDetail.addItem(f"• Description: {job_info.description}")

        # Hiển thị parameters
        if job_info.parameters:
            self.lstJobDetail.addItem("⚙️ Parameters:")
            for key, value in job_info.parameters.items():
                self.lstJobDetail.addItem(f"    • {key}: {value}")


    # =========================
    #      LOAD / UPDATE
    # =========================
    def update_project_list(self):
        """Cập nhật danh sách project (luôn đọc trực tiếp từ filesystem,
        không giữ cache riêng - project_repository.list_projects() đã
        rẻ vừa đủ để gọi lại mỗi lần thay vì tự đồng bộ tay một bản sao)."""
        self.lstProject.clear()
        for name in repo.list_projects():
            self.lstProject.addItem(name)


    def update_job_list(self):
        """Cập nhật danh sách job theo project đang chọn."""
        self.lstJob.clear()
        if self.current_project:
            for job in repo.list_jobs(self.current_project):
                self.lstJob.addItem(job)


    def filter_projects(self, text):
        text = text.lower().strip()
        self.lstProject.clear()
        for name in repo.list_projects():
            if text in name.lower():
                self.lstProject.addItem(name)


    def filter_jobs(self, text):
        text = text.lower().strip()
        self.lstJob.clear()
        if self.current_project:
            for name in repo.list_jobs(self.current_project):
                if text in name.lower():
                    self.lstJob.addItem(name)

    # =========================
    #     ACTIVE JOB SECTION
    # =========================
    def load_active_jobs(self, data_only=False):
        """Đọc danh sách job đang active (qua JobStore - atomic, cached)."""
        jobs = self.job_store.list_active_jobs()

        if data_only:
            return jobs

        self.lstJobActive.clear()
        for job in jobs:
            job_name = f"{job['project']} / {job['job']}"
            self.lstJobActive.addItem(job_name)

        return jobs

    def add_job_to_active(self):
        """Thêm job hiện tại vào danh sách active."""
        if not self.current_project:
            QMessageBox.warning(self, "No Project", "Please select a project first.")
            return

        item = self.lstJob.currentItem()
        if not item:
            QMessageBox.warning(self, "No Job", "Please select a job to add.")
            return

        job_name = item.text()
        if not self.job_store.add_active_job(self.current_project, job_name):
            QMessageBox.information(self, "Exists", f"Job '{job_name}' is already active.")
            return

        self.load_active_jobs()

    def remove_job_from_active(self):
        """Xóa job khỏi danh sách active."""
        item = self.lstJobActive.currentItem()
        if not item:
            QMessageBox.warning(self, "No selection", "Please select an active job to remove.")
            return

        project_name, job_name = repo.parse_job_ref(item.text())
        if not job_name:
            QMessageBox.warning(self, "Invalid", "Invalid job format.")
            return

        self.job_store.remove_active_job(project_name, job_name)
        self.load_active_jobs()
