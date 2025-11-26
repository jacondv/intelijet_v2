# -*- coding: utf-8 -*-
import os
import shutil
import json
import re

from datetime import datetime
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QInputDialog, QMessageBox, QListWidgetItem,QCheckBox,QLabel

from ui.compare_dlg_ui import Ui_frm_MainForm
from shared.config_loader import CONFIG as cfg

from ui.models.job_info import JobInfo
from ui.job_item_widget import FileItemWidget

BASE_DIR = cfg.BASE_DIR
DATA_DIR = cfg.DATA_DIR
PROJECT_DIR = os.path.join(BASE_DIR, DATA_DIR, "Projects")
ACTIVE_JOB_FILE = os.path.join(PROJECT_DIR, "active_jobs.json")


from PyQt5.QtWidgets import QDialog, QFormLayout, QLineEdit, QSpinBox, QComboBox, QDialogButtonBox


class JobInfoDialog(QDialog):
    """Dialog để nhập tất cả thông tin cho JobInfo"""
    def __init__(self, parent=None, default_name=""):
        super().__init__(parent)
        self.setWindowTitle("New Job Info")
        self.resize(300, 200)

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


class ReportViewManager(QDialog, Ui_frm_MainForm):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Report View Manager")

        os.makedirs(PROJECT_DIR, exist_ok=True)

        # Biến dữ liệu
        self.projects = {}
        self.current_project = None
        self.current_job = None
        self.selected_files = []
        self._initialized = False   # flag kiểm tra
        # Gọi initialize nếu chưa được gọi trước đó
        if not self._initialized:
            self.initialize()

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

        # ===== CONNECT BUTTONS ======
        self.btnCancel.clicked.connect(self.reject)
        
    # =========================
    #      PROJECT SECTION
    # =========================
    def get_result(self):
        
        return self.selected_files


    def initialize(self, current_project=None, current_job=None):
        """Khởi tạo lại dialog về trạng thái ban đầu."""
        self._initialized = True
        # ==  Hide some objects ==
        self.btnNewProject.hide()
        self.btnRenameProject.hide()
        self.btnDeleteProject.hide()
        self.btnNewJob.hide()
        self.btnEditJob.hide()
        self.btnDeleteJob.hide()
        self.btnOk.hide()

        self.current_project = current_project
        self.current_job = current_job
        self.selected_files = []
        self.load_projects()
        self.update_project_list()
        self.update_job_list()
        self.lblProjectName.setText(current_project or "#CurrentProject")
        self.lblJobName.setText(current_job or "#CurrentJob")    
        
        items = self.lstProject.findItems(self.current_project, QtCore.Qt.MatchExactly)
        if items:
            self.lstProject.setCurrentItem(items[0])
            
        items = self.lstJob.findItems(self.current_job, QtCore.Qt.MatchExactly)
        if items:
            self.lstJob.setCurrentItem(items[0])
            self.select_job(items[0])


    def new_project(self):
        """Tạo mới project (thư mục con trong ROOT_DIR)."""
        name, ok = QInputDialog.getText(self, "New Project", "Enter project name:")
        if not ok or not name.strip():
            return

        name = name.strip()
        project_path = os.path.join(PROJECT_DIR, name)

        if os.path.exists(project_path):
            QMessageBox.warning(self, "Exists", f"Project '{name}' already exists.")
            return

        os.makedirs(project_path)
        self.projects[name] = {"jobs": [], "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        self.update_project_list()

        # Chọn luôn project mới
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
        activate_jobs = self.load_active_jobs(data_only=True)
        for job in activate_jobs:
            if job["project"] == self.current_project:
                QMessageBox.warning(self, "Active Job", "Cannot rename a project with active jobs. Please remove its jobs from active jobs first.")
                return

        new_name, ok = QInputDialog.getText(self, "Rename Project", "Enter new name:", text=old_name)
        if not ok or not new_name.strip() or new_name == old_name:
            return

        new_name = new_name.strip()
        old_path = os.path.join(PROJECT_DIR, old_name)
        new_path = os.path.join(PROJECT_DIR, new_name)

        if os.path.exists(new_path):
            QMessageBox.warning(self, "Exists", f"Project '{new_name}' already exists.")
            return

        os.rename(old_path, new_path)
        self.projects[new_name] = self.projects.pop(old_name)
        self.update_project_list()

    def delete_project(self):
        """Xóa project (thư mục + job con)."""
        item = self.lstProject.currentItem()
        if not item:
            QMessageBox.warning(self, "No selection", "Please select a project to delete.")
            return

        name = item.text()
        activate_jobs = self.load_active_jobs(data_only=True)
        for job in activate_jobs:
            if job["project"] == name:
                QMessageBox.warning(self, "Active Job", "Cannot delete a project with active jobs. Please remove its jobs from active jobs first.")
                return
            
        if QMessageBox.question(self, "Confirm", f"Delete project '{name}' and all its jobs?") != QMessageBox.Yes:
            return

        # Xóa thư mục thật
        project_path = os.path.join(PROJECT_DIR, name)
        if os.path.exists(project_path):
            try:
                shutil.rmtree(project_path)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to delete project folder, some files may have already been removed : {e}")
                return

        # Cập nhật bộ nhớ
        if name in self.projects and not os.path.exists(project_path):
            del self.projects[name]

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

        project_path = os.path.join(PROJECT_DIR, self.current_project)

        # Hiển thị dialog nhập thông tin job
        dlg = JobInfoDialog(self)
        if dlg.exec() != QDialog.Accepted:
            return  # user cancel

        data = dlg.get_data()
        name = data["name"]
        if not name:
            QMessageBox.warning(self, "Invalid Name", "Job name cannot be empty.")
            return

        job_path = os.path.join(project_path, name)
        if os.path.exists(job_path):
            QMessageBox.warning(self, "Exists", f"Job '{name}' already exists.")
            return

        os.makedirs(job_path)

        # Tạo JobInfo từ dữ liệu dialog
        job_info = JobInfo(
            name=name,
            created=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            status=data["status"],
            description=data["description"],
            parameters=data["parameters"]
        )

        if not job_info.save(job_path):
            QMessageBox.critical(self, "Error", f"Unable to save job info:\n{JobInfo.INFO_FILE}")
            return

        # Cập nhật bộ nhớ
        self.projects[self.current_project]["jobs"].append(name)
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
        old_path = os.path.join(PROJECT_DIR, self.current_project, old_name)
        new_path = os.path.join(PROJECT_DIR, self.current_project, new_name)

        if os.path.exists(new_path):
            QMessageBox.warning(self, "Exists", f"Job '{new_name}' already exists.")
            return

        os.rename(old_path, new_path)
        jobs = self.projects[self.current_project]["jobs"]
        idx = jobs.index(old_name)
        jobs[idx] = new_name

        job_info = JobInfo.load(new_path)
        job_info.name = new_name
        job_info.save(new_path)

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
        activate_jobs = self.load_active_jobs(data_only=True)
        for job in activate_jobs:
            if job["project"] == self.current_project and job["job"] == self.current_job:
                QMessageBox.warning(self, "Active Job", f"⚠️ Cannot delete {self.current_job}.\nPlease cancel it from Active Work Orders first.")
                return
            
        if QMessageBox.question(self, "Confirm", f"⚠️ Do you really want to delete '{name}'?") != QMessageBox.Yes:
            return

        # Xóa thư mục
        job_path = os.path.join(PROJECT_DIR, self.current_project, name)
        if os.path.exists(job_path):
            shutil.rmtree(job_path)

        # Cập nhật bộ nhớ
        self.projects[self.current_project]["jobs"].remove(name)
        self.update_job_list()
        self.lblJobName.setText("#CurrentJob")


    def edit_job(self):
        """Chỉnh sửa thông tin job hiện tại"""
        if not self.current_project or not self.current_job:
            QMessageBox.warning(self, "No selection", "Please select a job to edit.")
            return
        
        activate_jobs = self.load_active_jobs(data_only=True)
        for job in activate_jobs:
            if job["project"] == self.current_project and job["job"] == self.current_job:
                QMessageBox.warning(self, "Active Job", "Cannot edit an active job. Please remove it from active jobs first.")
                return

        job_path = os.path.join(PROJECT_DIR, self.current_project, self.current_job)
        job_info = JobInfo.load(job_path)
        if not job_info:
            QMessageBox.critical(self, "Error", f"Cannot load job_info.json in {job_path}")
            return

        # Tạo dialog, load dữ liệu hiện tại
        dlg = JobInfoDialog(self, default_name=job_info.name)
        dlg.name_edit.setText(job_info.name)
        dlg.status_combo.setCurrentText(job_info.status)
        dlg.description_edit.setText(job_info.description)
        dlg.target_thickness_spin.setValue(job_info.parameters.get("target_thickness", 60))
        dlg.tolerance_spin.setValue(job_info.parameters.get("tolerance", 10))

        if dlg.exec() != QDialog.Accepted:
            return  # user cancel

        data = dlg.get_data()
        new_name = data["name"]

        # Nếu đổi tên folder
        if new_name != self.current_job:
            new_job_path = os.path.join(PROJECT_DIR, self.current_project, new_name)
            if os.path.exists(new_job_path):
                QMessageBox.warning(self, "Exists", f"Job '{new_name}' already exists.")
                return
            os.rename(job_path, new_job_path)
            self.projects[self.current_project]["jobs"].remove(self.current_job)
            self.projects[self.current_project]["jobs"].append(new_name)
            self.current_job = new_name
            job_path = new_job_path

        # Update thông tin job_info
        job_info.name = new_name
        job_info.status = data["status"]
        job_info.description = data["description"]
        job_info.parameters = data["parameters"]

        if not job_info.save(job_path):
            QMessageBox.critical(self, "Error", f"Unable to save job info:\n{JobInfo.INFO_FILE}")
            return

        self.update_job_list()
        self.select_job(self.lstJob.currentItem())  # reload detail


    def select_job(self, item: QListWidgetItem):
        """Hiển thị chi tiết job khi người dùng click chọn."""
        name = item.text()
        self.current_job = name
        self.lblJobName.setText(name)
        self.lstJobInfo.clear()  # 🔁 đổi từ lstJobDetail → lstJobInfo

        job_info_file = os.path.join(PROJECT_DIR, self.current_project, self.current_job, "job_info.json") 
        if not os.path.exists(job_info_file):
            QMessageBox.warning(self, "Error", "job_info.json not found")
            return

        with open(job_info_file, "r") as f:
            try:
                job_info = json.load(f)
            except json.JSONDecodeError:
                QMessageBox.warning(self, "Error", "job_info.json error")
                return

        # Hiển thị nội dung ra lstJobInfo từ job_info.json
        self.lstJobInfo.addItem(f"• Job: {job_info.get('name', '')}")
        self.lstJobInfo.addItem(f"• Created: {job_info.get('created', '')}")
        self.lstJobInfo.addItem(f"• Status: {job_info.get('status', '')}")
        # self.lstJobInfo.addItem(f"• Description: {job_info.get('description', '')}")

        # Hiển thị parameters
        params = job_info.get("parameters", {})
        if params:
            self.lstJobInfo.addItem("⚙️ Parameters:")
            for key, value in params.items():
                self.lstJobInfo.addItem(f"    • {key}: {value}")

        # Hiển thị lên lstJobDetail

        try:
            job_path = os.path.join(PROJECT_DIR, self.current_project, self.current_job)
            files = sorted([f for f in os.listdir(job_path) if f.lower().endswith(".pdf")])
        except Exception as e:
            return
        
        self.lstJobDetail.clear()
        for f in files:
            item = QListWidgetItem(self.lstJobDetail)
            f_widget = FileItemWidget(f,job_path, view_mode='3')
            filepath = os.path.join(job_path, f)
            item.setSizeHint(f_widget.sizeHint())
            
            self.lstJobDetail.addItem(item)
            self.lstJobDetail.setItemWidget(item, f_widget)
            f_widget.openSignal.connect(lambda filepath=filepath: self.on_file_opened(filepath))



    # =========================
    #      LOAD / UPDATE
    # =========================
    def load_projects(self):
        """Đọc danh sách project và job trực tiếp từ thư mục."""
        self.projects = {}

        if not os.path.exists(PROJECT_DIR):
            os.makedirs(PROJECT_DIR)
            return

        for project_name in sorted(os.listdir(PROJECT_DIR)):
            project_path = os.path.join(PROJECT_DIR, project_name)
            if os.path.isdir(project_path):
                jobs = [j for j in sorted(os.listdir(project_path)) if os.path.isdir(os.path.join(project_path, j))]
                self.projects[project_name] = {"jobs": jobs}


    def update_project_list(self):
        """Cập nhật danh sách project."""
        self.lstProject.clear()
        for name in sorted(self.projects.keys()):
            self.lstProject.addItem(name)


    def update_job_list(self):
        """Cập nhật danh sách job theo project đang chọn."""
        self.lstJob.clear()
        if self.current_project and self.current_project in self.projects:
            for job in sorted(self.projects[self.current_project]["jobs"]):
                self.lstJob.addItem(job)


    def filter_projects(self, text):
        text = text.lower().strip()
        self.lstProject.clear()
        for item in self.projects.keys():
            if text in item.lower():
                self.lstProject.addItem(item)


    def filter_jobs(self, text):
        text = text.lower().strip()
        self.lstJob.clear()
        if self.current_project and self.current_project in self.projects:
            for item in sorted(self.projects[self.current_project]["jobs"]):
                if text in item.lower():
                    self.lstJob.addItem(item)
    # =========================
    #     OPEN FILE SECTION
    # =========================

    def on_file_opened(self, filepath):
        import subprocess
        print(filepath)
        if os.path.exists(filepath):
            subprocess.Popen(["xdg-open", filepath])
        else:
            print(f"File not found: {filepath}")