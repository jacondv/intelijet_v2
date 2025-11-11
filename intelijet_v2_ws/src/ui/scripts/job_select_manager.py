# -*- coding: utf-8 -*-
import os
import json
from PyQt5 import QtWidgets, QtCore
from ui.job_select_dlg_ui import Ui_frm_JobSelect
from shared.config_loader import CONFIG as cfg

BASE_DIR = cfg.BASE_DIR
DATA_DIR = cfg.DATA_DIR
ROOT_DIR = os.path.join(BASE_DIR, DATA_DIR, "Projects")

ACTIVE_JOB_FILE = os.path.join(ROOT_DIR, "active_jobs.json")

class JobSelectManager(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_frm_JobSelect()
        self.ui.setupUi(self)

        # Dữ liệu
        self.projects = {}  # project_name -> list of jobs
        self.active_jobs = []  # list of dicts: {"project":..., "job":..., "path":...}

        # Load danh sách project/job từ thư mục
        self.load_projects()
        # Load active jobs từ file json
        self.load_active_jobs()
        self.update_project_list_widget()
        self.update_active_jobs_widget()

        # ===== SIGNALS =====
        self.ui.txtSearchProject.textChanged.connect(self.filter_projects)
        self.ui.txtSearchJob.textChanged.connect(self.filter_jobs)

        self.ui.listWidget.itemClicked.connect(self.on_project_selected)
        self.ui.btnAdd.clicked.connect(self.add_active_job)
        self.ui.btnRemove.clicked.connect(self.remove_active_job)

        # Chọn project đầu tiên (nếu có)
        if self.ui.listWidget.count() > 0:
            self.ui.listWidget.setCurrentRow(0)
            self.on_project_selected(self.ui.listWidget.item(0))

    # =========================
    #      LOAD PROJECT/JOB
    # =========================
    def load_projects(self):
        """Duyệt ROOT_DIR để lấy danh sách project và job."""
        self.projects = {}
        if not os.path.exists(ROOT_DIR):
            os.makedirs(ROOT_DIR)
            return
        for project_name in sorted(os.listdir(ROOT_DIR)):
            project_path = os.path.join(ROOT_DIR, project_name)
            if os.path.isdir(project_path):
                jobs = [j for j in sorted(os.listdir(project_path)) if os.path.isdir(os.path.join(project_path, j))]
                self.projects[project_name] = jobs

    def load_active_jobs(self):
        """Nạp active_jobs từ file JSON nếu có."""
        if os.path.exists(ACTIVE_JOB_FILE):
            with open(ACTIVE_JOB_FILE, "r") as f:
                self.active_jobs = json.load(f)
        else:
            self.active_jobs = []

    def save_active_jobs(self):
        """Lưu active_jobs vào file JSON."""
        os.makedirs(ROOT_DIR, exist_ok=True)
        with open(ACTIVE_JOB_FILE, "w") as f:
            json.dump(self.active_jobs, f, indent=4)

    # =========================
    #      UPDATE LIST WIDGET
    # =========================
    def update_project_list_widget(self, filter_text=""):
        self.ui.listWidget.clear()
        for project_name in sorted(self.projects.keys()):
            if filter_text.lower() in project_name.lower():
                self.ui.listWidget.addItem(project_name)

    def update_job_list_widget(self, project_name, filter_text=""):
        self.ui.listWidget_2.clear()
        if project_name in self.projects:
            for job_name in sorted(self.projects[project_name]):
                if filter_text.lower() in job_name.lower():
                    self.ui.listWidget_2.addItem(job_name)

    def update_active_jobs_widget(self):
        self.ui.listWidget_3.clear()
        for job in self.active_jobs:
            item_text = f"{job['project']} / {job['job']}"
            self.ui.listWidget_3.addItem(item_text)

    # =========================
    #      FILTERS
    # =========================
    def filter_projects(self, text):
        self.update_project_list_widget(filter_text=text)

    def filter_jobs(self, text):
        current_project_item = self.ui.listWidget.currentItem()
        if current_project_item:
            project_name = current_project_item.text()
            self.update_job_list_widget(project_name, filter_text=text)

    # =========================
    #      HANDLERS
    # =========================
    def on_project_selected(self, item):
        project_name = item.text()
        self.update_job_list_widget(project_name)
        self.ui.txtSearchJob.clear()

    def add_active_job(self):
        """Thêm job vào Active Work Orders."""
        project_item = self.ui.listWidget.currentItem()
        job_item = self.ui.listWidget_2.currentItem()
        if not project_item or not job_item:
            QtWidgets.QMessageBox.warning(self, "No Selection", "Please select a project and a job.")
            return

        project_name = project_item.text()
        job_name = job_item.text()
        job_path = os.path.join(ROOT_DIR, project_name, job_name)

        # Kiểm tra đã tồn tại trong active_jobs chưa
        for j in self.active_jobs:
            if j["project"] == project_name and j["job"] == job_name:
                QtWidgets.QMessageBox.information(self, "Exists", "Job already in Active Work Orders.")
                return

        # Thêm vào danh sách
        self.active_jobs.append({
            "project": project_name,
            "job": job_name,
            "path": job_path
        })
        self.save_active_jobs()
        self.update_active_jobs_widget()

    def remove_active_job(self):
        """Xóa job khỏi Active Work Orders."""
        selected_item = self.ui.listWidget_3.currentItem()
        if not selected_item:
            QtWidgets.QMessageBox.warning(self, "No Selection", "Please select a job to remove.")
            return

        text = selected_item.text()
        project_name, job_name = text.split(" / ", 1)

        # Xóa khỏi danh sách
        self.active_jobs = [j for j in self.active_jobs if not (j["project"] == project_name and j["job"] == job_name)]
        self.save_active_jobs()
        self.update_active_jobs_widget()


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = JobSelectManager()
    window.show()
    sys.exit(app.exec_())
