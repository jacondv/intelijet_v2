# -*- coding: utf-8 -*-
import os
import shutil
import json
import re

from datetime import datetime
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QInputDialog, QMessageBox, QListWidgetItem,QCheckBox,QLabel, QListWidget
from PyQt5.QtCore import pyqtSignal

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


class CompareManager(QDialog, Ui_frm_MainForm):
    polydataSignal = pyqtSignal(object)
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.resize(1300, 800)
        self.setWindowTitle("Compare Cloud Manager")
        os.makedirs(PROJECT_DIR, exist_ok=True)

        # Biến dữ liệu
        self.projects = {}
        self.current_project = None
        self.current_job = None
        self.selected_files = []
        self._initialized = False
        if not self._initialized:
            self.initialize()

        # Nạp danh sách
        # self.load_projects()

        # ====== CONNECT SIGNALS ======

        self.txtSearchProject.textChanged.connect(self.filter_projects)
        self.txtSearchJob.textChanged.connect(self.filter_jobs)
        self.lstProject.itemClicked.connect(self.select_project)
        self.lstJob.itemClicked.connect(self.select_job)

        self.chkShowPostScan.stateChanged.connect(lambda: self.load_job_detail_list(ascending=True))
        self.chkShowPreScan.stateChanged.connect(lambda: self.load_job_detail_list(ascending=True))
        self.chkShowCompared.stateChanged.connect(lambda: self.load_job_detail_list(ascending=True))
        # ===== CONNECT BUTTONS ======
        self.btnOk.clicked.connect(self.get_compare_files)
        self.btnCancel.clicked.connect(self.reject)
        self.btnDeleteItem.released.connect(self.delete_item)
        self.btnOpenItem.released.connect(self.on_file_opened)
        self.btnAsc.released.connect(lambda: self.load_job_detail_list(ascending=True))
        self.btnDesc.released.connect(lambda: self.load_job_detail_list(ascending=False))
        # self.update_project_list()

    # =========================
    #      PROJECT SECTION
    # =========================
    def get_result(self):
        # Đưa phần tử chứa 'pre' lên đầu
        self.selected_files.sort(key=lambda x: 0 if "pre" in x.lower() else 1)
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


    def select_project(self, item):
        """Chọn 1 project để hiển thị job bên phải."""
        name = item.text()
        self.current_project = name
        self.lblProjectName.setText(name)
        self.update_job_list()
        self.lstJobDetail.clear()


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

        self.load_job_detail_list(ascending=True)


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


    def delete_item(self):
        current_item = self.lstJobDetail.currentItem()
        if current_item is None:
            QMessageBox.warning(self, "Warning", "No files selected to delete.")
            return
        widget = self.lstJobDetail.itemWidget(current_item)
        if not widget:
            QMessageBox.warning(self, "Warning", "Widget for the item was not found.")
            return
        filename = widget.filename
        reply = QMessageBox.question(
            self,
            "Confirm",
            f"Are you sure you want to delete the file:\n{filename} ?",
            QMessageBox.Yes | QMessageBox.No
            )
        if reply != QMessageBox.Yes:
            return
        
        filepath = os.path.join(PROJECT_DIR, self.current_project, self.current_job, filename)
        try:
            if os.path.exists(filepath):
                os.remove(filepath)
            else:
                QMessageBox.warning(self, "Warning", "File does not exist on disk.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error while deleting file:\n{str(e)}")
            return    

        row = self.lstJobDetail.row(current_item)
        self.lstJobDetail.takeItem(row)   
        

    #Sort lstJobDetail 
    def load_job_detail_list(self,ascending=True):
        # self.lstJobDetail.sortItems(QtCore.Qt.AscendingOrder if ascending else QtCore.Qt.DescendingOrder)
                # Hiển thị lên lstJobDetail
        checked_filenames = []
        for i in range(self.lstJobDetail.count()):
            item = self.lstJobDetail.item(i)
            widget = self.lstJobDetail.itemWidget(item)  # đây là FileItemWidget
            if widget and widget.ui.chkChooseCloud.isChecked():  # lấy checkbox trực tiếp
                checked_filenames.append(widget.filename)       # lấy filename trực tiếp

        try:
            job_path = os.path.join(PROJECT_DIR, self.current_project, self.current_job)
            files = sorted([f for f in os.listdir(job_path) if f.lower().endswith(".ply")])
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error while retrieving files:\n{str(e)}")
            return
        
        # filter selct showing files post/pre/compared clouds files
        filter_list = []
        if self.chkShowPostScan.isChecked():
            filter_list.append("pre")
        if self.chkShowPreScan.isChecked():
            filter_list.append("post")
        if self.chkShowCompared.isChecked():
            filter_list.append("compared")

        if filter_list:
            files = self.filter_job_detail(files, filter_list)

        if not ascending:
            files.reverse()
        
        self.lstJobDetail.clear()
        for f in files:
            item = QListWidgetItem(self.lstJobDetail)
            f_widget = FileItemWidget(f,job_path, view_mode='2')
            filepath = os.path.join(job_path, f)
            item.setSizeHint(f_widget.sizeHint())
            if f in checked_filenames:
                f_widget.ui.chkChooseCloud.setChecked(True)

            self.lstJobDetail.addItem(item)
            self.lstJobDetail.setItemWidget(item, f_widget)
       
    #Filter lstJobDetail
    def filter_job_detail(self, origin_list, filter_list):
        """
        Lọc origin_list, chỉ giữ các phần tử **không chứa** bất kỳ cụm từ nào trong filter_list.

        Args:
            origin_list (list of str): danh sách gốc cần lọc
            filter_list (list of str): các cụm từ cần loại bỏ
            text_filter_list (list of str, optional): danh sách chứa các text khác (nếu muốn)

        Returns:
            list of str: danh sách đã lọc
        """
        filtered = []
        for item in origin_list:
            # Chuyển item sang lowercase để so sánh không phân biệt hoa thường
            item_lower = item.lower()

            # Kiểm tra nếu item chứa bất kỳ từ nào trong filter_list
            if any(f.lower() in item_lower for f in filter_list):
                filtered.append(item)
                    
        return filtered

    # =========================
    #     ACTIVE JOB SECTION
    # =========================

    def get_compare_files(self):
        self.selected_files = []
        checked_filenames = []
        for i in range(self.lstJobDetail.count()):
            item = self.lstJobDetail.item(i)
            widget = self.lstJobDetail.itemWidget(item)  # đây là FileItemWidget
            if widget and widget.ui.chkChooseCloud.isChecked():  # lấy checkbox trực tiếp
                checked_filenames.append(widget.filename)       # lấy filename trực tiếp
        if len(checked_filenames) < 2:
            QMessageBox.warning(self, "Select Files", "Please select at least two files to compare.")
            return
        
        if len(checked_filenames) > 2:
            QMessageBox.warning(self, "Select Files", f"{len(checked_filenames)} files selected. Please select only two files to compare.")
            return
        
        for filename in checked_filenames:
            filepath = os.path.join(PROJECT_DIR, self.current_project, self.current_job, filename)
            self.selected_files.append(filepath)

        print("Checked files for comparison:", checked_filenames)
        self.accept()
        
    def on_file_opened(self):
        from pps.data_converter import cloudconverter

        current_item = self.lstJobDetail.currentItem()
        if current_item is None:
            QMessageBox.warning(self, "Warning", "No files selected to open.")
            return
        widget = self.lstJobDetail.itemWidget(current_item)
        if not widget:
            QMessageBox.warning(self, "Warning", "Widget for the item was not found.")
            return
        filename = widget.filename       
        filepath = os.path.join(PROJECT_DIR, self.current_project, self.current_job, filename)
                # polydata = load_ply_as_polydata(filepath,0.02)
        o3d_cloud = cloudconverter.load_ply(filepath)
        if o3d_cloud is not None:
            self.polydataSignal.emit(o3d_cloud)
