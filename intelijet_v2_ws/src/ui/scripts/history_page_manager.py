import os
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QListWidgetItem, QMessageBox, QAbstractItemView
from PyQt5.QtCore import pyqtSignal, Qt

from ui.historyview_page_ui import Ui_frmHistoryView
from ui.job_item_widget import JobItemWidget, FileItemWidget
from ui.utils import load_ply_as_polydata

from shared.config_loader import CONFIG as cfg

BASE_DIR = cfg.BASE_DIR
DATA_DIR = cfg.DATA_DIR
PROJECT_DIR = os.path.join(BASE_DIR, DATA_DIR, "Projects")
ACTIVE_JOB_FILE = os.path.join(PROJECT_DIR, "active_jobs.json")

class HistoryPageManager(QWidget):
    polydataSignal = pyqtSignal(object)
    def __init__(self, parent=None):
        super().__init__(parent)
        # Load UI đã thiết kế
        self.ui = Ui_frmHistoryView()
        self.ui.setupUi(self)
        self.load_jobs()
        self.ui.lstJobDetail.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.ui.lstJobDetail.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)

    def setup_job_signals(self, job_widget, item, job_path):
        job_widget.clickedSignal.connect(lambda: self.on_item_selected(item, job_path))

    def load_jobs(self):
        import json
        self.ui.lstJobs.clear()
        json_file = ACTIVE_JOB_FILE
        if not os.path.exists(json_file):
            return
        with open(json_file, "r") as f:
            jobs = json.load(f)

        for job in jobs:
            job_name = job.get("job")
            job_path = job.get("path")
            if os.path.isdir(job_path):
                item = QListWidgetItem(self.ui.lstJobs)
                job_widget = JobItemWidget(job_name)
                job_widget.show_only_label()
                item.setSizeHint(job_widget.sizeHint())
                self.ui.lstJobs.addItem(item)
                self.ui.lstJobs.setItemWidget(item, job_widget)
                self.setup_job_signals(job_widget, item, job_path)

    def select_job(self, job_name: str):
        for i in range(self.ui.lstJobs.count()):
            item = self.ui.lstJobs.item(i)
            widget = self.ui.lstJobs.itemWidget(item)

            if not widget:
                continue

            # Giả sử JobItemWidget có QLabel tên lblJobName
            
            if widget.txtJobname.text().lower() == job_name.lower():
                self.ui.lstJobs.setCurrentItem(item)
                # nếu muốn scroll tới item
                self.ui.lstJobs.scrollToItem(item)

                return True

        return False

    def on_item_selected(self, item, job_path):
        self.ui.lstJobs.setCurrentItem(item)
        self.ui.lstJobDetail.clear()        
        
        try:
            files = sorted([f for f in os.listdir(job_path) if f.lower().endswith(".ply")],reverse=True)
        except Exception as e:
            return
        
        # Hiển thị lên lstJobDetail
        for f in files:
            item = QListWidgetItem(self.ui.lstJobDetail)
            f_widget = FileItemWidget(f,job_path,view_mode='3')
            filepath = os.path.join(job_path, f)
            item.setSizeHint(f_widget.sizeHint())
            
            self.ui.lstJobDetail.addItem(item)
            self.ui.lstJobDetail.setItemWidget(item, f_widget)
            f_widget.openSignal.connect(lambda item=item, filepath=filepath: self.on_file_opened(item, filepath))


    def on_file_opened(self, item, filepath):
        from pps.data_converter import cloudconverter

        self.ui.lstJobDetail.setCurrentItem(item)
        # polydata = load_ply_as_polydata(filepath,0.02)
        o3d_cloud = cloudconverter.load_ply(filepath)
        if o3d_cloud is not None:
            self.polydataSignal.emit(o3d_cloud)

