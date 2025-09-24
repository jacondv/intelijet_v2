import os
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QListWidgetItem, QMessageBox
from PyQt5.QtCore import pyqtSignal

from ui.historyview_page_ui import Ui_frmHistoryView
from ui.job_item_widget import JobItemWidget, FileItemWidget
from ui.utils import load_ply_as_polydata

from shared.config_loader import CONFIG as cfg

BASE_DIR = cfg.BASE_DIR
# BASE_DIR = "/mnt/c/work/projects/intelijet_v2"

class HistoryPageManager(QWidget):
    polydataSignal = pyqtSignal(object)
    def __init__(self, parent=None):
        super().__init__(parent)
        self.jobs_root = os.path.join(BASE_DIR, "data")
        # Load UI đã thiết kế
        self.ui = Ui_frmHistoryView()
        self.ui.setupUi(self)
        self.load_jobs_from_disk()


    def setup_job_signals(self, job_widget, item, job_path):
        job_widget.clickedSignal.connect(lambda: self.on_item_selected(item, job_path))
       

    def load_jobs_from_disk(self):
        self.ui.lstJobs.clear()
        for job_name in os.listdir(self.jobs_root):
            job_path = os.path.join(self.jobs_root, job_name)
            if os.path.isdir(job_path):
                item = QListWidgetItem(self.ui.lstJobs)
                job_widget = JobItemWidget(job_name)
                item.setSizeHint(job_widget.sizeHint())
                self.ui.lstJobs.addItem(item)
                self.ui.lstJobs.setItemWidget(item, job_widget)
                self.setup_job_signals(job_widget, item, job_path)


    def on_item_selected(self, item, job_path):
        self.ui.lstJobs.setCurrentItem(item)
        self.ui.lstJobDetail.clear()        
        
        try:
            files = [
                f for f in os.listdir(job_path)
            ]
        except Exception as e:
            return
        
        # Hiển thị lên lstJobDetail
        for f in files:
            item = QListWidgetItem(self.ui.lstJobDetail)
            f_widget = FileItemWidget(f,job_path)
            filepath = os.path.join(job_path, f)
            item.setSizeHint(f_widget.sizeHint())
            
            self.ui.lstJobDetail.addItem(item)
            self.ui.lstJobDetail.setItemWidget(item, f_widget)
            f_widget.openSignal.connect(lambda item=item, filepath=filepath: self.on_file_opened(item, filepath))


    def on_file_opened(self, item, filepath):
        self.ui.lstJobDetail.setCurrentItem(item)
        polydata = load_ply_as_polydata(filepath,0.02)
        if polydata is not None:
            self.polydataSignal.emit(polydata)

