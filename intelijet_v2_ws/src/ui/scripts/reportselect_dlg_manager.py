import os

from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QListWidgetItem, QMessageBox
from ui.reportselect_dlg_ui import Ui_ReportSelect
from ui.job_item_widget import JobItemWidget, FileItemWidget

from shared.config_loader import CONFIG as cfg

BASE_DIR = cfg.BASE_DIR
# BASE_DIR = "/mnt/c/work/projects/intelijet_v2"
class ReportSelectManager(QDialog):
    def __init__(self, parent=None, mode="label"):
        super().__init__(parent)
        self.result = (None, None)
        self.jobs_root = os.path.join(BASE_DIR, "data")
        # Load UI đã thiết kế
        self.ui = Ui_ReportSelect()
        self.ui.setupUi(self)
        self.view_mode = mode  # "view" hoặc "label"

        # # Kết nối signal
        # self.ui.btnCreateJob.clicked.connect(self.add_job)
        self.load_jobs_from_disk()

        # self.ui.txtFilter.textChanged.connect(self.filter_jobs)

        # Connect button
        

    def setup_job_signals(self, job_widget, item, job_path):
        job_widget.clickedSignal.connect(lambda: self.on_item_selected(item, job_path))

    def on_item_selected(self, item, job_path):
        self.ui.lstJobs.setCurrentItem(item)

    def load_jobs_from_disk(self):
        self.ui.lstJobs.clear()
        mode=self.view_mode
        for job_name in os.listdir(self.jobs_root):
            job_path = os.path.join(self.jobs_root, job_name)
            if os.path.isdir(job_path):
                item = QListWidgetItem(self.ui.lstJobs)
                job_widget = JobItemWidget(job_name)
                if mode == "label":
                    job_widget.show_only_label()
                item.setSizeHint(job_widget.sizeHint())
                self.ui.lstJobs.addItem(item)
                self.ui.lstJobs.setItemWidget(item, job_widget)
                self.setup_job_signals(job_widget, item, job_path)

    def on_item_selected(self, item, job_path):
        self.ui.lstJobs.setCurrentItem(item)

        self.ui.lstJobItems.clear()        
        try:
            files = [
                f for f in os.listdir(job_path) if f.lower().endswith(".pdf")
            ]
        except Exception as e:
            return
        
        # Hiển thị lên lstJobDetail
        for f in files:
            item = QListWidgetItem(self.ui.lstJobItems)
            f_widget = FileItemWidget(f,job_path, view_mode='view')
            filepath = os.path.join(job_path, f)
            item.setSizeHint(f_widget.sizeHint())
            
            self.ui.lstJobItems.addItem(item)
            self.ui.lstJobItems.setItemWidget(item, f_widget)
            f_widget.openSignal.connect(lambda filepath=filepath: self.on_file_opened(filepath))

    def on_file_opened(self, filepath):
        import subprocess
        subprocess.Popen(["xdg-open", filepath])


reportselect_dlg = ReportSelectManager()
