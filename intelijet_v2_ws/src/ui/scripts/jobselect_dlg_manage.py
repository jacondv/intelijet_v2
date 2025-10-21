import os

from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QListWidgetItem, QMessageBox
from ui.jobselect_dlg_ui import Ui_Dialog
from ui.job_item_widget import JobItemWidget, FileItemWidget

from shared.config_loader import CONFIG as cfg

BASE_DIR = cfg.BASE_DIR
# BASE_DIR = "/mnt/c/work/projects/intelijet_v2"
class JobSelectManager(QDialog):
    def __init__(self, parent=None, mode="label"):
        super().__init__(parent)
        self.result = (None, None)
        self.jobs_root = os.path.join(BASE_DIR, "data")
        # Load UI đã thiết kế
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.view_mode = mode  # "view" hoặc "label"

        # # Kết nối signal
        # self.ui.btnCreateJob.clicked.connect(self.add_job)
        self.load_jobs_from_disk()

        # self.ui.txtFilter.textChanged.connect(self.filter_jobs)

        # Connect button
        self.ui.btnAdd.released.connect(lambda: self.on_move_item(self.ui.lstJobItems, self.ui.lstJobCompare))
        self.ui.btnRemove.released.connect(lambda: self.on_move_item(self.ui.lstJobCompare, self.ui.lstJobItems))
        self.ui.buttonBox.accepted.connect(self.on_ok) 

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
        self.ui.lstJobCompare.clear()  
        try:
            files = [
                f for f in os.listdir(job_path)
            ]
        except Exception as e:
            return
        
        # Hiển thị lên lstJobDetail
        for f in files:
            item = QListWidgetItem(self.ui.lstJobItems)
            f_widget = FileItemWidget(f,job_path, view_mode='label')
            filepath = os.path.join(job_path, f)
            item.setSizeHint(f_widget.sizeHint())
            
            self.ui.lstJobItems.addItem(item)
            self.ui.lstJobItems.setItemWidget(item, f_widget)
            # f_widget.openSignal.connect(lambda item=item, filepath=filepath: self.on_file_opened(item, filepath))

    def on_move_item(self, soure, dest):
        current_item = soure.currentItem()


        if current_item is None:
            return
        current_item_widget = soure.itemWidget(current_item)
        f_widget = FileItemWidget(current_item_widget.filename, current_item_widget.filepath, view_mode='label')

        new_item  = QListWidgetItem(dest)
        new_item.setSizeHint(f_widget.sizeHint())
        dest.addItem(new_item)

        if f_widget is not None:
            dest.setItemWidget(new_item,f_widget)

            row = soure.row(current_item)
            soure.takeItem(row)

    def on_ok(self):
        item1 = item2 = None
        if self.ui.lstJobCompare.count() >= 2:
            w1 = self.ui.lstJobCompare.itemWidget(self.ui.lstJobCompare.item(0))
            w2 = self.ui.lstJobCompare.itemWidget(self.ui.lstJobCompare.item(1))
            if w1 is not None:
                item1 = getattr(w1, "fullpath", None)
            if w2 is not None:
                item2 = getattr(w2, "fullpath", None)

        self.result = {"file1": item1, "file2": item2}
        self.accept()  # chỉ cần gọi 1 lần

    def get_result(self):
        return self.result

jobcompare_dlg = JobSelectManager()
