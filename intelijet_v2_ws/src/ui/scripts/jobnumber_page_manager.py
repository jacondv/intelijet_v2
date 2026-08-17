import os

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QListWidgetItem, QMessageBox
from PyQt5.QtCore import pyqtSignal
from ui.jobsetting_page_ui import Ui_frmJobSetting
from ui.job_item_widget import JobItemWidget
from ui.models.file_name import is_sync_junk
from shared.config_loader import CONFIG as cfg

BASE_DIR = cfg.BASE_DIR
# BASE_DIR = "/mnt/c/work/projects/intelijet_v2"
class JobNumberPageManager(QWidget):
    def __init__(self, parent=None, mode="view"):
        super().__init__(parent)
        self.jobs_root = os.path.join(BASE_DIR, "data")
        # Load UI đã thiết kế
        self.ui = Ui_frmJobSetting()
        self.ui.setupUi(self)
        self.view_mode = mode  # "view" hoặc "label"

        # Kết nối signal
        self.ui.btnCreateJob.clicked.connect(self.add_job)
        self.load_jobs_from_disk()

        self.ui.txtFilter.textChanged.connect(self.filter_jobs)


    def setup_job_signals(self, job_widget, item, job_path):
        job_widget.job_deleted.connect(lambda name: self.delete_job(item, name, job_path))
        job_widget.job_renamed.connect(self.rename_job)
        job_widget.clickedSignal.connect(lambda: self.on_item_selected(item, job_path))

    def on_item_selected(self, item, job_path):
        self.ui.lstJobnumber.setCurrentItem(item)

    def load_jobs_from_disk(self):
        self.ui.lstJobnumber.clear()
        mode=self.view_mode
        for job_name in os.listdir(self.jobs_root):
            if is_sync_junk(job_name):
                continue
            job_path = os.path.join(self.jobs_root, job_name)
            if os.path.isdir(job_path):
                item = QListWidgetItem(self.ui.lstJobnumber)
                job_widget = JobItemWidget(job_name)
                if mode == "label":
                    job_widget.show_only_label()
                item.setSizeHint(job_widget.sizeHint())
                self.ui.lstJobnumber.addItem(item)
                self.ui.lstJobnumber.setItemWidget(item, job_widget)
                self.setup_job_signals(job_widget, item, job_path)

    def get_selected_job(self):
        item = self.ui.lstJobnumber.currentItem()
        if item:
            # Lấy widget gắn với item
            job_widget = self.ui.lstJobnumber.itemWidget(item)  # trả về JobItemWidget

            # Lấy text từ QLineEdit
            selected_job_name = job_widget.txtJobname.text()
            return selected_job_name
        


    def add_job(self):
        job_name = self.ui.txtJobNumber.text().strip().replace(" ", "_")
        if not job_name:
            QMessageBox.warning(self, "⚠️ Warning", "Please enter Job Number before adding.")
            return

        # Tạo thư mục job
        job_path = os.path.join(self.jobs_root, job_name)
        if os.path.exists(job_path):
            QMessageBox.warning(self, "⚠️ Warning", f"Job '{job_name}' already exists.")
            return
        os.makedirs(job_path)

        # Thêm vào UI
        item = QListWidgetItem(self.ui.lstJobnumber)
        job_widget = JobItemWidget(job_name)
        item.setSizeHint(job_widget.sizeHint())
        self.ui.lstJobnumber.addItem(item)
        self.ui.lstJobnumber.setItemWidget(item, job_widget)

        self.ui.txtJobNumber.clear()
        self.setup_job_signals(job_widget, item, job_path)


    def rename_job(self, old_name, new_name,widget):
        old_path = os.path.join(self.jobs_root, old_name)
        new_path = os.path.join(self.jobs_root, new_name)

        if os.path.exists(new_path):
            widget.rollback_name()   # ⚠️ rollback UI
            QMessageBox.warning(self, "⚠️ Warning", f"Job '{new_name}' already exists.")
            return False

        try:
            os.rename(old_path, new_path)
            # QMessageBox.information(self, "Renamed", f"Job '{old_name}' → '{new_name}'")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to rename job: {e}")


    def delete_job(self, item, job_name, job_path):
        reply = QMessageBox.question(self, "Confirmation",
                                     f"Are you sure you want to delete job '{job_name}'?",
                                     QMessageBox.Yes | QMessageBox.No,
                                     QMessageBox.No)

        if reply == QMessageBox.Yes:
            # Xóa thư mục trên disk
            import shutil
            if os.path.exists(job_path):
                shutil.rmtree(job_path)

            # Xóa khỏi UI
            row = self.ui.lstJobnumber.row(item)
            removed_item = self.ui.lstJobnumber.takeItem(row)
            widget = self.ui.lstJobnumber.itemWidget(removed_item)
            if widget:
                widget.deleteLater()
            del removed_item


    def filter_jobs(self, text):
        text = text.strip().lower()
        for i in range(self.ui.lstJobnumber.count()):
            item = self.ui.lstJobnumber.item(i)
            widget = self.ui.lstJobnumber.itemWidget(item)
            job_name = widget.txtJobname.text().lower()
            item.setHidden(text not in job_name)

    # ----------------- Hàm ẩn / hiện -----------------
    def set_visible(self, visible: bool):
        """
        Ẩn hoặc hiện toàn bộ JobNumberPageManager
        :param visible: True để hiện, False để ẩn
        """
        self.setVisible(visible)

        # Nếu muốn tắt tương tác luôn khi ẩn
        for child in self.findChildren(QWidget):
            child.setEnabled(visible)
            
    def hide_all_delete_buttons(self):
        for i in range(self.ui.lstJobnumber.count()):
            item = self.ui.lstJobnumber.item(i)
            widget = self.ui.lstJobnumber.itemWidget(item)
            if widget and hasattr(widget, "btnDelete"):
                widget.btnDelete.hide()
            
