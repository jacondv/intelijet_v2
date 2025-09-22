import os

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QListWidgetItem, QMessageBox
from PyQt5.QtCore import pyqtSignal
from ui.jobsetting_page_ui import Ui_frmJobSetting
from shared.config_loader import CONFIG as cfg

BASE_DIR = cfg.BASE_DIR
# BASE_DIR = "/mnt/c/work/projects/intelijet_v2"
class JobNumberPageManager(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.jobs_root = os.path.join(BASE_DIR, "data")
        # Load UI đã thiết kế
        self.ui = Ui_frmJobSetting()
        self.ui.setupUi(self)

        # Kết nối signal
        self.ui.btnCreateJob.clicked.connect(self.add_job)
        self.load_jobs_from_disk()

        self.ui.txtFilter.textChanged.connect(self.filter_jobs)


    def setup_job_signals(self, job_widget, item, job_path):
        job_widget.job_deleted.connect(lambda name: self.delete_job(item, name, job_path))
        job_widget.job_renamed.connect(self.rename_job)


    def load_jobs_from_disk(self):
        for job_name in os.listdir(self.jobs_root):
            job_path = os.path.join(self.jobs_root, job_name)
            if os.path.isdir(job_path):
                item = QListWidgetItem(self.ui.lstJobnumber)
                job_widget = JobItemWidget(job_name)
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
            
class JobItemWidget(QWidget):
    job_renamed = pyqtSignal(str, str,object)   # old_name, new_name
    job_deleted = pyqtSignal(str)

    def __init__(self, job_name="", parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)

        self.original_name = job_name

        # Ô nhập tên Job (ban đầu disable)
        self.txtJobname = QLineEdit(job_name)
        self.txtJobname.setDisabled(True)
        self.txtJobname.setStyleSheet("""
            QLineEdit {
                border: None;
                background-color: white;
                color: black;
                padding-bottom:2px;
            }
            QLineEdit:disabled {
                border: None;
                background-color: #f0f0f0;
                color: gray;
            }
        """)
        layout.addWidget(self.txtJobname)

        # Nút Edit
        self.btnEdit = QPushButton("Edit")
        self.btnEdit.setStyleSheet(self._btn_style("#2196F3"))
        layout.addWidget(self.btnEdit)

        # Nút Save (ẩn ban đầu)
        self.btnSave = QPushButton("Save")
        self.btnSave.setStyleSheet(self._btn_style("#4CAF50"))
        self.btnSave.hide()
        layout.addWidget(self.btnSave)

        # Nút Cancel (ẩn ban đầu)
        self.btnCancel = QPushButton("Cancel")
        self.btnCancel.setStyleSheet(self._btn_style("#9E9E9E"))
        self.btnCancel.hide()
        layout.addWidget(self.btnCancel)

        # Nút Delete
        self.btnDelete = QPushButton("X")
        self.btnDelete.setStyleSheet(self._btn_style("#f44336"))
        layout.addWidget(self.btnDelete)

        # Kết nối sự kiện
        self.btnEdit.clicked.connect(self.enable_edit)
        self.btnSave.clicked.connect(self.save_edit)
        self.btnCancel.clicked.connect(self.cancel_edit)
        self.btnDelete.clicked.connect(lambda: self.job_deleted.emit(self.original_name))

    def _btn_style(self, color):
        """Tạo style cho nút với màu nền"""
        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                font-size: 12px;
                border-radius: 6px;
                padding: 2px 6px;
                min-width: 40px;
                min-height: 24px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
        """

    def enable_edit(self):
        """Chuyển sang chế độ edit"""
        self.original_name = self.txtJobname.text()
        self.txtJobname.setDisabled(False)
        self.txtJobname.setFocus()
        self.btnEdit.hide()
        self.btnSave.show()
        self.btnCancel.show()

    def save_edit(self):
        """Xác nhận thay đổi"""
        new_name = self.txtJobname.text().strip().replace(" ", "_")
        if new_name and new_name != self.original_name:
            self.job_renamed.emit(self.original_name, new_name, self)
            self.original_name = new_name

        self.txtJobname.setDisabled(True)
        self.btnSave.hide()
        self.btnCancel.hide()
        self.btnEdit.show()

    def rollback_name(self):
        """Khôi phục lại tên cũ nếu rename fail"""
        self.txtJobname.setText(self.original_name)

    def cancel_edit(self):
        """Hủy thay đổi và khôi phục giá trị cũ"""
        self.txtJobname.setText(self.original_name)
        self.txtJobname.setDisabled(True)
        self.btnSave.hide()
        self.btnCancel.hide()
        self.btnEdit.show()

