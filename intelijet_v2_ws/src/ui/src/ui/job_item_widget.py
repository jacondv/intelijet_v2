from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QMessageBox, QLabel, QVBoxLayout
from PyQt5.QtCore import pyqtSignal, QEvent
from ui.job_item_ui import Ui_Form
from ui.file_item_detail import Ui_frmFileItemDetail
from ui.models.file_name import parse_filename as _parse_filename
import os
from datetime import datetime


class JobItemWidget(QWidget, Ui_Form):
    job_renamed = pyqtSignal(str, str, object)
    job_deleted = pyqtSignal(str)
    job_selected = pyqtSignal(str)
    clickedSignal = pyqtSignal()

    def __init__(self, job_name="", parent=None):
        super().__init__(parent)
        self.setupUi(self)   # nạp layout từ file job_item_ui.py

        self.original_name = job_name
        self.txtJobname.setText(job_name)

        # kết nối sự kiện
        self.btnEdit.clicked.connect(self.enable_edit)
        self.btnSave.clicked.connect(self.save_edit)
        self.btnCancel.clicked.connect(self.cancel_edit)
        self.btnDelete.clicked.connect(lambda: self.job_deleted.emit(self.original_name))
        self.btnSelect.clicked.connect(self.job_selected.emit)

        # Ẩn các nút khi load
        self.show_view_mode()

        self.txtJobname.installEventFilter(self)

    def show_edit_mode(self):
        """Chế độ edit: hiện Save, Cancel; ẩn Edit, Delete"""
        self.btnEdit.hide()
        self.btnDelete.hide()
        self.btnSave.show()
        self.btnCancel.show()
        self.txtJobname.setReadOnly(False)
        self.txtJobname.setFocus()
        self.btnSelect.hide()

    def show_view_mode(self):
        """Chế độ xem: hiện Edit, Delete; ẩn Save, Cancel"""
        self.btnSave.hide()
        self.btnCancel.hide()
        self.btnEdit.show()
        self.btnDelete.show()
        self.txtJobname.setReadOnly(True)
        self.btnSelect.hide()

    def show_only_label(self):
        """Chỉ hiện tên job, ẩn toàn bộ nút"""
        self.btnEdit.hide()
        self.btnSave.hide()
        self.btnCancel.hide()
        self.btnDelete.hide()
        self.txtJobname.setReadOnly(True)
        self.btnSelect.hide()

    def eventFilter(self, obj, event):
        if obj is self.txtJobname and event.type() == QEvent.MouseButtonPress:
            self.clickedSignal.emit()
            return True   # chặn event, không để QLineEdit xử lý nữa
        return super().eventFilter(obj, event)
    
    def enable_edit(self):
        """Chuyển sang chế độ edit"""
        self.original_name = self.txtJobname.text()
        self.show_edit_mode()

    def save_edit(self):
        """Xác nhận thay đổi"""
        new_name = self.txtJobname.text().strip().replace(" ", "_")
        if new_name and new_name != self.original_name:
            self.job_renamed.emit(self.original_name, new_name, self)
            self.original_name = new_name

        self.show_view_mode()

    def rollback_name(self):
        """Khôi phục lại tên cũ nếu rename fail"""
        self.txtJobname.setText(self.original_name)

    def cancel_edit(self):
        """Hủy thay đổi và khôi phục giá trị cũ"""
        self.txtJobname.setText(self.original_name)
        self.show_view_mode()


class FileItemWidget(QWidget):

    openSignal = pyqtSignal()

    def __init__(self, filename, filepath, parent=None, view_mode="auto"):
        super().__init__(parent)
        self.ui = Ui_frmFileItemDetail()
        self.ui.setupUi(self) 
        self.filepath = filepath
        self.filename = filename
        self.fullpath = os.path.join(self.filepath, self.filename)

        filename_info = self.parse_filename(filename=filename)

        self.ui.lblRow1.setText(filename_info['jobname'])
        self.ui.lblRow2.setText(filename_info['name'])
        self.ui.lblRow3.setText(filename_info['datetime'])
        self.ui.btnOpen.clicked.connect(self.on_open_clicked)
        
        if view_mode.lower() == 'label':
            self.show_only_label()
        elif view_mode.lower()=='2':
            self.show_only_checkbox()
        elif  view_mode.lower()=='3':
             self.show_only_button()
        else:
            self.show_default()
            
        

    def show_only_label(self):
        self.ui.btnOpen.hide()
        self.ui.chkChooseCloud.hide()
        
    def show_only_checkbox(self):
        self.ui.btnOpen.hide()
        self.ui.chkChooseCloud.show()
    
    def show_only_button(self):
        self.ui.btnOpen.show()
        self.ui.chkChooseCloud.hide()

    def show_default(self):
        self.ui.btnOpen.show()
        self.ui.chkChooseCloud.show()

    def parse_filename_old(self,filename: str):
        """
        Phân tích filename dạng: Jobname#yyyymmdd_hhmmss#name.ply
        Trả về dict chứa jobname, datetime, name và original filename.
        """

        def format_name(name: str) -> str:
            name_lower = name.lower()
            import re
            match = re.search(r"(\d+)$", name)
            if "pre" in name_lower:
                # tách số cuối
                number = match.group(1) if match else ""
                return f"PRESCAN({number})"
            elif "post" in name_lower: 
                number = match.group(1) if match else ""
                return f"POSTSCAN({number})"     
            elif "compare" in name_lower: 
                number = match.group(1) if match else ""
                return f"COMPARE({number})"         
            else:
                return name  # giữ nguyên nếu không có "pre"
            
        base = os.path.basename(filename)              # Lấy tên file (bỏ đường dẫn)
        name_no_ext, _ = os.path.splitext(base)        # Bỏ phần .ply
        
        parts = name_no_ext.split("#")
        jobname, datetime_raw, name = parts[:3] if len(parts) == 3 else ("Unknown", "Unknown", name_no_ext)
               
        name = format_name(name)
        
            
        try:
            dt = datetime.strptime(datetime_raw, "%Y%m%d_%H%M%S")
            datetime_str = dt.strftime("%d/%m/%Y %H:%M:%S")
            datetime_str = f"{datetime_str}"
        except ValueError:
            datetime_str = datetime_raw  # nếu lỗi định dạng, giữ nguyên

        return {
            "jobname": jobname,
            "name": name,
            "datetime": datetime_str,
        }
        
    def parse_filename(self, filename: str):
        
        info = _parse_filename(filename)
        
        if 'pre_scan' in info['type']:
            name = f"[{info['scan_id']}]PRESCAN"
        elif 'post_scan' in info['type']:
            name = f"[{info['scan_id']}]POSTSCAN({info['index']})"
        elif 'compared' in info['type']:
            name = f"[{info['scan_id']}]COMPARE({info['index']})"
        elif 'report' in info['type']:
            name = f"[{info['scan_id']}]FINALREPORT"
        else:
            name = f"[{info['scan_id']}]{info['type'].upper()}({info['index']})"

        result =  {
            "jobname": info['job'],
            "name": name,
            "datetime": info['timestamp'],
        }
      
        return result




    def on_open_clicked(self):
        self.openSignal.emit()



class FolderItemWidget(QWidget, Ui_Form):
    folder_renamed = pyqtSignal(str, str, object)
    folder_deleted = pyqtSignal(str)
    folder_selected = pyqtSignal(str)
    clickedSignal = pyqtSignal()

    def __init__(self, folder_name="", parent=None):
        super().__init__(parent)
        self.setupUi(self)   # nạp layout từ file job_item_ui.py

        self.original_name = folder_name
        self.txtName.setText(folder_name)

        # kết nối sự kiện
        self.btnEdit.clicked.connect(self.enable_edit)
        self.btnSave.clicked.connect(self.save_edit)
        self.btnCancel.clicked.connect(self.cancel_edit)
        self.btnDelete.clicked.connect(lambda: self.folder_deleted.emit(self.original_name))
        self.btnSelect.clicked.connect(self.folder_selected.emit)

        # Ẩn các nút khi load
        self.show_view_mode()

        self.txtName.installEventFilter(self)

    def show_edit_mode(self):
        """Chế độ edit: hiện Save, Cancel; ẩn Edit, Delete"""
        self.btnEdit.hide()
        self.btnDelete.hide()
        self.btnSave.show()
        self.btnCancel.show()
        self.txtName.setReadOnly(False)
        self.txtName.setFocus()
        self.btnSelect.hide()

    def show_view_mode(self):
        """Chế độ xem: hiện Edit, Delete; ẩn Save, Cancel"""
        self.btnSave.hide()
        self.btnCancel.hide()
        self.btnEdit.show()
        self.btnDelete.show()
        self.txtName.setReadOnly(True)
        self.btnSelect.hide()

    def show_only_label(self):
        """Chỉ hiện tên job, ẩn toàn bộ nút"""
        self.btnEdit.hide()
        self.btnSave.hide()
        self.btnCancel.hide()
        self.btnDelete.hide()
        self.txtName.setReadOnly(True)
        self.btnSelect.hide()

    def eventFilter(self, obj, event):
        if obj is self.txtName and event.type() == QEvent.MouseButtonPress:
            self.clickedSignal.emit()
            return True   # chặn event, không để QLineEdit xử lý nữa
        return super().eventFilter(obj, event)
    
    def enable_edit(self):
        """Chuyển sang chế độ edit"""
        self.original_name = self.txtName.text()
        self.show_edit_mode()

    def save_edit(self):
        """Xác nhận thay đổi"""
        new_name = self.txtName.text().strip().replace(" ", "_")
        if new_name and new_name != self.original_name:
            self.folder_renamed.emit(self.original_name, new_name, self)
            self.original_name = new_name

        self.show_view_mode()

    def rollback_name(self):
        """Khôi phục lại tên cũ nếu rename fail"""
        self.txtName.setText(self.original_name)

    def cancel_edit(self):
        """Hủy thay đổi và khôi phục giá trị cũ"""
        self.txtName.setText(self.original_name)
        self.show_view_mode()