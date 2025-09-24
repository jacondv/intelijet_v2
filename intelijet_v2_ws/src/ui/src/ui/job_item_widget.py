from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QMessageBox, QLabel
from PyQt5.QtCore import pyqtSignal, QEvent


class JobItemWidget(QWidget):
    job_renamed = pyqtSignal(str, str,object)   # old_name, new_name
    job_deleted = pyqtSignal(str)
    clickedSignal = pyqtSignal() 

    def __init__(self, job_name="", parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)

        self.original_name = job_name

        # Ô nhập tên Job (ban đầu disable)
        self.txtJobname = QLineEdit(job_name)
        self.txtJobname.setReadOnly(True)
        self.txtJobname.setStyleSheet("""
            QLineEdit {
                border: None;
                margin: 5px;
                color: white;     
                padding: 2px;

            }                     
            QLineEdit:hover {
                padding-left: 8px;   
            }
                                      
        """)
        layout.addWidget(self.txtJobname)

        # Nút Edit
        self.btnEdit = QPushButton("Edit")
        self.btnEdit.setStyleSheet(self._btn_style("#2196F3"))
        self.btnEdit.hide()
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
        self.btnDelete.hide()        
        layout.addWidget(self.btnDelete)

        # Kết nối sự kiện
        self.btnEdit.clicked.connect(self.enable_edit)
        self.btnSave.clicked.connect(self.save_edit)
        self.btnCancel.clicked.connect(self.cancel_edit)
        self.btnDelete.clicked.connect(lambda: self.job_deleted.emit(self.original_name))

        self.txtJobname.installEventFilter(self)

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


    def eventFilter(self, obj, event):
        if obj is self.txtJobname and event.type() == QEvent.MouseButtonPress:
            self.clickedSignal.emit()
            return True   # chặn event, không để QLineEdit xử lý nữa
        return super().eventFilter(obj, event)
    
    def enable_edit(self):
        """Chuyển sang chế độ edit"""
        self.original_name = self.txtJobname.text()
        self.txtJobname.setReadOnly(False)
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


class FileItemWidget(QWidget):
    openSignal = pyqtSignal()
    def __init__(self, filename, filepath, parent=None):
        super().__init__(parent)
        self.filepath = filepath

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0) 

        self.label = QLabel(filename)
        self.label.setStyleSheet("""
                        QLabel {
                            color: white;
                        }

                    """)
        layout.addWidget(self.label)

        self.btn_open = QPushButton("Open")
        self.btn_open.setStyleSheet("""
                        QPushButton {
                            border: 2px solid #005a9e;   /* màu border */
                            border-radius: 0px;       /* bo góc */
                            background-color: #0078d7;   /* màu nền */
                        }
                        QPushButton:hover {
                            background-color: #005a9e;   /* khi hover */
                        }
                        QPushButton:pressed {
                            background-color: #004578;   /* khi nhấn */
                        }

                    """)
        self.btn_open.setFixedSize(80, 35)  # width=60, height=25
        layout.addWidget(self.btn_open)

        # connect button
        self.btn_open.clicked.connect(self.on_open_clicked)
        

    
    def on_open_clicked(self):
        self.openSignal.emit()
