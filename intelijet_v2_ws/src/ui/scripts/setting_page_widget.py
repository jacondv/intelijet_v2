# setting_page_widget.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from ui_setting_page import Ui_setting_page  # file UI của bạn

class SettingPageWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_setting_page()
        self.ui.setupUi(self)
