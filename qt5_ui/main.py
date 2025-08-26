from PyQt5 import QtWidgets, uic
import sys

class MainWindow(QtWidgets.QFrame):  # <-- vì main.ui là QFrame
    def __init__(self):
        super().__init__()

        # Load UI chính từ file main.ui
        uic.loadUi("pps.ui", self)

        setting_page_widget = uic.loadUi("setting_page.ui")  # không truyền self
        container = self.tab_setting
        if container.layout() is None:
            container.setLayout(QtWidgets.QVBoxLayout())
        container.layout().addWidget(setting_page_widget)

# ====== Khởi chạy App ======
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()   # hiện QFrame ra màn hình
    sys.exit(app.exec_())
