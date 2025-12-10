from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QMovie

class InlineLoading:
    """
    Overlay loading animation đè lên 1 widget cụ thể.
    Mỗi lần show() sẽ tạo mới widget overlay, hide() sẽ xóa.
    """
    def __init__(self, parent_widget, gif_path=":/icon/icon/loadding.gif", spinner_size=64):
        self.parent_widget = parent_widget
        self.gif_path = gif_path
        self.spinner_size = spinner_size
        self.overlay = None
        self.movie = None

    def show(self):
        self.overlay = QWidget(self.parent_widget)
        self.overlay.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.overlay.setAttribute(Qt.WA_NoSystemBackground, True)
        # self.overlay.setAttribute(Qt.WA_TranslucentBackground, True)
        self.overlay.setGeometry(self.parent_widget.rect())
        self.overlay.show()

        label = QLabel(self.overlay)
        label.setAlignment(Qt.AlignCenter)
        label.setFixedSize(self.spinner_size, self.spinner_size)
        label.move(
            (self.overlay.width() - self.spinner_size) // 2,
            (self.overlay.height() - self.spinner_size) // 2
        )
        label.setStyleSheet("background: transparent;")

        self.movie = QMovie(self.gif_path)
        self.movie.setScaledSize(label.size())
        label.setMovie(self.movie)
        self.movie.start()


        # nếu parent resize → overlay vẫn giữ kích thước (không tự động)
        # bạn có thể gắn overlay vào layout hoặc override resizeEvent của parent
        # label.setStyleSheet("background: rgb(255,0,0);")  # label trong suốt


    def hide(self):
        if self.movie and self.movie.isValid():
            self.movie.stop()
            self.movie = None
        if self.overlay:
            self.overlay.deleteLater()
            self.overlay = None


# How to use:
# self.loading = InlineLoading(self.ui.cloudFrame, gif_path=":/icon/icon/loadding.gif")
# self.loading.show()