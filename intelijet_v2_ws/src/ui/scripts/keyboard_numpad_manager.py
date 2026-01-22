from PyQt5.QtWidgets import QDialog, QPushButton, QLineEdit, QApplication
from PyQt5.QtCore import Qt
from ui.number_keyboard_ui import Ui_number_keyboard

class NumPadKeyboard(QDialog, Ui_number_keyboard):
    """Bàn phím số chuyên nghiệp, có thể attach bất kỳ QLineEdit nào"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self._current_widget = None  # widget đang nhận input
        self._init_signals()
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)

    def _init_signals(self):
        """Gán signal cho tất cả nút số và các nút chức năng"""
        for btn in self.findChildren(QPushButton):
            name = btn.objectName()
            if name.startswith("btn") and name not in ["btn_Enter", "btn_Backspace", "btn_Hide"]:
                char_map = {"btn_Minus": "-", "btn_Dot": "."}
                char = char_map.get(name, name[4:])
                btn.clicked.connect(lambda checked, c=char: self._insert_text(c))
        
        self.btn_Backspace.clicked.connect(self._backspace)
        self.btn_Enter.clicked.connect(self._enter)
        self.btn_Hide.clicked.connect(self._hide)

    # --- Các phương thức thao tác với textbox ---
    def _insert_text(self, char):
        if self._current_widget:
            self._current_widget.insert(char)

    def _backspace(self):
        if self._current_widget:
            text = self._current_widget.text()
            self._current_widget.setText(text[:-1])

    def _enter(self):
        if self._current_widget:
            self._current_widget.clearFocus()
        self.hide()

    def _hide(self):
        if self._current_widget:
            self._current_widget.clearFocus()
        self.hide()

    # --- Phương thức attach widget ---
    def attach(self, widget: QLineEdit):
        """Gọi khi textbox nhận focus"""
        self._current_widget = widget
        self._position_near(widget)
        self.show()
        self.raise_()

    def _position_near(self, widget):
        """Đặt bàn phím gần widget, tránh che nội dung"""
        desktop = QApplication.desktop()
        screen_rect = desktop.availableGeometry(widget)  # màn hình chứa widget
        pos = widget.mapToGlobal(widget.rect().center())
        
        # Tính x, y để bàn phím nằm gần widget và không ra ngoài màn hình
        x = min(pos.x() - self.width() // 2, screen_rect.right() - self.width())
        y = screen_rect.center().y() - self.height() // 2
        self.move(max(screen_rect.left(), x), max(screen_rect.top(), y))
