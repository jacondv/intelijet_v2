from PyQt5.QtWidgets import QDialog, QPushButton, QLineEdit, QApplication, QTextEdit,QPlainTextEdit,QWidget
from PyQt5.QtCore import Qt, QTimer, QEvent
from ui.keyboard_ui import Ui_keyboard

class FullKeyboard(QDialog, Ui_keyboard):

    _instance = None   # lưu singleton

    @staticmethod
    def get_instance():
        if FullKeyboard._instance is None:
            FullKeyboard._instance = FullKeyboard()
        return FullKeyboard._instance

    def __init__(self, parent=None):
        if FullKeyboard._instance is not None:
            raise Exception("Use FullKeyboard.get_instance() instead of constructor.")
        
        super().__init__(parent)
        self.setupUi(self)
        self._current_widget = None
        self._shift_on = False
        self._init_signals()
        # self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.Window)
        self.setAttribute(Qt.WA_ShowWithoutActivating)

        self.SHIFT_MAP = {
            "1": "!", "2": "@", "3": "#", "4": "$", "5": "%",
            "6": "^", "7": "&", "8": "*", "9": "(", "0": ")"
        }

    # ------------------------------------------------------------
    def closeEvent(self, event):
        self.hide()
        
    # ------------------------------------------------------------
    # SIGNAL setup
    # ------------------------------------------------------------
    def _init_signals(self):
        for btn in self.findChildren(QPushButton):
            name = btn.objectName()

            if name.startswith("btn_") and name not in ["btn_Enter", "btn_Backspace", "btn_Shift", "btn_Hide"]:
                char = btn.text()
                btn.clicked.connect(lambda checked, c=char: self._insert(c))

        self.btn_Backspace.clicked.connect(self._backspace)
        self.btn_Enter.clicked.connect(self._enter)
        self.btn_Hide.clicked.connect(self._hide)
        self.btn_Shift.clicked.connect(self._toggle_shift)

    # ------------------------------------------------------------
    # CORE typing logic
    # ------------------------------------------------------------
# --- Chèn ký tự vào QLineEdit ---
    def _insert(self, char):
        if not self._current_widget:
            return

        # Nếu Shift bật, chèn ký tự shift map (chữ hoa hoặc ký tự đặc biệt)
        if self._shift_on:
        # mapping số -> ký tự đặc biệt
            shift_map = self.SHIFT_MAP
            if char in shift_map:
                char = shift_map[char]
            else:
                char = char.upper()
        else:
            char = char.lower()


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
            self._current_widget = None
        self.hide()

    # ------------------------------------------------------------
    # SHIFT logic (invert character mode)
    # ------------------------------------------------------------
    def _toggle_shift(self):
        self._shift_on = not self._shift_on

        for btn in self.findChildren(QPushButton):
            name = btn.objectName()
            if not name.startswith("btn_"):
                continue
            if name in ["btn_Enter", "btn_Backspace", "btn_Shift", "btn_Hide"]:
                continue

            txt = btn.text()

            if self._shift_on:
                # nếu là số thì đổi sang ký tự shift
                if txt in self.SHIFT_MAP:
                    btn.setText(self.SHIFT_MAP[txt])
                else:
                    btn.setText(txt.upper())
            else:
                # revert về giá trị gốc
                for k, v in self.SHIFT_MAP.items():
                    if txt == v:
                        txt = k
                        break
                btn.setText(txt.lower())

        self.btn_Shift.setStyleSheet(
            "background: #2c8bff; color: white;" if self._shift_on else ""
        )
    # ------------------------------------------------------------
    # ATTACH widget
    # ------------------------------------------------------------
    def attach(self, widget: QLineEdit):
        if self._current_widget == widget:
            return
        if not isinstance(widget, (QLineEdit, QTextEdit, QPlainTextEdit)):
            return
        self._current_widget = widget
        self.show()
        QTimer.singleShot(10, lambda: self._position(widget))
        
    # ------------------------------------------------------------
    # positioning logic
    # ------------------------------------------------------------
    def _position(self, widget):
        desktop = QApplication.desktop()
        screen_rect = desktop.availableGeometry(widget)

        # vị trí global của widget
        widget_rect = widget.geometry()
        widget_global_pos = widget.mapToGlobal(widget_rect.topLeft())
        widget_global_bottom = widget_global_pos.y() + widget_rect.height()

        keyboard_h = self.height()

        # khoảng trống phía dưới và phía trên
        space_below = screen_rect.bottom() - widget_global_bottom
        space_above = widget_global_pos.y() - screen_rect.top()

        # tính X (luôn căn giữa theo widget)
        x = widget_global_pos.x() + widget_rect.width()//2 - self.width()//2
        x = max(screen_rect.left(), min(x, screen_rect.right() - self.width()))

        # chọn vị trí: ưu tiên đặt dưới, nếu không đủ thì đặt lên trên
        if space_below >= keyboard_h:
            # đặt dưới widget
            y = widget_global_bottom + 5
        else:
            # đặt trên widget
            y = widget_global_pos.y() - keyboard_h - 5

        # giới hạn Y trong màn hình
        y = max(screen_rect.top(), min(y, screen_rect.bottom() - keyboard_h))
        self.move(x, y)



#How to use:

# Example usage in setting_page_manager.py:
    # in __init__ method of SettingPageManager
    # self.keyboard=FullKeyboard.get_instance()
    # for edit in self.findChildren(QWidget):
    #     edit.focusInEvent = lambda ev, w=edit: self.keyboard.attach(w)
    # ------------------------------------------------------------
    # Override eventFilter to hide keyboard on window deactivate
    # ------------------------------------------------------------
    # def eventFilter(self, obj, event):
    #     if event.type() == QEvent.WindowActivate:
    #         self.keyboard.hide()
    #         focused_widget = self.focusWidget()
    #         if focused_widget and isinstance(focused_widget, (QLineEdit, QTextEdit, QPlainTextEdit)):
    #             focused_widget.clearFocus()
    #             self.keyboard._current_widget = None
    #     return super().eventFilter(obj, event)
    

# How to use in any dialog:
# dialog = JobInfoDialog()
# attach_keyboard(dialog)
# dialog.show()
