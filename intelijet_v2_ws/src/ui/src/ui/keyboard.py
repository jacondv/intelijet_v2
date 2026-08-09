"""App-wide on-screen QWERTY keyboard for touch input.

Self-contained: typing goes straight into the focused QLineEdit/QTextEdit/
QSpinBox via Qt calls, no external process and no window-manager
cooperation required. Replaces an earlier approach that shelled out to the
`onboard` binary - that depended on the window manager honoring onboard's
"don't take focus" hint, which the WSLg compositor does not do reliably:
onboard grabbing OS-level window activation caused a show/hide feedback
loop (every activation change toggled the keyboard, visible as constant
flicker) and, even once that loop was fixed, X11-injected keystrokes never
reliably landed in the target field. Keeping keystrokes entirely inside Qt
avoids both problems - and, for the same reason, this window intentionally
uses a plain top-level Window rather than Qt.Tool/FramelessWindowHint: a
non-activating "utility" window is exactly what stopped receiving mouse
clicks under WSLg.

Install once, application-wide:
    app.installEventFilter(TouchKeyboard())

Everything else - which widget to type into, where to place the keyboard,
switching to the symbols page, loading/saving its size - is handled
automatically from there.
"""
from types import SimpleNamespace

from PyQt5.QtCore import Qt, QObject, QEvent, QSize, QTimer, QPropertyAnimation, QEasingCurve
from PyQt5.QtWidgets import (
    QApplication, QDialog, QWidget, QStackedWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QSizeGrip, QLineEdit, QTextEdit, QPlainTextEdit, QAbstractSpinBox,
)

from shared.config_loader import CONFIG as cfg, save_config

DEFAULT_SIZE = (900, 320)
MIN_SIZE = (500, 220)
FADE_MS = 120
SAVE_SIZE_DEBOUNCE_MS = 600

LETTERS_PAGE, SYMBOLS_PAGE = 0, 1

# Each sub-list is one keyboard row, left to right.
_LETTER_ROWS = [
    list("1234567890"),
    list("qwertyuiop"),
    list("asdfghjkl"),
    ["Shift"] + list("zxcvbnm") + ["Backspace"],
    ["Sym", "Space", "Enter", "Hide"],
]
_SYMBOL_ROWS = [
    list("!@#$%^&*()"),
    list("-_=+[]{}\\|"),
    list(";:'\",.<>/?"),
    ["ABC", "~", "`", "Backspace"],
    ["Space", "Enter", "Hide"],
]

# Relative width of keys that aren't a single character (plain keys get 10).
_KEY_STRETCH = {"Shift": 16, "Backspace": 16, "Space": 50, "Enter": 18, "Hide": 14, "Sym": 14, "ABC": 14}
# Plain ASCII labels only - some minimal container images don't ship a font
# with full Unicode symbol coverage (arrows/glyphs render as tofu boxes).
_KEY_LABEL = {
    "Space": "", "Backspace": "<=", "Enter": "ENTER", "Hide": "HIDE",
    "Shift": "SHIFT", "Sym": "?123", "ABC": "ABC",
}
_SHIFT_DIGITS = {"1": "!", "2": "@", "3": "#", "4": "$", "5": "%",
                 "6": "^", "7": "&", "8": "*", "9": "(", "0": ")"}

_PANEL_QSS = """
OnScreenKeyboard {
    background: #232830;
}
QPushButton {
    background: #333b47;
    color: #eef1f5;
    border: 1px solid #414b59;
    border-radius: 6px;
    font-size: 15px;
    font-weight: 600;
}
QPushButton:pressed {
    background: #4a5566;
    border-color: #59657a;
}
QPushButton#key_Shift, QPushButton#key_Backspace, QPushButton#key_Sym,
QPushButton#key_ABC, QPushButton#key_Hide {
    color: #b9c2d0;
    font-size: 12px;
    letter-spacing: 0.5px;
}
QPushButton#key_Enter {
    background: #2c8bff;
    color: white;
    border-color: #2c8bff;
    font-size: 12px;
}
QPushButton#key_Enter:pressed {
    background: #1f6fd1;
}
"""


class OnScreenKeyboard(QDialog):
    """Singleton QWERTY keyboard. Get it via OnScreenKeyboard.instance()."""

    _instance = None

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Keyboard")
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.setMinimumSize(*MIN_SIZE)
        self.setStyleSheet(_PANEL_QSS)

        self._target = None
        self._shift_on = False
        self._buttons = {}
        self._popup_mode = False

        self._opacity_anim = QPropertyAnimation(self, b"windowOpacity", self)
        self._opacity_anim.setDuration(FADE_MS)
        self._opacity_anim.setEasingCurve(QEasingCurve.OutCubic)

        self._save_size_timer = QTimer(self)
        self._save_size_timer.setSingleShot(True)
        self._save_size_timer.setInterval(SAVE_SIZE_DEBOUNCE_MS)
        self._save_size_timer.timeout.connect(self._save_size)

        self._build_ui()
        self.resize(self._load_size())

    # ----- layout -----

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(4)

        self._stack = QStackedWidget(self)
        self._stack.addWidget(self._build_page(_LETTER_ROWS, self._buttons))
        self._stack.addWidget(self._build_page(_SYMBOL_ROWS, {}))
        outer.addWidget(self._stack)

        grip_row = QHBoxLayout()
        grip_row.addStretch(1)
        grip_row.addWidget(QSizeGrip(self))
        outer.addLayout(grip_row)

    def _build_page(self, rows, registry):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        for row in rows:
            row_layout = QHBoxLayout()
            row_layout.setSpacing(4)
            for key in row:
                row_layout.addWidget(self._make_key(key, registry), _KEY_STRETCH.get(key, 10))
            layout.addLayout(row_layout)
        return page

    def _make_key(self, key, registry):
        btn = QPushButton(_KEY_LABEL.get(key, key.upper()))
        btn.setObjectName(f"key_{key}")
        btn.setMinimumSize(40, 40)
        # Never take Qt focus away from the field being typed into.
        btn.setFocusPolicy(Qt.NoFocus)
        btn.clicked.connect(lambda checked, k=key: self._on_key(k))
        registry[key] = btn
        return btn

    # ----- typing -----

    def _on_key(self, key):
        if self._target is None:
            return
        if key == "Shift":
            self._toggle_shift()
        elif key == "Sym":
            self._stack.setCurrentIndex(SYMBOLS_PAGE)
        elif key == "ABC":
            self._stack.setCurrentIndex(LETTERS_PAGE)
        elif key == "Backspace":
            self._target.backspace()
        elif key in ("Enter", "Hide"):
            self._target.clearFocus()
            self.detach()
        else:
            self._insert(" " if key == "Space" else key)

    def _insert(self, char):
        if self._shift_on:
            char = _SHIFT_DIGITS.get(char, char.upper())
        self._target.insert(char)

    def _toggle_shift(self):
        self._shift_on = not self._shift_on
        self._buttons["Shift"].setStyleSheet(
            "background: #2c8bff; color: white;" if self._shift_on else ""
        )
        for key, btn in self._buttons.items():
            if len(key) == 1 and key.isalpha():
                btn.setText(key.upper() if self._shift_on else key.lower())
            elif key in _SHIFT_DIGITS:
                btn.setText(_SHIFT_DIGITS[key] if self._shift_on else key)

    # ----- attach / detach -----

    def attach(self, widget):
        """Show the keyboard next to `widget` and route typing into it.
        `widget` may be a QLineEdit/QTextEdit/QPlainTextEdit, or a spin box
        (its internal line edit is used as the actual typing target)."""
        target = widget.lineEdit() if isinstance(widget, QAbstractSpinBox) else widget
        if not isinstance(target, QLineEdit):
            return

        self._adapt_to_modal_state()

        same_target = target is self._target
        self._target = target
        if not same_target:
            self._reposition(widget)
        if not (same_target and self.windowOpacity() >= 0.99 and self.isVisible()):
            self._fade_to(1.0)

    def _adapt_to_modal_state(self):
        """QDialog.exec_() (used app-wide for every input dialog, default
        ApplicationModal) blocks mouse input to every other top-level window
        in the app - except popups, which Qt explicitly exempts from modal
        blocking (the same mechanism that lets a QComboBox dropdown work
        while opened from inside a modal dialog). Flip to a Popup window
        while a modal dialog is active so this keyboard keeps receiving
        clicks; use a normal window otherwise, since Popup auto-closes on
        any click outside it - unwanted friction when there's no modal
        dialog to work around."""
        modal_active = QApplication.activeModalWidget() is not None
        if modal_active == self._popup_mode:
            return
        self._popup_mode = modal_active
        was_visible = self.isVisible()
        self.setWindowFlags((Qt.Popup if modal_active else Qt.Window) | Qt.WindowStaysOnTopHint)
        if was_visible:
            self.show()

    def detach(self):
        self._target = None
        self._stack.setCurrentIndex(LETTERS_PAGE)
        self._fade_to(0.0)

    # ----- animation -----

    def _fade_to(self, opacity):
        if opacity > 0 and not self.isVisible():
            self.setWindowOpacity(0.0)
            self.show()
        self._opacity_anim.stop()
        self._opacity_anim.setStartValue(self.windowOpacity())
        self._opacity_anim.setEndValue(opacity)
        self._opacity_anim.start()
        if opacity <= 0:
            QTimer.singleShot(FADE_MS, self._finish_hide)

    def _finish_hide(self):
        # A re-attach may have happened while the fade-out was in flight.
        if self._target is None:
            self.hide()

    # ----- positioning -----

    def _reposition(self, widget):
        screen = QApplication.desktop().availableGeometry(widget)
        below = widget.mapToGlobal(widget.rect().bottomLeft())
        above = widget.mapToGlobal(widget.rect().topLeft())

        x = below.x() + widget.width() // 2 - self.width() // 2
        x = max(screen.left(), min(x, screen.right() - self.width()))

        if screen.bottom() - below.y() >= self.height() + 10:
            y = below.y() + 10
        else:
            y = above.y() - self.height() - 10
        y = max(screen.top(), min(y, screen.bottom() - self.height()))

        self.move(x, y)

    # ----- size persistence (survives to the next launch) -----

    def _load_size(self):
        saved = getattr(cfg, "keyboard", None)
        width = getattr(saved, "width", DEFAULT_SIZE[0])
        height = getattr(saved, "height", DEFAULT_SIZE[1])
        return QSize(max(width, MIN_SIZE[0]), max(height, MIN_SIZE[1]))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._save_size_timer.start()

    def _save_size(self):
        if not hasattr(cfg, "keyboard"):
            cfg.keyboard = SimpleNamespace()
        cfg.keyboard.width = self.width()
        cfg.keyboard.height = self.height()
        save_config(cfg)


class TouchKeyboard(QObject):
    """QApplication-wide event filter: attaches OnScreenKeyboard to whatever
    text input widget currently has focus, application-wide."""

    def __init__(self):
        super().__init__()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.FocusIn:
            QTimer.singleShot(50, self._sync)
        elif event.type() == QEvent.FocusOut:
            # A FocusOut caused by the whole app window losing/regaining OS
            # activation (e.g. this keyboard's own dialog being clicked) is
            # not a real change of which field the user is typing into -
            # reacting to it would toggle the keyboard open/closed forever.
            if event.reason() != Qt.ActiveWindowFocusReason:
                QTimer.singleShot(50, self._sync)
        return False

    def _sync(self):
        widget = QApplication.focusWidget()
        keyboard = OnScreenKeyboard.instance()
        if isinstance(widget, (QLineEdit, QTextEdit, QPlainTextEdit, QAbstractSpinBox)):
            keyboard.attach(widget)
        else:
            keyboard.detach()
