"""App-wide on-screen QWERTY keyboard for touch input.

Embedded as a plain CHILD WIDGET of whichever top-level window the field
being typed into belongs to (the main window, or a dialog) - not a
separate top-level window at all. Every previous design here was a
separate window (variously Qt.Tool, Qt.Popup, plain Qt.Window, with and
without WA_ShowWithoutActivating, with global click interception, with the
window discarded and recreated on every hide...) and each one hit a
different window-manager/compositor problem under WSLg: clicks not
delivered, an activation-stealing flicker loop, a stuck popup mouse grab,
hide() not actually unmapping the window. A plain child widget sidesteps
all of that at once: Qt handles stacking, input delivery and visibility
for its own child widgets entirely itself, with zero window-manager
involvement, so none of those failure modes are reachable anymore.

Typing goes straight into the focused QLineEdit/QTextEdit/QSpinBox via Qt
calls (no external process, e.g. the `onboard` binary this originally
shelled out to - dropped for the same class of window-manager-cooperation
reasons above).

Install once, application-wide:
    app.installEventFilter(TouchKeyboard())

Everything else - which widget to type into, which window to embed into,
where to place the keyboard inside it, making room if that window is too
small, switching to the symbols page, loading/saving its size - is
handled automatically from there.
"""
from types import SimpleNamespace

from PyQt5.QtCore import Qt, QObject, QEvent, QSize, QTimer
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import (
    QApplication, QDialog, QWidget, QStackedWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QPlainTextEdit, QAbstractSpinBox,
)

from shared.config_loader import CONFIG as cfg, save_config

DEFAULT_SIZE = (900, 320)
MIN_SIZE = (500, 220)
SAVE_SIZE_DEBOUNCE_MS = 600
RESIZE_MARGIN = 18

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
    border: 1px solid #3a4250;
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
QLabel#resize_handle {
    min-width: 16px;
    max-width: 16px;
    min-height: 16px;
    max-height: 16px;
    border-right: 2px solid #4a5566;
    border-bottom: 2px solid #4a5566;
    border-bottom-right-radius: 3px;
    margin: 2px 4px 2px 0;
}
"""


class OnScreenKeyboard(QWidget):
    """QWERTY keyboard. Don't construct directly - use show_for()/
    hide_current(), which take care of creating/embedding/discarding it
    in the right parent window."""

    _instance = None
    # Class-level (survives instance recreation): the dialog we've already
    # grown/repositioned to make room for the keyboard, so we don't redo
    # that on every keystroke in the same dialog.
    _prepared_window = None

    @classmethod
    def show_for(cls, widget):
        """Show the keyboard, embedded as a child of `widget`'s own
        top-level window, and route typing into it. `widget` may be a
        QLineEdit/QTextEdit/QPlainTextEdit, or a spin box (its internal
        line edit is used as the actual typing target)."""
        target = widget.lineEdit() if isinstance(widget, QAbstractSpinBox) else widget
        if not isinstance(target, QLineEdit):
            return

        window = widget.window()
        kb = cls._instance
        if kb is not None and kb.parent() is window and kb._target is target:
            return  # already showing, for this exact field, in this window

        cls.hide_current()
        kb = cls(window)
        cls._instance = kb
        kb._target = target
        kb._focus_widget = widget
        kb._attach_to_window(window, widget)
        kb.show()
        kb.raise_()

    @classmethod
    def hide_current(cls):
        if cls._instance is not None:
            cls._instance.hide()
            cls._instance.deleteLater()
            cls._instance = None

    @classmethod
    def current_focus_widget(cls):
        """The widget show_for() was last called with (e.g. a QSpinBox),
        or its internal QLineEdit if it's a plain text field - whichever a
        click should be considered "still part of the field being edited"
        rather than a click away that should close the keyboard. None if
        not currently showing."""
        return cls._instance._focus_widget if cls._instance is not None else None

    def __init__(self, parent):
        super().__init__(parent)
        self.setObjectName("OnScreenKeyboard")
        self.setMinimumSize(*MIN_SIZE)
        self.setStyleSheet(_PANEL_QSS)
        # A plain QWidget doesn't paint its QSS `background` on its own -
        # without this it stays transparent, showing whatever's behind it.
        self.setAttribute(Qt.WA_StyledBackground, True)

        self._target = None
        self._focus_widget = None
        self._shift_on = False
        self._buttons = {}

        # Drag-to-move / drag-to-resize state, implemented with plain Qt
        # mouse events (works the same for a child widget as it would for
        # a top-level window - no QSizeGrip/native dragging involved).
        self._drag_offset = None
        self._resize_origin = None
        self._resize_start_size = None

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

        handle = QLabel()
        handle.setObjectName("resize_handle")
        handle.setAttribute(Qt.WA_TransparentForMouseEvents)
        handle_row = QHBoxLayout()
        handle_row.addStretch(1)
        handle_row.addWidget(handle)
        outer.addLayout(handle_row)

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
        elif key == "Enter":
            # A real physical Enter key, while a QLineEdit inside a dialog
            # has focus, both emits returnPressed() and (via Qt's own
            # propagation to the dialog) triggers that dialog's default
            # button - e.g. confirms "New Project" without a separate tap
            # on its OK button. Dispatching a synthetic KeyEvent gets us
            # the exact same behavior for free, generically, for whatever
            # dialog happens to be open.
            target = self._target
            OnScreenKeyboard.hide_current()
            self._send_key(target, Qt.Key_Return)
        elif key == "Hide":
            self._target.clearFocus()
            OnScreenKeyboard.hide_current()
        else:
            self._insert(" " if key == "Space" else key)

    def _send_key(self, target, key):
        for event_type in (QEvent.KeyPress, QEvent.KeyRelease):
            QApplication.sendEvent(target, QKeyEvent(event_type, key, Qt.NoModifier))

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

    # ----- making room / placement -----

    def _attach_to_window(self, window, widget):
        """Make room for the keyboard inside `window` and place it: if the
        window has its own top-level layout (every dialog in this app
        does), add the keyboard as that layout's last row - it then
        occupies real, reserved space instead of floating on top of
        existing widgets, so it can never cover the field being typed
        into. Falls back to floating just below `widget` for windows with
        no such layout (the main window - already fullscreen, with plenty
        of room regardless)."""
        layout = window.layout() if isinstance(window, QDialog) else None
        if layout is not None:
            layout.addWidget(self)
            self._make_room_in(window)
        else:
            self._reposition(widget)

    def _make_room_in(self, window):
        """Grow `window` (once per dialog, not on every keystroke) so the
        keyboard just added to its layout actually fits, and move it to
        the top of the screen so there's room to grow downward into - a
        dialog centered on screen (Qt's default) has nowhere to grow."""
        if window is OnScreenKeyboard._prepared_window:
            return
        OnScreenKeyboard._prepared_window = window

        needed = window.layout().sizeHint()
        window.resize(max(window.width(), needed.width()), max(window.height(), needed.height()))

        screen = QApplication.desktop().availableGeometry(window)
        x = screen.left() + (screen.width() - window.width()) // 2
        y = screen.top() + 24
        window.move(x, y)

    def _reposition(self, widget):
        """Position just below `widget`, in the shared parent window's own
        coordinates (not global screen ones - this is a child widget)."""
        parent = self.parentWidget()
        below = widget.mapTo(parent, widget.rect().bottomLeft())

        x = below.x() + widget.width() // 2 - self.width() // 2
        x = max(0, min(x, parent.width() - self.width()))
        y = min(below.y() + 10, parent.height() - self.height())
        y = max(0, y)

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

    # ----- drag-to-move / drag-to-resize -----
    # Pressing in the bottom-right corner resizes; pressing anywhere else
    # on the keyboard's own background (i.e. not on a key button, which
    # consumes the press itself) moves it within the parent window.

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            return
        if self._at_resize_corner(event.pos()):
            self._resize_origin = event.globalPos()
            self._resize_start_size = self.size()
        else:
            self._drag_offset = event.pos()

    def mouseMoveEvent(self, event):
        parent = self.parentWidget()
        if self._resize_origin is not None:
            delta = event.globalPos() - self._resize_origin
            self.resize(
                max(MIN_SIZE[0], self._resize_start_size.width() + delta.x()),
                max(MIN_SIZE[1], self._resize_start_size.height() + delta.y()),
            )
        elif self._drag_offset is not None:
            new_pos = self.pos() + event.pos() - self._drag_offset
            x = max(0, min(new_pos.x(), parent.width() - self.width()))
            y = max(0, min(new_pos.y(), parent.height() - self.height()))
            self.move(x, y)

    def mouseReleaseEvent(self, event):
        self._drag_offset = None
        self._resize_origin = None

    def _at_resize_corner(self, pos):
        return pos.x() >= self.width() - RESIZE_MARGIN and pos.y() >= self.height() - RESIZE_MARGIN


class TouchKeyboard(QObject):
    """QApplication-wide event filter: shows/hides OnScreenKeyboard as
    focus moves in and out of text input widgets, application-wide."""

    def __init__(self):
        super().__init__()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.MouseButtonPress:
            # Don't wait for a FocusOut that may never come: clicking on a
            # widget/area that doesn't itself take Qt focus (a label, empty
            # space, a NoFocus button...) leaves the field's focus exactly
            # where it was as far as Qt is concerned, so no FocusOut ever
            # fires - the only reliable "user clicked away" signal is the
            # click itself. Close on any click that isn't on the keyboard
            # or on the field currently being typed into - which includes
            # that field's own built-in clear button (setClearButtonEnabled
            # makes it a real child widget of the QLineEdit): closing
            # mid-press there would abort that click's own press/release
            # pairing before it can register as a click at all.
            kb = OnScreenKeyboard._instance
            if kb is not None and isinstance(obj, QWidget) and not self._belongs_to_ui(kb, obj):
                OnScreenKeyboard.hide_current()
        elif event.type() == QEvent.FocusIn:
            QTimer.singleShot(50, self._sync)
        elif event.type() == QEvent.FocusOut:
            # A FocusOut caused by the whole app window losing/regaining OS
            # activation is not a real change of which field the user is
            # typing into - reacting to it would toggle the keyboard
            # open/closed on every activation change instead of only on a
            # genuine focus change.
            if event.reason() != Qt.ActiveWindowFocusReason:
                QTimer.singleShot(50, self._sync)
        return False

    def _belongs_to_ui(self, kb, obj):
        if obj is kb or kb.isAncestorOf(obj):
            return True
        focus_widget = OnScreenKeyboard.current_focus_widget()
        return focus_widget is not None and (obj is focus_widget or focus_widget.isAncestorOf(obj))

    def _sync(self):
        widget = QApplication.focusWidget()
        if isinstance(widget, (QLineEdit, QTextEdit, QPlainTextEdit, QAbstractSpinBox)):
            OnScreenKeyboard.show_for(widget)
        else:
            OnScreenKeyboard.hide_current()
