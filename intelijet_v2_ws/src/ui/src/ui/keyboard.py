"""App-wide on-screen keyboard, backed by the external `onboard` binary.

Earlier versions of this module launched a brand new `onboard` process on
every focus-in and killed it again on every focus-out. That's what caused
the flicker/lost-keystrokes/can't-move symptoms: each launch/kill is a
fresh X11 window being mapped and unmapped, and onboard grabs OS-level
window activation the moment its window is mapped - so every single
show/hide cycle fought the window manager for activation, which is also
exactly when it's most likely to drop injected keystrokes or fail to
register as draggable.

Fix: launch `onboard` ONCE and keep that same process (and its one X11
window) alive for the whole app session. Showing/hiding it after that goes
through onboard's own D-Bus service (`org.onboard.Onboard.Keyboard`,
methods Show/Hide) instead of process spawn/kill - the same lightweight
call onboard's own built-in auto-show feature uses internally. No new
window is ever created after the first launch, so there's nothing left to
fight the window manager for activation on every keystroke.

onboard is also configured once (via gsettings, best-effort - harmless if
gsettings/dconf isn't available) to float as a freely draggable/resizable
window (docking explicitly OFF) and stay above the app without needing to
fight for activation - the earlier "can't move it" complaint was really
the process-recreation problem above (a window that's destroyed and
recreated on every show never has a stable position to remember or drag
from), not something that needed docking to fix.

Note: onboard's OWN auto-show feature (show/hide driven by AT-SPI
accessibility events, no app-side code needed at all) would be the
better long-term fix, but requires Qt's AT-SPI accessibility bridge -
not present in this build of Qt5 (verified: no atspi/dbus linkage in
libQt5Gui, no accessibility bridge plugin). Revisit if a Qt5 build with
`-feature-accessibility-atspi-bridge` becomes available.
"""
from PyQt5.QtCore import Qt, QObject, QEvent, QTimer
from PyQt5.QtWidgets import (
    QLineEdit, QTextEdit,
    QSpinBox, QDoubleSpinBox,
    QApplication
)
import subprocess
import warnings

try:
    import dbus
except ImportError:
    dbus = None

ONBOARD_DBUS_NAME = "org.onboard.Onboard"
ONBOARD_DBUS_PATH = "/org/onboard/Onboard/Keyboard"
ONBOARD_DBUS_IFACE = "org.onboard.Onboard.Keyboard"

# gsettings applied once at startup, best-effort - free-floating (NOT
# docked: user can drag it anywhere/resize it), stays above the app
# without needing to fight for activation.
_ONBOARD_GSETTINGS = [
    ("org.onboard.window", "docking-enabled", "false"),
    ("org.onboard.window", "force-to-top", "true"),
    ("org.onboard.window", "window-decoration", "false"),
    ("org.onboard.auto-show", "hide-on-key-press", "false"),
]


def _configure_onboard():
    for schema, key, value in _ONBOARD_GSETTINGS:
        try:
            subprocess.run(
                ["gsettings", "set", schema, key, value],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=2,
            )
        except Exception:
            pass  # gsettings/dconf not available - onboard just uses its defaults


class TouchKeyboard(QObject):
    def __init__(self):
        super().__init__()
        self.proc = None
        self._dbus_keyboard = None
        _configure_onboard()

    def _ensure_running(self):
        """Launch onboard if it isn't already running. Only ever called
        once in practice - after the first launch self.proc stays alive
        for the rest of the app session."""
        if self.proc is not None and self.proc.poll() is None:
            return
        try:
            self.proc = subprocess.Popen(
                ["onboard"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except FileNotFoundError:
            warnings.warn("On-screen keyboard (onboard) is not installed.")
        except Exception as e:
            warnings.warn(f"Failed to launch onboard keyboard: {e}")

    def _dbus_iface(self):
        """The Show/Hide D-Bus proxy, connecting lazily (onboard needs a
        moment after launch before its service is on the bus) and
        reconnecting if the call below finds it stale."""
        if dbus is None:
            return None
        if self._dbus_keyboard is not None:
            return self._dbus_keyboard
        try:
            bus = dbus.SessionBus()
            proxy = bus.get_object(ONBOARD_DBUS_NAME, ONBOARD_DBUS_PATH)
            self._dbus_keyboard = dbus.Interface(proxy, ONBOARD_DBUS_IFACE)
        except dbus.DBusException:
            return None
        return self._dbus_keyboard

    def show_keyboard(self):
        self._ensure_running()
        iface = self._dbus_iface()
        if iface is None:
            return  # not installed, or onboard's D-Bus service not up yet
        try:
            iface.Show()
        except dbus.DBusException:
            self._dbus_keyboard = None  # stale proxy - reconnect next call

    def hide_keyboard(self):
        iface = self._dbus_iface()
        if iface is None:
            return
        try:
            iface.Hide()
        except dbus.DBusException:
            self._dbus_keyboard = None

    def has_input_focus(self):
        w = QApplication.focusWidget()
        return isinstance(
            w,
            (QLineEdit, QTextEdit, QSpinBox, QDoubleSpinBox)
        )

    def update_keyboard(self):
        if self.has_input_focus():
            self.show_keyboard()
        else:
            self.hide_keyboard()

    def eventFilter(self, obj, event):
        if event.type() == QEvent.FocusIn:
            # Delay nhẹ để Qt xử lý xong focus (fix dialog case)
            QTimer.singleShot(50, self.update_keyboard)
        elif event.type() == QEvent.FocusOut:
            # A FocusOut caused by the whole app window losing/regaining OS
            # activation (e.g. onboard's own window taking activation when
            # shown) is not a real change of which field the user is typing
            # into - only react to a genuine focus change. Safe to check
            # this now that show/hide are cheap D-Bus calls rather than a
            # process spawn/kill, so an occasional false positive here
            # costs nothing.
            if event.reason() != Qt.ActiveWindowFocusReason:
                QTimer.singleShot(50, self.update_keyboard)

        return False
