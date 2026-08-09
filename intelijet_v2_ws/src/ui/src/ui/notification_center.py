# ui/notification_center.py
"""Central place for operator-facing notifications and device status colors.

NotificationCenter has no ROS dependency - it only receives (source, message,
level) pushes from whoever is listening to ROS/device state, so it can be
unit-tested standalone (see ui/tests/test_notification_center.py).

Two ways to feed it:
  - push(source, message, level):  adds to history (deduped, capped) and
    updates the "current label" text, subject to the pin rule below.
  - push_transient(message, level): updates the "current label" text only,
    never added to history. Used for high-frequency progress ticks that
    would otherwise spam the history list.

Pin rule: an "error" push pins the label for PIN_SECONDS - an "info" arriving
during that window is dropped (label unchanged); a "warning" or another
"error" can still override it immediately.
"""
import time
from collections import deque

from PyQt5.QtCore import QObject, pyqtSignal

MAX_HISTORY = 50
DEDUP_WINDOW_SECONDS = 5
PIN_SECONDS = 10

LEVEL_COLORS = {
    "info": "#2ecc71",     # green
    "warning": "#f39c12",  # amber
    "error": "#e74c3c",    # red
    "unknown": "#95a5a6",  # gray
}


def set_device_label(label, state):
    """Set a device status QLabel's text + color based on its connection
    state (DeviceStatus.CONNECTED / DISCONNECTED / anything else -> unknown)."""
    state_text = (state or "UNKNOWN").upper()
    if state_text == "CONNECTED":
        color = LEVEL_COLORS["info"]
    elif state_text == "DISCONNECTED":
        color = LEVEL_COLORS["error"]
    else:
        color = LEVEL_COLORS["unknown"]
    label.setText(state_text)
    label.setStyleSheet(f"color: white; background-color: {color}; padding: 2px 6px; border-radius: 3px;")


class NotificationCenter(QObject):
    # Emitted for every item added to history: dict(timestamp, level, source, message)
    notification_added = pyqtSignal(object)
    # Emitted whenever the "current label" text should change (history or transient)
    label_changed = pyqtSignal(str, str)  # (text, level)

    def __init__(self, max_history=MAX_HISTORY, dedup_window=DEDUP_WINDOW_SECONDS,
                 pin_seconds=PIN_SECONDS, parent=None):
        super().__init__(parent)
        self._max_history = max_history
        self._dedup_window = dedup_window
        self._pin_seconds = pin_seconds
        self._history = deque(maxlen=max_history)
        self._pinned_until = 0.0
        self._pinned_level = None

    def history(self):
        """Return items oldest-first."""
        return list(self._history)

    def push(self, source, message, level="info"):
        """Add a persistent notification (kept in history) and try to update
        the current label. Returns the item dict."""
        now = time.time()

        for item in reversed(self._history):
            if item["source"] == source and item["message"] == message:
                if now - item["timestamp"] < self._dedup_window:
                    item["timestamp"] = now
                    self._try_update_label(message, level, now)
                    return item
                break

        item = {"timestamp": now, "level": level, "source": source, "message": message}
        self._history.append(item)
        self.notification_added.emit(item)
        self._try_update_label(message, level, now)
        return item

    def push_transient(self, message, level="info"):
        """Update the current label without touching history."""
        self._try_update_label(message, level, time.time())

    def _try_update_label(self, message, level, now):
        if now < self._pinned_until and level == "info":
            return  # an error is still pinned, drop this info-level update
        if level == "error":
            self._pinned_until = now + self._pin_seconds
            self._pinned_level = "error"
        self.label_changed.emit(message, level)
