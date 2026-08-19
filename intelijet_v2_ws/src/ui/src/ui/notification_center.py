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
import json
import os
import time
from collections import deque
from datetime import datetime

from PyQt5.QtCore import QObject, pyqtSignal

from shared.config_loader import CONFIG as cfg

MAX_HISTORY = 50
DEDUP_WINDOW_SECONDS = 5
PIN_SECONDS = 10

LOG_DIR = os.path.join(cfg.BASE_DIR, cfg.DATA_DIR, "logs")
LOG_DATE_FORMAT = "%Y%m%d"
MAX_LOG_FILES = 365


def log_path_for(date):
    """date: a datetime.date/datetime, or a string already in LOG_DATE_FORMAT."""
    if not isinstance(date, str):
        date = date.strftime(LOG_DATE_FORMAT)
    return os.path.join(LOG_DIR, f"{date}.jsonl")


def list_log_dates():
    """Dates (LOG_DATE_FORMAT strings) with an alarm log on disk, newest first."""
    if not os.path.isdir(LOG_DIR):
        return []
    dates = []
    for name in os.listdir(LOG_DIR):
        if name.endswith(".jsonl") and len(name) == len(LOG_DATE_FORMAT) + 2 + 6:
            dates.append(name[: -len(".jsonl")])
    return sorted(dates, reverse=True)


def _prune_old_logs():
    """Keep only the MAX_LOG_FILES most recent daily log files."""
    dates = list_log_dates()  # newest first
    for stale in dates[MAX_LOG_FILES:]:
        try:
            os.remove(log_path_for(stale))
        except OSError:
            pass


def read_log(date):
    """Items (oldest-first) persisted for one date. Empty list if none/unreadable."""
    path = log_path_for(date)
    items = []
    if not os.path.isfile(path):
        return items
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                items.append(json.loads(line))
            except ValueError:
                continue
    return items

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

        # Restore today's already-persisted notifications so a restart
        # (e.g. after the desktop icon's down+up relaunch) doesn't blank
        # out the ALARM tab / notification history mid-day.
        today = datetime.now().strftime(LOG_DATE_FORMAT)
        for item in read_log(today)[-max_history:]:
            self._history.append(item)

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
        self._append_to_log(item)
        self.notification_added.emit(item)
        self._try_update_label(message, level, now)
        return item

    def _append_to_log(self, item):
        """Persist one notification to today's data/logs/<YYYYMMDD>.jsonl file."""
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            path = log_path_for(datetime.fromtimestamp(item["timestamp"]).date())
            is_new_file = not os.path.isfile(path)
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(item) + "\n")
            if is_new_file:
                _prune_old_logs()
        except OSError:
            pass  # logging to disk must never break the live notification flow

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
