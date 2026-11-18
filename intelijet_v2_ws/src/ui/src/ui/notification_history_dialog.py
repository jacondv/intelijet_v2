# ui/notification_history_dialog.py
"""Simple, code-only (no .ui file) dialog listing recent notifications from
a NotificationCenter. Opened by clicking/tapping the notification label."""
import time

from PyQt5.QtWidgets import QDialog, QVBoxLayout, QListWidget, QListWidgetItem, QPushButton
from PyQt5.QtGui import QColor, QBrush

from ui.notification_center import LEVEL_COLORS


class NotificationHistoryDialog(QDialog):
    def __init__(self, notification_center, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Notification history")
        self.resize(520, 420)

        self._list = QListWidget(self)
        self._list.setStyleSheet("font-size: 14px;")

        close_btn = QPushButton("Close", self)
        close_btn.setMinimumHeight(48)
        close_btn.clicked.connect(self.accept)

        layout = QVBoxLayout(self)
        layout.addWidget(self._list)
        layout.addWidget(close_btn)

        self._notification_center = notification_center
        self._populate(notification_center.history())
        notification_center.notification_added.connect(self._on_notification_added)

    def done(self, result):
        # Avoid leaking a live connection back to this (about to be closed) dialog.
        try:
            self._notification_center.notification_added.disconnect(self._on_notification_added)
        except TypeError:
            pass
        super().done(result)

    def _populate(self, items):
        self._list.clear()
        for item in items:
            self._add_row(item)
        self._list.scrollToBottom()

    def _add_row(self, item):
        ts = time.strftime("%H:%M:%S", time.localtime(item["timestamp"]))
        level = item.get("level", "info")
        text = f"{ts} [{level.upper()}] {item['message']}"
        list_item = QListWidgetItem(text)
        list_item.setForeground(QBrush(QColor(LEVEL_COLORS.get(level, LEVEL_COLORS["unknown"]))))
        self._list.addItem(list_item)

    def _on_notification_added(self, item):
        self._add_row(item)
        self._list.scrollToBottom()
