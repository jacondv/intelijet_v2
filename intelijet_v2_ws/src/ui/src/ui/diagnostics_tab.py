# ui/diagnostics_tab.py
"""Diagnostics/Alarm tab (HMI-style): a full, filterable log of every
notification pushed to a NotificationCenter for as long as the app has
been running. Read-only by design - no acknowledge/clear (see
docs/plan/phase_11_pps_error_logging_diagnostics_tab.md)."""
import time

from PyQt5.QtGui import QColor, QBrush
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QLineEdit,
    QTableWidget, QTableWidgetItem, QHeaderView,
)

from ui.notification_center import LEVEL_COLORS

LEVEL_FILTERS = ["All", "Info", "Warning", "Error"]
ALL_SOURCES = "All"
COLUMNS = ["Timestamp", "Level", "Source", "Message"]


class DiagnosticsTab(QWidget):
    def __init__(self, notification_center, parent=None):
        super().__init__(parent)
        self._notification_center = notification_center
        self._all_items = list(notification_center.history())
        self._sources = sorted({item["source"] for item in self._all_items})

        self._build_ui()
        self._rebuild_table()

        notification_center.notification_added.connect(self._on_notification_added)

    # ----------------- UI -----------------
    def _build_ui(self):
        filter_row = QHBoxLayout()

        filter_row.addWidget(QLabel("Level:"))
        self._level_filter = QComboBox(self)
        self._level_filter.addItems(LEVEL_FILTERS)
        self._level_filter.currentIndexChanged.connect(self._rebuild_table)
        filter_row.addWidget(self._level_filter)

        filter_row.addWidget(QLabel("Source:"))
        self._source_filter = QComboBox(self)
        self._source_filter.addItem(ALL_SOURCES)
        self._source_filter.addItems(self._sources)
        self._source_filter.currentIndexChanged.connect(self._rebuild_table)
        filter_row.addWidget(self._source_filter)

        filter_row.addWidget(QLabel("Search:"))
        self._search_box = QLineEdit(self)
        self._search_box.setPlaceholderText("Filter by message...")
        self._search_box.textChanged.connect(self._rebuild_table)
        filter_row.addWidget(self._search_box, 1)

        self._table = QTableWidget(0, len(COLUMNS), self)
        self._table.setHorizontalHeaderLabels(COLUMNS)
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        header = self._table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)

        layout = QVBoxLayout(self)
        layout.addLayout(filter_row)
        layout.addWidget(self._table)

    # ----------------- Data -----------------
    def _on_notification_added(self, item):
        self._all_items.append(item)
        if item["source"] not in self._sources:
            self._sources.append(item["source"])
            self._source_filter.blockSignals(True)
            self._source_filter.addItem(item["source"])
            self._source_filter.blockSignals(False)
        if self._matches_filter(item):
            self._append_row(item)
            self._table.scrollToBottom()

    def _matches_filter(self, item):
        level_choice = self._level_filter.currentText()
        if level_choice != "All" and item.get("level", "info").lower() != level_choice.lower():
            return False
        source_choice = self._source_filter.currentText()
        if source_choice != ALL_SOURCES and item.get("source", "") != source_choice:
            return False
        search_text = self._search_box.text().strip().lower()
        if search_text and search_text not in item.get("message", "").lower():
            return False
        return True

    def _rebuild_table(self, *_args):
        self._table.setRowCount(0)
        for item in self._all_items:
            if self._matches_filter(item):
                self._append_row(item)
        self._table.scrollToBottom()

    def _append_row(self, item):
        row = self._table.rowCount()
        self._table.insertRow(row)

        ts = time.strftime("%H:%M:%S", time.localtime(item["timestamp"]))
        level = item.get("level", "info")
        color = QColor(LEVEL_COLORS.get(level, LEVEL_COLORS["unknown"]))

        values = [ts, level.upper(), item.get("source", ""), item.get("message", "")]
        for col, value in enumerate(values):
            cell = QTableWidgetItem(value)
            cell.setForeground(QBrush(color))
            self._table.setItem(row, col, cell)
