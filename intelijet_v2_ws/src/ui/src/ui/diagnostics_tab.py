# ui/diagnostics_tab.py
"""Diagnostics/Alarm tab (HMI-style): a full, filterable log of every
notification pushed to a NotificationCenter for as long as the app has
been running. Read-only by design - no acknowledge/clear (see
docs/plan/phase_11_pps_error_logging_diagnostics_tab.md).

Visual design follows the "HMI System Alarm & Event Log" light industrial
mock-up: a light header bar, a card-style filter toolbar with a caption
label above each control, and a striped log table with colored level
badges and a tinted row for WARNING/ERROR entries."""
import time
from datetime import date as date_cls

from PyQt5.QtCore import QDate, Qt
from PyQt5.QtGui import QColor, QBrush
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QComboBox,
    QLineEdit, QCalendarWidget, QTableWidget, QTableWidgetItem, QHeaderView,
    QDialog, QPushButton, QScrollArea, QFrame,
)

from ui.notification_center import LEVEL_COLORS, LOG_DATE_FORMAT, read_log

LEVEL_FILTERS = ["All Levels", "Info", "Warning", "Error"]
ALL_SOURCES = "All Sources"
COLUMNS = ["Timestamp", "Level", "Source", "Message"]

ROW_HEIGHT = 64
TABLE_FONT_SIZE = 24
DIALOG_FONT_SIZE = 32

# ---- HMI light theme palette (matches the app's existing industrial
# blue/white theme - see intelijet_ui.py - rather than introducing a new
# color scheme) ----
BG_PAGE = "#EAF2F8"
BG_CARD = "#FFFFFF"
BG_CARD_ALT = "#F4F8FB"
BG_HEADER = "#FFFFFF"
BORDER = "#C7D6E3"
TEXT_PRIMARY = "#1F2D3A"
TEXT_MUTED = "#5C7080"
ACCENT = "#2F4F6E"
LEVEL_ERROR = "#E74C3C"
LEVEL_WARN = "#F39C12"
LEVEL_INFO = "#22A559"
ROW_ERROR_BG = "#FDECEA"
ROW_WARN_BG = "#FFF6E5"
BADGE_ERROR_BG = "#FBDEDB"
BADGE_WARN_BG = "#FDECD1"
BADGE_INFO_BG = "#DAF3E4"

BADGE_COLORS = {"error": LEVEL_ERROR, "warning": LEVEL_WARN, "info": LEVEL_INFO}
BADGE_BG_COLORS = {"error": BADGE_ERROR_BG, "warning": BADGE_WARN_BG, "info": BADGE_INFO_BG}
ROW_TINTS = {"error": ROW_ERROR_BG, "warning": ROW_WARN_BG}


class DiagnosticsTab(QWidget):
    def __init__(self, notification_center, parent=None):
        super().__init__(parent)
        self._notification_center = notification_center
        self._today = date_cls.today().strftime(LOG_DATE_FORMAT)
        self._viewing_date = self._today  # date currently shown; only "today" gets live updates
        self._all_items = list(notification_center.history())
        self._sources = sorted({item["source"] for item in self._all_items})

        self._build_ui()
        self._rebuild_table()

        notification_center.notification_added.connect(self._on_notification_added)

    # ----------------- UI -----------------
    def _build_ui(self):
        self.setStyleSheet(f"QWidget {{ background-color: {BG_PAGE}; color: {TEXT_PRIMARY}; }}")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(14)

        layout.addWidget(self._build_header())
        layout.addWidget(self._build_filter_toolbar())
        layout.addWidget(self._build_table(), 1)

    def _build_header(self):
        header = QFrame(self)
        header.setStyleSheet(
            f"QFrame {{ background-color: {BG_HEADER}; border: 1px solid {BORDER};"
            f" border-left: 6px solid {ACCENT}; }}"
        )
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(20, 14, 20, 14)
        title = QLabel("HMI System Alarm & Event Log", header)
        title.setStyleSheet(f"font-size: 28px; font-weight: bold; color: {TEXT_PRIMARY}; border: none;")
        header_layout.addWidget(title)
        header_layout.addStretch(1)
        return header

    @staticmethod
    def _field_control_style():
        return (
            f"QPushButton, QComboBox, QLineEdit {{"
            f"    background-color: {BG_CARD_ALT};"
            f"    color: {TEXT_PRIMARY};"
            f"    border: 1px solid {BORDER};"
            f"    border-radius: 0px;"
            f"    padding: 8px 12px;"
            f"    font-size: 22px;"
            f"}}"
            f"QPushButton:hover, QComboBox:hover {{ border: 1px solid {ACCENT}; }}"
            f"QComboBox::drop-down {{ border: none; width: 36px; }}"
            f"QComboBox QAbstractItemView {{"
            f"    background-color: {BG_CARD_ALT}; color: {TEXT_PRIMARY};"
            f"    selection-background-color: {ACCENT}; selection-color: #FFFFFF; font-size: 20px;"
            f"}}"
        )

    def _labeled_field(self, caption, control):
        box = QVBoxLayout()
        box.setSpacing(6)
        label = QLabel(caption)
        label.setStyleSheet(f"font-size: 16px; color: {TEXT_MUTED}; border: none;")
        box.addWidget(label)
        box.addWidget(control)
        return box

    def _build_filter_toolbar(self):
        card = QFrame(self)
        card.setStyleSheet(f"QFrame {{ background-color: {BG_CARD}; border: 1px solid {BORDER}; }}")
        grid = QGridLayout(card)
        grid.setContentsMargins(20, 18, 20, 18)
        grid.setHorizontalSpacing(24)
        grid.setVerticalSpacing(14)

        control_style = self._field_control_style()

        # Date
        self._selected_date = QDate.currentDate()
        self._date_button = QPushButton(self._selected_date.toString("dd/MM/yyyy"), card)
        self._date_button.setMinimumHeight(56)
        self._date_button.setStyleSheet(control_style)
        self._date_button.clicked.connect(self._open_date_picker)
        grid.addLayout(self._labeled_field("DATE", self._date_button), 0, 0)

        # Level
        self._level_filter = QComboBox(card)
        self._level_filter.setMinimumHeight(56)
        self._level_filter.addItems(LEVEL_FILTERS)
        self._level_filter.setStyleSheet(control_style)
        self._level_filter.currentIndexChanged.connect(self._rebuild_table)
        grid.addLayout(self._labeled_field("LEVEL", self._level_filter), 0, 1)

        # Source
        self._source_filter = QComboBox(card)
        self._source_filter.setMinimumHeight(56)
        self._source_filter.addItem(ALL_SOURCES)
        self._source_filter.addItems(self._sources)
        self._source_filter.setStyleSheet(control_style)
        self._source_filter.currentIndexChanged.connect(self._rebuild_table)
        grid.addLayout(self._labeled_field("SOURCE", self._source_filter), 0, 2)

        # Search - spans the full width, second row
        self._search_box = QLineEdit(card)
        self._search_box.setMinimumHeight(56)
        self._search_box.setPlaceholderText("Filter by message content...")
        self._search_box.setStyleSheet(control_style)
        self._search_box.textChanged.connect(self._rebuild_table)
        grid.addLayout(self._labeled_field("SEARCH MESSAGE", self._search_box), 1, 0, 1, 3)

        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        return card

    def _build_table(self):
        self._table = QTableWidget(0, len(COLUMNS), self)
        self._table.setHorizontalHeaderLabels(COLUMNS)
        self._table.verticalHeader().setVisible(False)
        self._table.verticalHeader().setDefaultSectionSize(ROW_HEIGHT)
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        self._table.setShowGrid(False)
        self._table.setAlternatingRowColors(True)
        self._table.setStyleSheet(
            f"QTableWidget {{"
            f"    background-color: {BG_CARD};"
            f"    alternate-background-color: {BG_CARD_ALT};"
            f"    gridline-color: {BORDER};"
            f"    border: 1px solid {BORDER};"
            f"    font-size: {TABLE_FONT_SIZE}px;"
            f"}}"
            f"QTableWidget::item {{ border: none; padding: 4px 10px; }}"
            f"QTableWidget::item:selected {{ background-color: {ACCENT}; color: #FFFFFF; }}"
            f"QHeaderView::section {{"
            f"    background-color: {BG_HEADER};"
            f"    color: {TEXT_MUTED};"
            f"    border: none;"
            f"    border-bottom: 2px solid {BORDER};"
            f"    padding: 10px;"
            f"    font-size: 18px;"
            f"    font-weight: bold;"
            f"}}"
        )
        header = self._table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        self._table.itemDoubleClicked.connect(self._show_item_detail)
        return self._table

    # ----------------- Date selection -----------------
    def _open_date_picker(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Select Date")
        dlg.resize(560, 480)
        dlg.setStyleSheet(f"QDialog {{ background-color: {BG_CARD}; }}")
        layout = QVBoxLayout(dlg)

        calendar = QCalendarWidget(dlg)
        calendar.setMaximumDate(QDate.currentDate())
        calendar.setSelectedDate(self._selected_date)
        calendar.setGridVisible(True)
        calendar.setStyleSheet(
            f"QCalendarWidget {{ background-color: {BG_CARD}; font-size: 18px; }}"
            f"QCalendarWidget QWidget {{ background-color: {BG_CARD}; alternate-background-color: {BG_CARD}; }}"
            f"QCalendarWidget QToolButton {{"
            f"    font-size: 20px; height: 48px; color: {TEXT_PRIMARY};"
            f"    background-color: {BG_CARD}; border: none; border-radius: 0px;"
            f"}}"
            f"QCalendarWidget QToolButton:hover {{ background-color: {BG_CARD_ALT}; }}"
            f"QCalendarWidget QMenu {{ background-color: {BG_CARD_ALT}; color: {TEXT_PRIMARY}; }}"
            f"QCalendarWidget QAbstractItemView {{"
            f"    font-size: 20px; color: {TEXT_PRIMARY}; background-color: {BG_CARD};"
            f"    selection-background-color: {ACCENT}; selection-color: #FFFFFF;"
            f"}}"
            f"QCalendarWidget QAbstractItemView:disabled {{ color: #A9BACB; }}"
        )
        layout.addWidget(calendar)

        calendar.clicked.connect(lambda qdate: self._pick_date(qdate, dlg))
        dlg.exec_()

    def _pick_date(self, qdate, dlg):
        dlg.accept()
        self._selected_date = qdate
        self._date_button.setText(qdate.toString("dd/MM/yyyy"))
        self._on_date_changed(qdate)

    def _on_date_changed(self, qdate):
        chosen = qdate.toString("yyyyMMdd")  # matches LOG_DATE_FORMAT = "%Y%m%d"
        self._viewing_date = chosen
        if chosen == self._today:
            self._all_items = list(self._notification_center.history())
        else:
            self._all_items = read_log(chosen)
        for item in self._all_items:
            if item.get("source") and item["source"] not in self._sources:
                self._sources.append(item["source"])
        self._sync_source_choices()
        self._rebuild_table()

    def _sync_source_choices(self):
        current = self._source_filter.currentText()
        self._source_filter.blockSignals(True)
        self._source_filter.clear()
        self._source_filter.addItem(ALL_SOURCES)
        self._source_filter.addItems(sorted(set(self._sources)))
        idx = self._source_filter.findText(current)
        self._source_filter.setCurrentIndex(idx if idx >= 0 else 0)
        self._source_filter.blockSignals(False)

    # ----------------- Data -----------------
    def _on_notification_added(self, item):
        if self._viewing_date != self._today:
            return  # viewing a past day's log - live pushes don't belong on screen
        self._all_items.append(item)
        if item["source"] not in self._sources:
            self._sources.append(item["source"])
            self._source_filter.blockSignals(True)
            self._source_filter.addItem(item["source"])
            self._source_filter.blockSignals(False)
        if self._matches_filter(item):
            self._append_row(item, row=0)  # newest first
            self._table.scrollToTop()

    def _matches_filter(self, item):
        level_choice = self._level_filter.currentText()
        if level_choice != LEVEL_FILTERS[0] and item.get("level", "info").lower() != level_choice.lower():
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
        filtered = [item for item in self._all_items if self._matches_filter(item)]
        filtered.sort(key=lambda i: i["timestamp"], reverse=True)  # newest first
        for item in filtered:
            self._append_row(item)
        self._table.scrollToTop()

    def _append_row(self, item, row=None):
        if row is None:
            row = self._table.rowCount()
        self._table.insertRow(row)

        ts = time.strftime("%H:%M:%S", time.localtime(item["timestamp"]))
        level = item.get("level", "info")
        row_bg = ROW_TINTS.get(level)

        ts_item = QTableWidgetItem(ts)
        ts_item.setData(Qt.UserRole, item)
        ts_item.setForeground(QBrush(QColor(TEXT_MUTED)))
        self._table.setItem(row, 0, ts_item)

        level_widget = self._make_level_badge(level)
        self._table.setCellWidget(row, 1, level_widget)

        source_item = QTableWidgetItem(item.get("source", ""))
        source_item.setForeground(QBrush(QColor(TEXT_MUTED)))
        self._table.setItem(row, 2, source_item)

        message_item = QTableWidgetItem(item.get("message", ""))
        message_item.setForeground(QBrush(QColor(TEXT_PRIMARY)))
        self._table.setItem(row, 3, message_item)

        if row_bg:
            for col in (0, 2, 3):
                self._table.item(row, col).setBackground(QBrush(QColor(row_bg)))
            level_widget.setStyleSheet(level_widget.styleSheet() + f"QWidget {{ background-color: {row_bg}; }}")

    def _make_level_badge(self, level):
        color = BADGE_COLORS.get(level, LEVEL_COLORS.get(level, TEXT_MUTED))
        bg = BADGE_BG_COLORS.get(level, BG_CARD_ALT)
        wrapper = QWidget()
        wrapper.setStyleSheet("background-color: transparent;")
        wlayout = QHBoxLayout(wrapper)
        wlayout.setContentsMargins(10, 6, 10, 6)
        badge = QLabel(level.upper())
        badge.setAlignment(Qt.AlignCenter)
        badge.setStyleSheet(
            f"background-color: {bg};"
            f"color: {color};"
            f"border: 1px solid {color};"
            f"padding: 4px 10px;"
            f"font-size: 16px;"
            f"font-weight: bold;"
        )
        wlayout.addWidget(badge)
        wlayout.addStretch(1)
        return wrapper

    # ----------------- Detail popup -----------------
    def _show_item_detail(self, _clicked_item):
        row = self._table.currentRow()
        first_cell = self._table.item(row, 0)
        item = first_cell.data(Qt.UserRole) if first_cell else None
        if not item:
            return

        ts = time.strftime("%d/%m/%Y %H:%M:%S", time.localtime(item["timestamp"]))
        level = item.get("level", "info").upper()
        color = BADGE_COLORS.get(item.get("level", "info"), TEXT_MUTED)

        # No Qt.FramelessWindowHint - keep the native title bar so the
        # popup stays draggable/movable, and non-modal so it doesn't lock
        # the rest of the app while open.
        dlg = QDialog(self, Qt.Window)
        dlg.setWindowTitle("Alarm Detail")
        dlg.resize(760, 460)
        dlg.setStyleSheet(f"QDialog {{ background-color: {BG_PAGE}; }}")
        layout = QVBoxLayout(dlg)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        header = QLabel(f"[{level}]  {ts}  -  {item.get('source', '')}", dlg)
        header.setStyleSheet(f"font-size: {DIALOG_FONT_SIZE}px; font-weight: bold; color: {color};")
        header.setWordWrap(True)
        layout.addWidget(header)

        # Plain (non-editable, non-focusable) QLabel instead of a text-edit
        # widget - the onboard on-screen keyboard auto-pops up for any
        # focusable/editable text widget, which we don't want for a
        # read-only detail view.
        body_label = QLabel(item.get("message", ""), dlg)
        body_label.setWordWrap(True)
        body_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        body_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        body_label.setStyleSheet(f"font-size: {TABLE_FONT_SIZE}px; color: {TEXT_PRIMARY};")

        scroll = QScrollArea(dlg)
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"QScrollArea {{ background-color: {BG_CARD}; border: 1px solid {BORDER}; }}")
        body_label.setStyleSheet(body_label.styleSheet() + f" background-color: {BG_CARD}; padding: 12px;")
        scroll.setWidget(body_label)
        layout.addWidget(scroll, 1)

        close_btn = QPushButton("Close", dlg)
        close_btn.setStyleSheet(
            f"font-size: 22px; padding: 10px 24px; min-height: 48px; min-width: 120px;"
            f"background-color: {BG_CARD_ALT}; color: {TEXT_PRIMARY}; border: 1px solid {BORDER};"
        )
        close_btn.clicked.connect(dlg.close)
        layout.addWidget(close_btn, 0, Qt.AlignRight)

        dlg.setAttribute(Qt.WA_DeleteOnClose)
        dlg.setModal(False)  # movable, non-blocking - user can keep working while it's open
        self._detail_dlg = dlg  # keep a reference so it isn't garbage-collected while shown
        dlg.show()
