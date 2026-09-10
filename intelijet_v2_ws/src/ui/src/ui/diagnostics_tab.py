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

from shared.error_codes import lookup as lookup_error_code
from ui.notification_center import LEVEL_COLORS, LOG_DATE_FORMAT, read_log
from ui.widgets.picker_item_delegate import PickerItemDelegate
from ui.style_tokens import (
    LIGHT_BG, CARD_BG, ROW_BG, ACCENT_YELLOW,
    BORDER as BORDER_TOKEN, TEXT, TEXT_MUTED as TEXT_MUTED_TOKEN,
)

LEVEL_FILTERS = ["All Levels", "Info", "Warning", "Error"]
ALL_SOURCES = "All Sources"
COLUMNS = ["Timestamp", "Level", "Source", "Message"]

ROW_HEIGHT = 64
TABLE_FONT_SIZE = 24
DIALOG_FONT_SIZE = 32

# ---- Shared light-card palette (same tokens as style_tokens.py, used by
# JOB/SYSTEM/REPORT) - kept as local names here since severity colors
# below need their own dedicated constants anyway. ----
BG_PAGE = LIGHT_BG
BG_CARD = CARD_BG
BG_CARD_ALT = ROW_BG
BG_HEADER = CARD_BG
BORDER = BORDER_TOKEN
TEXT_PRIMARY = TEXT
TEXT_MUTED = TEXT_MUTED_TOKEN
ACCENT = ACCENT_YELLOW
CARD_RADIUS = "10px"
FIELD_RADIUS = "8px"
LEVEL_ERROR = "#E74C3C"
LEVEL_WARN = "#F39C12"
LEVEL_INFO = "#22A559"
ROW_ERROR_BG = "#FDECEA"
ROW_WARN_BG = "#FFF6E5"
BADGE_ERROR_BG = "#FBDEDB"
BADGE_WARN_BG = "#FDECD1"
BADGE_INFO_BG = "#DAF3E4"

BADGE_COLORS = {"error": LEVEL_ERROR, "warning": LEVEL_WARN, "info": LEVEL_INFO}
# Severity order for sorting by the Level column - higher is more severe,
# so clicking Level ascending shows Info->Warning->Error.
LEVEL_RANK = {"info": 0, "warning": 1, "error": 2}
BADGE_BG_COLORS = {"error": BADGE_ERROR_BG, "warning": BADGE_WARN_BG, "info": BADGE_INFO_BG}
ROW_TINTS = {"error": ROW_ERROR_BG, "warning": ROW_WARN_BG}


class DiagnosticsTab(QWidget):
    def __init__(self, notification_center, parent=None):
        super().__init__(parent)
        # A plain QWidget doesn't paint its stylesheet's background-color
        # by default (only styled widgets like QFrame do) - without this
        # attribute, this page's background falls through to whatever the
        # parent QStackedWidget/QMainWindow paints (the app's dark theme),
        # not the light gray set in _build_ui().
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._notification_center = notification_center
        self._today = date_cls.today().strftime(LOG_DATE_FORMAT)
        self._viewing_date = self._today  # date currently shown; only "today" gets live updates
        self._all_items = list(notification_center.history())
        self._sources = sorted({item["source"] for item in self._all_items})
        # Timestamp column, newest first - matches the previous hardcoded
        # default before header-click sorting existed.
        self._sort_column = 0
        self._sort_ascending = False

        self._build_ui()
        self._rebuild_table()

        notification_center.notification_added.connect(self._on_notification_added)

    # ----------------- UI -----------------
    def _build_ui(self):
        self.setStyleSheet(f"QWidget {{ background-color: {BG_PAGE}; color: {TEXT_PRIMARY}; }}")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(40, 40, 40, 40)
        layout.setSpacing(32)

        layout.addWidget(self._build_header())
        layout.addWidget(self._build_filter_toolbar())
        layout.addWidget(self._build_table(), 1)

    def _build_header(self):
        header = QFrame(self)
        header.setStyleSheet(
            f"QFrame {{ background-color: {BG_HEADER}; border: 1px solid {BORDER};"
            f" border-left: 6px solid {ACCENT}; border-radius: {CARD_RADIUS}; }}"
        )
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(36, 28, 36, 28)
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
            f"    border-radius: {FIELD_RADIUS};"
            f"    padding: 8px 12px;"
            f"    font-size: 22px;"
            f"}}"
            f"QPushButton:hover, QComboBox:hover {{ border: 1px solid {ACCENT}; }}"
            f"QComboBox::drop-down {{ border: none; width: 36px; }}"
            f"QComboBox QAbstractItemView {{"
            f"    background-color: {BG_CARD_ALT}; color: {TEXT_PRIMARY};"
            f"    selection-background-color: {ACCENT}; selection-color: #153E42; font-size: 20px;"
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
        card.setStyleSheet(
            f"QFrame {{ background-color: {BG_CARD}; border: 1px solid {BORDER};"
            f" border-radius: {CARD_RADIUS}; }}"
        )
        grid = QGridLayout(card)
        grid.setContentsMargins(36, 36, 36, 36)
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
        self._level_filter.setCurrentIndex(LEVEL_FILTERS.index("Error"))
        self._level_filter.setStyleSheet(control_style)
        self._level_filter.setItemDelegate(PickerItemDelegate(self._level_filter))
        self._level_filter.currentIndexChanged.connect(self._rebuild_table)
        grid.addLayout(self._labeled_field("LEVEL", self._level_filter), 0, 1)

        # Source
        self._source_filter = QComboBox(card)
        self._source_filter.setMinimumHeight(56)
        self._source_filter.addItem(ALL_SOURCES)
        self._source_filter.addItems(self._sources)
        self._source_filter.setStyleSheet(control_style)
        self._source_filter.setItemDelegate(PickerItemDelegate(self._source_filter))
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
            f"    border-radius: {CARD_RADIUS};"
            f"    font-size: {TABLE_FONT_SIZE}px;"
            f"}}"
            f"QTableWidget::item {{ border: none; padding: 4px 10px; }}"
            f"QTableWidget::item:selected {{ background-color: {ACCENT}; color: #153E42; }}"
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
        # Click-to-sort headers instead of a separate sort dropdown -
        # standard grid convention (Excel, Windows Event Viewer): click a
        # column to sort by it, click again to flip direction. Manual
        # (not QTableWidget's built-in setSortingEnabled) because the
        # Level column is a cell widget (badge), which Qt's own sort
        # doesn't reorder correctly - see _rebuild_table/_sort_key.
        header.setSectionsClickable(True)
        header.setSortIndicatorShown(True)
        header.sectionClicked.connect(self._on_header_clicked)
        header.setSortIndicator(self._sort_column, Qt.DescendingOrder)
        self._table.itemDoubleClicked.connect(self._show_item_detail)
        return self._table

    def _on_header_clicked(self, column):
        if column == self._sort_column:
            self._sort_ascending = not self._sort_ascending
        else:
            self._sort_column = column
            self._sort_ascending = True
        self._table.horizontalHeader().setSortIndicator(
            self._sort_column, Qt.AscendingOrder if self._sort_ascending else Qt.DescendingOrder
        )
        self._rebuild_table()

    # ----------------- Date selection -----------------
    def _open_date_picker(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Select Date")
        dlg.resize(840, 720)
        dlg.setStyleSheet(f"QDialog {{ background-color: {BG_CARD}; }}")
        layout = QVBoxLayout(dlg)

        calendar = QCalendarWidget(dlg)
        calendar.setMaximumDate(QDate.currentDate())
        calendar.setSelectedDate(self._selected_date)
        calendar.setGridVisible(True)
        calendar.setStyleSheet(
            f"QCalendarWidget {{ background-color: {BG_CARD}; font-size: 27px; }}"
            f"QCalendarWidget QWidget {{ background-color: {BG_CARD}; alternate-background-color: {BG_CARD}; }}"
            f"QCalendarWidget QToolButton {{"
            f"    font-size: 30px; height: 72px; color: {TEXT_PRIMARY};"
            f"    background-color: {BG_CARD}; border: none; border-radius: 0px;"
            f"}}"
            f"QCalendarWidget QToolButton:hover {{ background-color: {BG_CARD_ALT}; }}"
            f"QCalendarWidget QMenu {{ background-color: {BG_CARD_ALT}; color: {TEXT_PRIMARY}; }}"
            f"QCalendarWidget QAbstractItemView {{"
            f"    font-size: 30px; color: {TEXT_PRIMARY}; background-color: {BG_CARD};"
            f"    selection-background-color: {ACCENT}; selection-color: #153E42;"
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
            # A live push only has an unambiguous "correct" slot to insert
            # into when sorted by Timestamp (new items are the newest by
            # definition). Sorted by any other column, where a new item
            # lands among existing values isn't knowable without a real
            # sort, so just rebuild - live pushes are infrequent enough
            # for this to be cheap.
            if self._sort_column != 0:
                self._rebuild_table()
            elif self._sort_ascending:
                self._append_row(item)  # oldest-first - new entry belongs at the bottom
                self._table.scrollToBottom()
            else:
                self._append_row(item, row=0)  # newest-first (default)
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

    def _sort_key(self, item):
        if self._sort_column == 1:
            return LEVEL_RANK.get(item.get("level", "info").lower(), 0)
        if self._sort_column == 2:
            return item.get("source", "").lower()
        if self._sort_column == 3:
            return item.get("message", "").lower()
        return item["timestamp"]  # column 0, and the fallback default

    def _rebuild_table(self, *_args):
        self._table.setRowCount(0)
        filtered = [item for item in self._all_items if self._matches_filter(item)]
        filtered.sort(key=self._sort_key, reverse=not self._sort_ascending)
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

        # Only one Alarm Detail window at a time - close whatever's open
        # before building the next one, instead of letting popups pile up
        # (there's no taskbar/alt-tab on this touchscreen kiosk to manage
        # a stack of them, so a second click needs to replace, not add).
        existing = getattr(self, "_detail_dlg", None)
        if existing is not None:
            try:
                existing.close()
            except RuntimeError:
                pass  # already destroyed (WA_DeleteOnClose beat us to it)

        ts = time.strftime("%d/%m/%Y %H:%M:%S", time.localtime(item["timestamp"]))
        level = item.get("level", "info").upper()
        color = BADGE_COLORS.get(item.get("level", "info"), TEXT_MUTED)

        # No Qt.FramelessWindowHint - keep the native title bar so the
        # popup stays draggable/movable, and non-modal so it doesn't lock
        # the rest of the app while open.
        dlg = QDialog(self, Qt.Window)
        dlg.setWindowTitle("Alarm Detail")
        dlg.resize(1520, 920)
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

        # Troubleshooting guide - only shown for entries carrying a known
        # error code (see shared/error_codes.py). Older history entries
        # (logged before this feature, or a level with no assigned code)
        # simply don't have one - item.get(), not item["code"], on purpose.
        entry = lookup_error_code(item.get("code")) if item.get("code") else None
        if entry is not None:
            dlg.resize(1520, 1240)

            guide_header = QLabel("How to check / fix", dlg)
            guide_header.setStyleSheet(
                f"font-size: {DIALOG_FONT_SIZE}px; font-weight: bold; color: {TEXT_PRIMARY};"
            )
            layout.addWidget(guide_header)

            guide_label = QLabel(entry["guide"], dlg)
            guide_label.setWordWrap(True)
            guide_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
            guide_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            guide_label.setStyleSheet(
                f"font-size: {TABLE_FONT_SIZE}px; color: {TEXT_PRIMARY};"
                f"background-color: {BG_CARD}; padding: 12px;"
            )

            guide_scroll = QScrollArea(dlg)
            guide_scroll.setWidgetResizable(True)
            guide_scroll.setStyleSheet(f"QScrollArea {{ background-color: {BG_CARD}; border: 1px solid {BORDER}; }}")
            guide_scroll.setWidget(guide_label)
            layout.addWidget(guide_scroll, 1)

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
        # show() alone doesn't guarantee top stacking order on every WM -
        # raise_()+activateWindow() is the standard Qt one-two for "make
        # this the frontmost, focused window right now".
        dlg.raise_()
        dlg.activateWindow()
