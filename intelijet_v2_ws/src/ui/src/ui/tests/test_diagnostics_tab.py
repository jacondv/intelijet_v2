from PyQt5.QtWidgets import QApplication

from ui.notification_center import NotificationCenter
from ui.diagnostics_tab import DiagnosticsTab

# A QApplication instance is required before any QWidget can be constructed -
# reused across tests in this module, same pattern as test_status_binder.py.
_app = QApplication.instance() or QApplication([])


def _row_texts(tab):
    table = tab._table
    return [
        [table.item(row, col).text() for col in range(table.columnCount())]
        for row in range(table.rowCount())
    ]


def _select(combo, text):
    combo.setCurrentIndex(combo.findText(text))


def test_loads_existing_history_on_creation():
    center = NotificationCenter()
    center.push("lidar", "Lidar disconnected", "error")
    center.push("encoder", "Encoder ready", "info")

    tab = DiagnosticsTab(center)

    assert tab._table.rowCount() == 2


def test_appends_new_notification_via_signal():
    center = NotificationCenter()
    tab = DiagnosticsTab(center)
    assert tab._table.rowCount() == 0

    center.push("pps", "Scan failed: timeout", "error")

    assert tab._table.rowCount() == 1
    row = _row_texts(tab)[0]
    assert row[1] == "ERROR"
    assert row[2] == "pps"
    assert row[3] == "Scan failed: timeout"


def test_level_filter_hides_non_matching_rows():
    center = NotificationCenter()
    center.push("lidar", "Lidar disconnected", "error")
    center.push("encoder", "Encoder ready", "info")
    tab = DiagnosticsTab(center)
    assert tab._table.rowCount() == 2

    _select(tab._level_filter, "Error")
    assert tab._table.rowCount() == 1
    assert _row_texts(tab)[0][2] == "lidar"

    _select(tab._level_filter, "All")
    assert tab._table.rowCount() == 2


def test_source_filter_hides_non_matching_rows():
    center = NotificationCenter()
    center.push("lidar", "Lidar disconnected", "error")
    center.push("encoder", "Encoder ready", "info")
    tab = DiagnosticsTab(center)

    _select(tab._source_filter, "encoder")
    assert tab._table.rowCount() == 1
    assert _row_texts(tab)[0][2] == "encoder"


def test_transient_notification_not_shown():
    center = NotificationCenter()
    tab = DiagnosticsTab(center)

    center.push_transient("Compare 42%", "info")

    assert tab._table.rowCount() == 0
