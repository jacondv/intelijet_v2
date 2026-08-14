from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton

from shared.msg import DeviceStatus
from ui.system_status import SystemStatus, DeviceState
from ui.status_binder import StatusBinder, Status

# A QApplication instance is required before any QWidget can be
# constructed - reused across tests in this module (module-scoped is fine,
# these tests don't need isolation between widget trees since each test
# builds its own fresh container/binder).
_app = QApplication.instance() or QApplication([])


def _make_container():
    container = QWidget()
    labels = {}
    for name in ["lblEncoder", "lblEncoderRawValue", "txtEncodeValueRaw",
                 "lblEncoderStatus", "lblLidarStatus", "lblPCANStatus", "lblPLCStatus"]:
        w = QLabel(container)
        w.setObjectName(name)
        labels[name] = w
    buttons = {}
    for name in ["btnPreScan", "btnPostScan", "btnCompare", "btnCancel",
                 "btnOpenScanner", "btnCloseScanner"]:
        b = QPushButton(container)
        b.setObjectName(name)
        buttons[name] = b
    return container, labels, buttons


def _device(state):
    return DeviceState(name="x", device_state=state, process_state="", mode="", detail="")


def test_encoder_text_only_updates_when_present():
    container, labels, _ = _make_container()
    binder = StatusBinder(container)

    binder.apply(SystemStatus(devices={}, encoder_deg=None, encoder_raw=None))
    assert labels["lblEncoder"].text() == ""

    binder.apply(SystemStatus(devices={}, encoder_deg=12.3, encoder_raw=99))
    assert labels["lblEncoder"].text() == "12.30"
    assert labels["lblEncoderRawValue"].text() == "99"
    assert labels["txtEncodeValueRaw"].text() == "99"


def test_device_labels_show_unknown_when_missing():
    container, labels, _ = _make_container()
    binder = StatusBinder(container)

    binder.apply(SystemStatus(devices={}))
    assert labels["lblLidarStatus"].text() == "UNKNOWN"


def test_device_labels_reflect_connected_state():
    container, labels, _ = _make_container()
    binder = StatusBinder(container)

    status = SystemStatus(devices={
        "lidar": _device(DeviceStatus.CONNECTED),
        "encoder": _device(DeviceStatus.DISCONNECTED),
    })
    binder.apply(status)
    assert labels["lblLidarStatus"].text() == "CONNECTED"
    assert labels["lblEncoderStatus"].text() == "DISCONNECTED"


def test_pps_prescan_state_activates_only_prescan_button():
    container, _, buttons = _make_container()
    binder = StatusBinder(container)

    status = SystemStatus(devices={"pps": _device(DeviceStatus.PRESCAN)})
    binder.apply(status)

    active_color = Status.ACTIVE.color()
    inactive_color = Status.INACTIVE.color()
    assert active_color in buttons["btnPreScan"].styleSheet()
    for name in ["btnPostScan", "btnCompare", "btnCancel", "btnOpenScanner", "btnCloseScanner"]:
        assert inactive_color in buttons[name].styleSheet()


def test_pps_unrecognized_state_leaves_buttons_untouched():
    container, _, buttons = _make_container()
    binder = StatusBinder(container)

    # First set a known state so buttons have a real stylesheet...
    binder.apply(SystemStatus(devices={"pps": _device(DeviceStatus.PRESCAN)}))
    before = buttons["btnPreScan"].styleSheet()

    # ...then an unrecognized one must not touch anything (matches the old
    # if/elif's behavior of doing nothing for an unmatched pps state).
    binder.apply(SystemStatus(devices={"pps": _device("SomeUnknownState")}))
    assert buttons["btnPreScan"].styleSheet() == before
