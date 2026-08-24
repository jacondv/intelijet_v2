"""Single place that pushes a SystemStatus snapshot (see ui/system_status.py)
onto UI widgets - replaces three previously-uncoordinated mechanisms:
app.py's hardcoded encoder/device-label pushes, the old DataBinder's
pps-only if/elif chain, and (conceptually) extends the declarative-table
pattern already used by config_mapping in update_data_utils.py, applied
here against live status instead of static config.
"""
from dataclasses import dataclass
from enum import Enum
from typing import Callable

from PyQt5 import QtWidgets

from shared.msg import DeviceStatus
from ui.notification_center import set_device_label


class Status(Enum):
    ACTIVE   = ("Active", "#00FF00")
    INACTIVE = ("Inactive", "#CCCCCC")
    DEFAULT  = ("Default", "#FFFFFF")
    WARNING  = ("Warning", "#FFA500")
    ERROR    = ("Error", "#FF0000")
    READY    = ("Ready", "#0000FF")

    def label(self):
        return self.value[0]

    def color(self):
        return self.value[1]


def set_control_button_stage(widget, state=Status.DEFAULT):
    # Full border (not just the left edge) so a button that's currently
    # running its operation reads as "lit up" from every side, not just
    # a thin accent strip on one edge.
    hex_color = state.color()
    style_dict = {
        "border": f"4px solid {hex_color}",
        "padding": "40px 0px 40px 0px"
    }
    style_str = "; ".join([f"{k}: {v}" for k, v in style_dict.items()])
    if widget:
        widget.setStyleSheet(style_str)


def _set_text(widget, value, fmt="{}"):
    if widget is not None:
        widget.setText(fmt.format(value))


def _set_device_label(widget, device):
    set_device_label(widget, device.device_state if device else None)


def _is_connected(status, device_name):
    device = status.devices.get(device_name)
    return device is not None and (device.device_state or "").upper() == "CONNECTED"


# The devices whose connection actually matters for running a scan -
# "Encoder" and "Scanner" (the lidar) in the operator's own words. PLC/PCAN
# aren't scan-blocking on their own (e.g. PLC only matters for housing
# open/close), so they're deliberately left out of this gate.
SCAN_REQUIRED_DEVICES = ("encoder", "lidar")

# Every device this HMI tracks a connection badge for - used only to decide
# whether the status bar can honestly say "SYSTEM READY" (see
# lblSystemStatus/lblStatusDot below), not for gating any specific button.
ALL_TRACKED_DEVICES = ("plc", "pcan", "lidar", "encoder")


def _scan_ready(status):
    return all(_is_connected(status, name) for name in SCAN_REQUIRED_DEVICES)


def _all_devices_connected(status):
    return all(_is_connected(status, name) for name in ALL_TRACKED_DEVICES)


def _set_system_ready_dot(widget, all_connected):
    color = "#4caf50" if all_connected else "#e74c3c"
    widget.setStyleSheet(f"background-color: {color}; border-radius: 5px; margin-left: 8px;")


def _set_system_ready_label(widget, all_connected):
    text = "SYSTEM READY" if all_connected else "DEVICE DISCONNECTED"
    color = "#4caf50" if all_connected else "#e74c3c"
    widget.setText(text)
    widget.setStyleSheet(f"color: {color}; font-weight: 800; margin-left: 6px; margin-right: 12px;")


@dataclass
class BindingRule:
    getter: Callable       # (SystemStatus) -> value
    setter: Callable       # (widget, value) -> None
    skip_if_none: bool = True  # False for widgets that must always reflect
                                # "no data" (e.g. device labels -> UNKNOWN)


# objectName -> BindingRule. Each entry is independent of the others - add
# a new live value here (getter against SystemStatus + setter against the
# widget) rather than adding another hardcoded push in app.py.
STATUS_BINDINGS = {
    "lblEncoder": BindingRule(
        lambda s: s.encoder_deg,
        lambda w, v: _set_text(w, v, "{:.2f}"),
    ),
    "lblEncoderRawValue": BindingRule(
        lambda s: s.encoder_raw,
        lambda w, v: _set_text(w, v, "{}"),
    ),
    "txtEncodeValueRaw": BindingRule(
        lambda s: s.encoder_raw,
        lambda w, v: _set_text(w, v, "{}"),
    ),
    "lblEncoderStatus": BindingRule(
        lambda s: s.devices.get("encoder"), _set_device_label, skip_if_none=False,
    ),
    "lblLidarStatus": BindingRule(
        lambda s: s.devices.get("lidar"), _set_device_label, skip_if_none=False,
    ),
    "lblPCANStatus": BindingRule(
        lambda s: s.devices.get("pcan"), _set_device_label, skip_if_none=False,
    ),
    "lblPLCStatus": BindingRule(
        lambda s: s.devices.get("plc"), _set_device_label, skip_if_none=False,
    ),
    # SYSTEM READY only means something if it actually reflects every
    # tracked device's connection state - previously this text/dot were
    # hardcoded once at startup and never updated again, so it kept
    # saying "SYSTEM READY" even with every device shown DISCONNECTED.
    "lblStatusDot": BindingRule(_all_devices_connected, _set_system_ready_dot, skip_if_none=False),
    "lblSystemStatus": BindingRule(_all_devices_connected, _set_system_ready_label, skip_if_none=False),
    # Pre-Scan/Post-Scan only make sense with the Encoder and Scanner
    # (lidar) actually connected - previously these stayed clickable
    # regardless of device connection state.
    "btnPreScan": BindingRule(_scan_ready, lambda w, v: w.setEnabled(v), skip_if_none=False),
    "btnPostScan": BindingRule(_scan_ready, lambda w, v: w.setEnabled(v), skip_if_none=False),
}

# pps device_state -> {button_name: Status for the "active" button(s)}.
# Every button not listed for a given state gets Status.INACTIVE. A
# device_state with no entry here is left alone entirely (matches the old
# if/elif's behavior of doing nothing for an unrecognized pps state).
PPS_CONTROLLED_BUTTONS = [
    'btnPreScan', 'btnPostScan', 'btnCompare', 'btnCancel',
    'btnOpenScanner', 'btnCloseScanner',
]

PPS_BUTTON_STAGE_TABLE = {
    DeviceStatus.PRESCAN: {"btnPreScan": Status.ACTIVE},
    DeviceStatus.POSTSCAN: {"btnPostScan": Status.ACTIVE},
    DeviceStatus.OPEN_HOUSING: {"btnOpenScanner": Status.ACTIVE},
    DeviceStatus.CLOSE_HOUSING: {"btnCloseScanner": Status.ACTIVE},
    DeviceStatus.IDLE: {},
    DeviceStatus.PRESCAN_ERROR: {"btnPreScan": Status.ERROR},
    DeviceStatus.POSTSCAN_ERROR: {"btnPostScan": Status.ERROR},
}


class StatusBinder:
    def __init__(self, root_widget: QtWidgets.QWidget):
        self.root_widget = root_widget
        self._widget_cache = {}
        self._build_cache()

    def _build_cache(self):
        """Quét toàn bộ widget con và lưu theo objectName"""
        for child in self.root_widget.findChildren(QtWidgets.QWidget):
            name = child.objectName()
            if name:  # chỉ cache widget có đặt tên
                self._widget_cache[name] = child

    def register(self, name, widget):
        """For widgets outside root_widget's own tree - e.g. the status
        bar's lblStatusDot/lblSystemStatus, which live on the QMainWindow's
        QStatusBar rather than under centralFrame."""
        self._widget_cache[name] = widget

    def apply(self, status):
        """status: ui.system_status.SystemStatus"""
        for obj_name, rule in STATUS_BINDINGS.items():
            widget = self._widget_cache.get(obj_name)
            if widget is None:
                continue
            value = rule.getter(status)
            if value is None and rule.skip_if_none:
                continue
            rule.setter(widget, value)

        pps = status.devices.get("pps")
        pps_state = pps.device_state if pps else None
        active_map = PPS_BUTTON_STAGE_TABLE.get(pps_state)
        if active_map is None:
            return
        for btn_name in PPS_CONTROLLED_BUTTONS:
            widget = self._widget_cache.get(btn_name)
            set_control_button_stage(widget, active_map.get(btn_name, Status.INACTIVE))
