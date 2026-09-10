"""Typed snapshot of live system state, passed from RosThread to App over
ui_data_update (a pyqtSignal(object) - see ros_thread.py/app.py). Replaces
the old untyped dict (`data_store`) that used to carry this same
information as loose keys.

No ROS/Qt imports on purpose - keeps this unit-testable the same way
NotificationCenter/JobStore are (see ui/tests/).
"""
from dataclasses import dataclass, field


@dataclass
class DeviceState:
    name: str
    device_state: str
    process_state: str
    mode: str
    detail: str
    last_update: float = 0.0


@dataclass
class SystemStatus:
    devices: dict = field(default_factory=dict)   # name -> DeviceState
    encoder_deg: float = None
    encoder_raw: int = None
    # Storage & Data card (SYSTEM tab) - None until the first background
    # scan completes (see RosThread._update_storage_stats), same
    # "None means not known yet" convention as encoder_deg/encoder_raw.
    storage_used_gb: float = None
    storage_max_gb: float = None
    project_count: int = None
    project_max: int = None


def build_system_status(
    device_status_dicts, encoder_deg=None, encoder_raw=None,
    storage_used_gb=None, storage_max_gb=None, project_count=None, project_max=None,
):
    """device_status_dicts: {name: {device_state, process_state, mode,
    detail, last_update, ...}} - the shape StatusReader.get_status()
    already returns (via ros_msg_to_dict). Pure function, no rospy
    dependency, so it's directly unit-testable."""
    devices = {
        name: DeviceState(
            name=name,
            device_state=d.get("device_state"),
            process_state=d.get("process_state"),
            mode=d.get("mode"),
            detail=d.get("detail"),
            last_update=d.get("last_update", 0.0),
        )
        for name, d in device_status_dicts.items()
    }
    return SystemStatus(
        devices=devices, encoder_deg=encoder_deg, encoder_raw=encoder_raw,
        storage_used_gb=storage_used_gb, storage_max_gb=storage_max_gb,
        project_count=project_count, project_max=project_max,
    )
