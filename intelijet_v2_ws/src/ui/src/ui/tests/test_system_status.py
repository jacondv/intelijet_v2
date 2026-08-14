from ui.system_status import build_system_status, DeviceState, SystemStatus


def test_build_system_status_maps_device_dicts():
    raw = {
        "lidar": {"device_state": "Connected", "process_state": "Standby",
                   "mode": "Continuous", "detail": "", "last_update": 123.0},
    }
    status = build_system_status(raw, encoder_deg=12.5, encoder_raw=42)

    assert isinstance(status, SystemStatus)
    assert status.encoder_deg == 12.5
    assert status.encoder_raw == 42
    assert status.devices["lidar"] == DeviceState(
        name="lidar", device_state="Connected", process_state="Standby",
        mode="Continuous", detail="", last_update=123.0,
    )


def test_build_system_status_defaults_encoder_to_none():
    status = build_system_status({})
    assert status.encoder_deg is None
    assert status.encoder_raw is None
    assert status.devices == {}
