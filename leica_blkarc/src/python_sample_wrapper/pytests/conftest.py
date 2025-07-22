import subprocess
import time
from typing import Optional, Tuple

import pytest
from blk_arc_grpc import device_pb2 as device_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import BLK_ARC_IP, ConnectionType


def pytest_addoption(parser):
    parser.addoption("--custom-ip", action="store", default="", help="Custom IP-address of the scanner.")


@pytest.fixture
def custom_ip(request):
    return request.config.getoption("--custom-ip")


@pytest.fixture
def blk_arc_connection_type(custom_ip: str) -> Optional[ConnectionType]:

    def ping_device(ip: str) -> bool:
        command = ["ping", "-c", "1", ip, "-W", "5"]
        try:
            if subprocess.call(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=2.0) == 0:
                return True
        except subprocess.TimeoutExpired:
            print(f"No BLK ARC connected on '{ip}'.")
        return False

    if custom_ip:
        BLK_ARC_IP[ConnectionType.WIRED] = custom_ip
        if ping_device(ip=custom_ip):
            print(f"Using custom-IP '{custom_ip}' in WIRED-mode.")
            return ConnectionType.WIRED

    for type, ip in BLK_ARC_IP.items():
        if ping_device(ip=ip):
            print(f"Found a BLK ARC connected in {type.name}-Mode.")
            return type
    return None


@pytest.fixture
def connected_blk_arc(blk_arc_connection_type: ConnectionType) -> BLK_ARC:
    assert blk_arc_connection_type
    blk_arc = BLK_ARC()
    assert blk_arc.connect(blk_arc_connection_type)
    assert blk_arc.wait_till_idle(timeout=60.0)

    yield blk_arc

    blk_arc.stop_capture()
    blk_arc.disconnect()


def stop_scan_if_scanning(connected_blk_arc: BLK_ARC) -> None:
    device_status = connected_blk_arc.get_device_status()
    assert device_status is not None
    if device_status.state in (device_message.DeviceStateResponse.State.CAPTURE_STARTING,
                               device_message.DeviceStateResponse.State.CAPTURE_RUNNING,
                               device_message.DeviceStateResponse.State.CAPTURE_RUNNING_STATIC):
        assert connected_blk_arc.stop_capture() is not None


@pytest.fixture
def blkarc_scanning(connected_blk_arc: BLK_ARC) -> BLK_ARC:
    stop_scan_if_scanning(connected_blk_arc=connected_blk_arc)
    assert connected_blk_arc.wait_till_idle(timeout=60.0)
    start_scan_info = connected_blk_arc.start_capture()
    assert start_scan_info
    assert type(start_scan_info.scan_id) == str
    assert int(start_scan_info.scan_id) >= 0

    while not connected_blk_arc.is_scanning():
        time.sleep(1)

    yield connected_blk_arc

    connected_blk_arc.stop_capture()
    connected_blk_arc.wait_till_idle(timeout=15)
    connected_blk_arc.disconnect()


@pytest.fixture
def blk_arc_with_finished_scan(connected_blk_arc: BLK_ARC) -> Tuple[BLK_ARC, int]:
    stop_scan_if_scanning(connected_blk_arc=connected_blk_arc)
    assert connected_blk_arc.wait_till_idle(timeout=60.0)
    start_scan_info = connected_blk_arc.start_capture()
    assert start_scan_info is not None
    time.sleep(20)
    connected_blk_arc.stop_capture()
    assert connected_blk_arc.wait_till_idle(timeout=60.0)

    return (connected_blk_arc, start_scan_info.scan_id)
