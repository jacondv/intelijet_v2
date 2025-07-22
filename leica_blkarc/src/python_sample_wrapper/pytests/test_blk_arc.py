import time
from datetime import timedelta
from unittest.mock import Mock

import blk_arc_grpc.about_pb2 as about_message
import blk_arc_grpc.clock_pb2 as clock_message
import blk_arc_grpc.device_pb2 as device_message
import blk_arc_grpc.power_pb2 as power_message

from blk_arc_sample_wrapper.blk_arc import BLK_ARC
from blk_arc_sample_wrapper.blk_arc_config import ConnectionType


def test_connect(blk_arc_connection_type: ConnectionType):
    assert blk_arc_connection_type
    blk_arc = BLK_ARC()
    assert blk_arc.connect(blk_arc_connection_type)
    blk_arc.disconnect()


def test_connect_failed_secure_channel(blk_arc_connection_type: ConnectionType, mocker: Mock):
    assert blk_arc_connection_type
    blk_arc = BLK_ARC()
    mock_create_secure_grpc_channel = mocker.patch(
        "blk_arc_sample_wrapper.blk_arc.BLK_ARC_StubHandler._create_secure_grpc_channel", return_value=None)
    assert not blk_arc.connect(blk_arc_connection_type)
    mock_create_secure_grpc_channel.assert_called()


def test_connect_failed_certificate(blk_arc_connection_type: ConnectionType, mocker: Mock):
    assert blk_arc_connection_type
    blk_arc = BLK_ARC()
    mock_get_certificate = mocker.patch(
        "blk_arc_sample_wrapper.blk_arc.BLK_ARC_StubHandler._get_certificate_from_device", return_value=None)
    assert not blk_arc.connect(blk_arc_connection_type)
    mock_get_certificate.assert_called()


def test_is_connected(blk_arc_connection_type: ConnectionType):
    assert blk_arc_connection_type
    blk_arc = BLK_ARC()
    assert not blk_arc.is_connected()
    blk_arc.connect(blk_arc_connection_type)
    assert blk_arc.is_connected()
    blk_arc.disconnect()
    assert not blk_arc.is_connected()


def test_disconnect(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    assert connected_blk_arc.disconnect()
    assert not connected_blk_arc.is_connected()


def test_get_device_info(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    device_info = connected_blk_arc.get_device_info()
    assert device_info
    assert str(type(device_info)) == str(about_message.ManufactureInfoResponse)


def test_get_firmware_version(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    firmware_version = connected_blk_arc.get_firmware_version()
    assert firmware_version
    assert type(firmware_version) == str


def test_get_device_status(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    device_status = connected_blk_arc.get_device_status()
    assert device_status
    assert str(type(device_status)) == str(device_message.DeviceStateResponse)


def test_get_free_disk_space(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    disk_space_info = connected_blk_arc.get_free_disk_space_percentage()
    assert disk_space_info
    assert type(disk_space_info) == float
    assert 0 <= disk_space_info <= 100


def test_power_status(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    power_status = connected_blk_arc.power_status()
    assert power_status
    assert str(type(power_status)) == str(power_message.PowerStatusResponse)


def test_data_only_mode_active(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()
    assert not connected_blk_arc.data_only_mode_active()


def test_data_only_mode_active_no_power_status(connected_blk_arc: BLK_ARC, mocker: Mock):
    assert connected_blk_arc.is_connected()

    mock_power_status = mocker.patch("blk_arc_sample_wrapper.blk_arc.BLK_ARC.power_status", return_value=None)
    assert not connected_blk_arc.data_only_mode_active()
    mock_power_status.assert_called()


def test_get_time_from_clock_sources(connected_blk_arc: BLK_ARC):
    TIME_SLEEP = 5

    assert connected_blk_arc.is_connected()
    clock_status_start = connected_blk_arc.get_time_from_clock_sources()
    head_assembly_clock_start = clock_status_start.head_assembly_clock
    assert clock_status_start
    assert str(type(clock_status_start)) == str(clock_message.TimestampsResponse)

    time.sleep(TIME_SLEEP)

    clock_status_end = connected_blk_arc.get_time_from_clock_sources()
    head_assembly_clock_end = clock_status_end.head_assembly_clock

    assert abs((head_assembly_clock_start.ToTimedelta() + timedelta(seconds=TIME_SLEEP)) -
               head_assembly_clock_end.ToTimedelta()) < timedelta(seconds=1)
