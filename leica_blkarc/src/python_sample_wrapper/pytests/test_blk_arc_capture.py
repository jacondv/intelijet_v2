import time
from datetime import datetime, timedelta
from unittest.mock import Mock

from blk_arc_sample_wrapper.blk_arc import BLK_ARC


def test_start_stop_capture_is_scanning(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()

    start_scan_info = connected_blk_arc.start_capture()
    assert start_scan_info
    assert type(start_scan_info.scan_id) == str
    assert int(start_scan_info.scan_id) >= 0

    while not connected_blk_arc.is_scanning():
        time.sleep(1)
    start_time = datetime.now()    # start counting time, when device announces it actually started

    # Scan for about 3s, then check actual duration.
    time.sleep(3)

    stop_response = connected_blk_arc.stop_capture()
    actual_duration = datetime.now() - start_time
    assert stop_response
    assert abs(stop_response.duration.ToTimedelta() - actual_duration) < timedelta(seconds=1)
    assert int(stop_response.scan_id) == int(start_scan_info.scan_id)
    assert not connected_blk_arc.is_scanning()


def test_scan_data_stream(connected_blk_arc: BLK_ARC):
    assert connected_blk_arc.is_connected()

    start_scan_info = connected_blk_arc.start_capture()
    assert start_scan_info

    while not connected_blk_arc.is_scanning():
        time.sleep(1)

    capture_stream = connected_blk_arc.start_scan_data_stream()
    assert capture_stream

    baser_count = 0
    trajectory_count = 0
    previous_baser_ts = None
    previous_trajectory_ts = None

    start_time = time.time()
    for message in capture_stream:
        assert message.HasField("trajectory") or message.HasField("baser") or message.HasField(
            "slam_metric") or message.HasField("scan_event")
        if message.HasField("baser"):
            baser_count += len(message.baser.points)
            for point in message.baser.points:
                if previous_baser_ts:
                    assert point.timestamp.ToTimedelta() > previous_baser_ts
                previous_baser_ts = point.timestamp.ToTimedelta()
        elif message.HasField("trajectory"):
            trajectory_count += len(message.trajectory.points)
            for point in message.trajectory.points:
                if previous_trajectory_ts:
                    assert point.timestamp.ToTimedelta() > previous_trajectory_ts
                previous_trajectory_ts = point.timestamp.ToTimedelta()
        if (time.time() - start_time) > 10:
            capture_stream.cancel()
            break

    assert baser_count > 0
    assert trajectory_count > 0


def test_begin_static_pose(blkarc_scanning: BLK_ARC):
    assert blkarc_scanning.begin_static_pose()


def test_end_static_pose(blkarc_scanning: BLK_ARC):
    blkarc_scanning.begin_static_pose()
    while not blkarc_scanning.is_in_static_pose():
        time.sleep(1)
    assert blkarc_scanning.end_static_pose()


def test_begin_static_pose_not_scanning(connected_blk_arc: BLK_ARC):
    assert not connected_blk_arc.begin_static_pose()


def test_end_static_pose_not_scanning(connected_blk_arc: BLK_ARC):
    assert not connected_blk_arc.end_static_pose()


def test_end_static_pose_while_not_in_static_pose(blkarc_scanning: BLK_ARC):
    assert not blkarc_scanning.end_static_pose()


def test_begin_static_pose_while_already_in_static_pose(blkarc_scanning: BLK_ARC):
    assert blkarc_scanning.begin_static_pose()
    while not blkarc_scanning.is_in_static_pose():
        time.sleep(1)
    assert not blkarc_scanning.begin_static_pose()


def test_is_scanning_no_device_state(connected_blk_arc: BLK_ARC, mocker: Mock):
    assert connected_blk_arc.is_connected()

    mock_device_state = mocker.patch("blk_arc_sample_wrapper.blk_arc.BLK_ARC.get_device_status", return_value=None)
    assert not connected_blk_arc.is_scanning()
    mock_device_state.assert_called()


def test_is_in_static_pose_no_device_state(connected_blk_arc: BLK_ARC, mocker: Mock):
    assert connected_blk_arc.is_connected()

    mock_device_state = mocker.patch("blk_arc_sample_wrapper.blk_arc.BLK_ARC.get_device_status", return_value=None)
    assert not connected_blk_arc.is_in_static_pose()
    mock_device_state.assert_called()
