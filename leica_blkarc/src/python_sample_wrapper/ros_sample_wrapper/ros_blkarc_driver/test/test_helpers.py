import os
import rospy
import time
import unittest
from typing import Dict, Tuple

from ros_blkarc_driver import BLKARCROSWrapper

from ros_blkarc_msgs.srv import Capture, CaptureResponse, DownloadScan, DownloadScanResponse, GetDeviceState, GetDeviceStateRequest, GetDeviceStateResponse
from std_srvs.srv import Trigger

DEFAULT_WAIT_FOR_DEVICE_TO_REACH_STATE = 60.0


def setup_blkarc_node() -> BLKARCROSWrapper:

    # Set up ROS node and setup wrapper
    rospy.init_node("blkarc_ros_wrapper")
    blkarc_ros_wrapper = BLKARCROSWrapper()
    rospy.on_shutdown(blkarc_ros_wrapper.close)

    return blkarc_ros_wrapper


def get_connect_service_handle(sensor_name: str) -> rospy.ServiceProxy:
    connect_service_name = f"/{sensor_name}/connect"
    rospy.wait_for_service(service=connect_service_name)
    return rospy.ServiceProxy(name=connect_service_name, service_class=Trigger)


def get_disconnect_service_handle(sensor_name: str) -> rospy.ServiceProxy:
    disconnect_service_name = f"/{sensor_name}/disconnect"
    rospy.wait_for_service(service=disconnect_service_name)
    return rospy.ServiceProxy(name=disconnect_service_name, service_class=Trigger)


def get_reboot_handle(sensor_name: str) -> rospy.ServiceProxy:
    reboot_service_name = f"/{sensor_name}/reboot"
    rospy.wait_for_service(service=reboot_service_name)
    return rospy.ServiceProxy(name=reboot_service_name, service_class=Trigger)


def get_start_capture_service_handle(sensor_name: str) -> rospy.ServiceProxy:
    start_capture_service_name = f"/{sensor_name}/start_capture"
    rospy.wait_for_service(service=start_capture_service_name)
    return rospy.ServiceProxy(name=start_capture_service_name, service_class=Capture)


def get_stop_capture_service_handle(sensor_name: str) -> rospy.ServiceProxy:
    stop_capture_service_name = f"/{sensor_name}/stop_capture"
    rospy.wait_for_service(service=stop_capture_service_name)
    return rospy.ServiceProxy(name=stop_capture_service_name, service_class=Capture)


def get_download_scan_service_handle(sensor_name: str) -> rospy.ServiceProxy:
    download_scan_service_name = f"/{sensor_name}/download_scan"
    rospy.wait_for_service(service=download_scan_service_name)
    return rospy.ServiceProxy(name=download_scan_service_name, service_class=DownloadScan)


def get_get_device_state_handle(sensor_name: str) -> rospy.ServiceProxy:
    get_device_status_service_name = f"/{sensor_name}/get_device_state"
    rospy.wait_for_service(service=get_device_status_service_name)
    return rospy.ServiceProxy(name=get_device_status_service_name, service_class=GetDeviceState)


def get_begin_static_pose(sensor_name: str) -> rospy.ServiceProxy:
    begin_static_pose_service_name = f"/{sensor_name}/begin_static_pose"
    rospy.wait_for_service(service=begin_static_pose_service_name)
    return rospy.ServiceProxy(name=begin_static_pose_service_name, service_class=Trigger)


def get_end_static_pose(sensor_name: str) -> rospy.ServiceProxy:
    end_static_pose_service_name = f"/{sensor_name}/end_static_pose"
    rospy.wait_for_service(service=end_static_pose_service_name)
    return rospy.ServiceProxy(name=end_static_pose_service_name, service_class=Trigger)


def get_trigger_detail_image(sensor_name: str) -> rospy.ServiceProxy:
    trigger_detail_image_service_name = f"/{sensor_name}/trigger_detail_image"
    rospy.wait_for_service(service=trigger_detail_image_service_name)
    return rospy.ServiceProxy(name=trigger_detail_image_service_name, service_class=Trigger)


def get_all_service_handles(sensor_name: str) -> Dict[str, rospy.ServiceProxy]:
    service_handles = {}
    service_handles["connect"] = get_connect_service_handle(sensor_name=sensor_name)
    service_handles["disconnect"] = get_disconnect_service_handle(sensor_name=sensor_name)
    service_handles["start_capture"] = get_start_capture_service_handle(sensor_name=sensor_name)
    service_handles["stop_capture"] = get_stop_capture_service_handle(sensor_name=sensor_name)
    service_handles["download_scan"] = get_download_scan_service_handle(sensor_name=sensor_name)
    service_handles["get_device_state"] = get_get_device_state_handle(sensor_name=sensor_name)
    service_handles["reboot"] = get_reboot_handle(sensor_name=sensor_name)
    service_handles["begin_static_pose"] = get_begin_static_pose(sensor_name=sensor_name)
    service_handles["end_static_pose"] = get_end_static_pose(sensor_name=sensor_name)
    service_handles["trigger_detail_image"] = get_trigger_detail_image(sensor_name=sensor_name)
    return service_handles


def get_all_service_handles_and_ensure_device_is_idle(sensor_name: str) -> Tuple[bool, Dict[str, rospy.ServiceProxy]]:
    handles = get_all_service_handles(sensor_name=sensor_name)
    device_is_idle = wait_till_blk_arc_is_idle(get_device_state_handle=handles["get_device_state"])
    return device_is_idle, handles


def assert_capture_service(test_case: unittest.TestCase, response: CaptureResponse, success_expected: bool) -> None:
    test_case.assertTrue(response.success if success_expected else not response.success,
                         f"Expected {success_expected}, got {not success_expected} instead.")
    test_case.assertTrue(
        response.scan_id > -1 if success_expected else not response.scan_id > -1,
        f"Scan_id field expected to be {'non-negative' if success_expected else 'negative'}, instead got {response.scan_id}."
    )


def assert_device_state(test_case: unittest.TestCase, get_device_state_handle: rospy.ServiceProxy,
                        expected_states: Tuple[int]) -> None:
    actual_state = get_device_state_handle.call(GetDeviceStateRequest()).state
    test_case.assertTrue(
        actual_state in expected_states,
        f"Expected states to be {[BLKARCROSWrapper.ros_device_state_to_string(ros_device_state=expected_state) for expected_state in expected_states]}. Acutal state: {BLKARCROSWrapper.ros_device_state_to_string(ros_device_state=actual_state)}"
    )


def wait_till_blk_arc_is_in_state(get_device_state_handle: rospy.ServiceProxy,
                                  state: GetDeviceStateResponse,
                                  timeout: float = DEFAULT_WAIT_FOR_DEVICE_TO_REACH_STATE) -> bool:
    timeout_time = time.time() + timeout
    timed_out = False
    state_reached = False
    while not timed_out and not state_reached:
        device_state_response = get_device_state_handle.call(GetDeviceStateRequest())
        state_reached = device_state_response.state == state
        timed_out = time.time() > timeout_time
        time.sleep(1.0)
    if timed_out:
        rospy.logwarn(f"Device still not in state {state} after waiting {timeout} seconds")
    return state_reached


def wait_till_blk_arc_is_idle(get_device_state_handle: rospy.ServiceProxy,
                              timeout: float = DEFAULT_WAIT_FOR_DEVICE_TO_REACH_STATE) -> bool:
    return wait_till_blk_arc_is_in_state(get_device_state_handle=get_device_state_handle,
                                         state=GetDeviceStateResponse.STATE_IDLE,
                                         timeout=timeout)


def wait_and_assert_blk_arc_is_in_state(test_case: unittest.TestCase,
                                        get_device_state_handle: rospy.ServiceProxy,
                                        state: GetDeviceStateResponse,
                                        timeout: float = DEFAULT_WAIT_FOR_DEVICE_TO_REACH_STATE):
    test_case.assertTrue(
        wait_till_blk_arc_is_in_state(get_device_state_handle=get_device_state_handle, state=state, timeout=timeout),
        f"Failed to reach state {BLKARCROSWrapper.ros_device_state_to_string(ros_device_state=state)} after {timeout} seconds"
    )


def assert_download_scan_service(test_case: unittest.TestCase, response: DownloadScanResponse,
                                 success_expected: bool) -> None:
    # Check if the fields in the response message of the download_scan_service are correct

    # success in response message is the same as the one expected from the test
    test_case.assertTrue(response.success if success_expected else not response.success,
                         f"Expected {success_expected}, got {not success_expected} instead.")

    # Scan ID should be positive if scan succeeded, -1
    test_case.assertTrue(
        response.scan_id > -1 if success_expected else not response.scan_id > -1,
        f"Scan_id field expected to be {'non-negative' if success_expected else 'negative'}, instead got {response.scan_id}."
    )

    # Filename should not be empty if succeeded, empty string otherwise
    test_case.assertTrue(response.filename if success_expected else not response.filename,
                         f"Scan_id field expected to be {'non-empty' if success_expected else 'empty'}.")

    # Size of file should be non-zero if succeeded, zero otherwise
    test_case.assertTrue(response.size_bytes > 0 if success_expected else not response.size_bytes > 0,
                         f"byte_size field expected to be {'non-zero' if success_expected else 'zero'}.")


def delete_file(test_case: unittest.TestCase, file_location: str) -> None:

    # Check if a file exists in the specified location
    file_exists = os.path.exists(file_location)
    test_case.assertTrue(file_exists, f"file {file_location} does not exist.")

    if file_exists:
        # Remove the file
        os.remove(file_location)

        # Affirm that file is deleted
        test_case.assertFalse(
            os.path.exists(file_location),
            f"file {file_location} should not exist but it does, deletion not performed successfully.")
