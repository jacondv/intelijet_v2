#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import time
import unittest

from ros_blkarc_msgs.srv import CaptureRequest
from std_srvs.srv import TriggerRequest


class TestStopCapture(unittest.TestCase):

    # Test if stop capture service call behaves correctly
    def test_stop_capture(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Call stop capture when only connected, should be unsuccessful
        stop_response_1 = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response_1, success_expected=False)

        # Ensure scanner is ready to scan
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Start scan
        start_response = service_handles["start_capture"].call(CaptureRequest())
        start_scan_id = start_response.scan_id
        test_helpers.assert_capture_service(test_case=self, response=start_response, success_expected=True)

        # Wait for a bit
        time.sleep(1.0)

        # Disconnect
        disconnect_response_1 = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response_1.success, "Sensor failed to disconnect")

        # Stop scan, should be unsuccessful
        stop_response_2 = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response_2, success_expected=False)

        # Connect
        connect_response = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response.success, "Sensor failed to connect")

        # Stop scan, should be successful
        stop_response_3 = service_handles["stop_capture"].call(CaptureRequest())
        stop_scan_id = stop_response_3.scan_id
        test_helpers.assert_capture_service(test_case=self, response=stop_response_3, success_expected=True)

        # Make sure that scan_id matches the one provided by the start scan response
        self.assertEqual(start_scan_id, stop_scan_id,
                         f"Start scan_id {start_scan_id} is different from stop scan_id {stop_scan_id}")

        # Wait till scanner has stopped capture completely
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Call stop scan again, should be unsuccessful
        stop_response_4 = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response_4, success_expected=False)

        # Disconnect
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")

        # Stop scan, should be unsuccessful
        stop_response_5 = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response_5, success_expected=False)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_stop_capture', TestStopCapture)
