#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import time
import unittest

from ros_blkarc_msgs.srv import CaptureRequest, GetDeviceStateResponse
from std_srvs.srv import TriggerRequest


class TestReboot(unittest.TestCase):

    # Test if reboot service call behaves correctly
    def test_reboot(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Reboot while IDLE
        reboot_response_1 = service_handles["reboot"].call(TriggerRequest())
        self.assertTrue(reboot_response_1.success, "Sensor failed to reboot")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_UNKNOWN,
                                                         timeout=5.0)

        # Wait for reboot to process
        time.sleep(60.0)

        # Attempt to connect again and ensure in IDLE state
        connect_response = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response.success, "Sensor failed to connect")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_IDLE,
                                                         timeout=3.0)

        # Start capture
        start_response = service_handles["start_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=start_response, success_expected=True)
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING,
                                                         timeout=60.0)

        # Reboot while scanning
        test_helpers.assert_device_state(test_case=self,
                                         get_device_state_handle=service_handles["get_device_state"],
                                         expected_states=(GetDeviceStateResponse.STATE_CAPTURE_RUNNING,))
        reboot_response_2 = service_handles["reboot"].call(TriggerRequest())
        self.assertTrue(reboot_response_2.success, "Sensor failed to reboot")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_UNKNOWN,
                                                         timeout=10.0)

        # Wait for reboot to process
        time.sleep(60.0)

        # Attempt to connect again and ensure in CAPTURE_STARTING/CAPTURE_RUNNING state (device is programmed to continue scanning after reboot)
        connect_response = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response.success, "Sensor failed to disconnect")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING,
                                                         timeout=60.0)

        # Stop capture, ensure in IDLE state
        stop_response = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response, success_expected=True)
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_IDLE,
                                                         timeout=30.0)

        # Disconnect device, reboot will fail
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_UNKNOWN,
                                                         timeout=5.0)
        reboot_response_3 = service_handles["reboot"].call(TriggerRequest())
        self.assertFalse(reboot_response_3.success, "Sensor failed to reboot")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_reboot', TestReboot)
