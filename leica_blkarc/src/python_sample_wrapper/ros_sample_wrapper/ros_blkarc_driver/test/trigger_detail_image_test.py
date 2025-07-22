#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import time
import unittest

from ros_blkarc_msgs.srv import CaptureRequest, GetDeviceStateResponse
from std_srvs.srv import TriggerRequest


class TestTriggerDetailImage(unittest.TestCase):

    # Test if trigger detail image works correctly
    def test_trigger_detail_image(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Trigger detail image when in IDLE state, should be unsuccessful
        trigger_response_1 = service_handles["trigger_detail_image"].call(TriggerRequest())
        self.assertFalse(trigger_response_1.success, "Device unexpectedly triggered a detail image")

        # Ensure scanner is ready to scan
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Start scan
        start_response = service_handles["start_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=start_response, success_expected=True)

        # Wait till scan is running
        test_helpers.wait_till_blk_arc_is_in_state(get_device_state_handle=service_handles["get_device_state"],
                                                   state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING)

        # Trigger detail image when while capture running, should be successful
        trigger_response_2 = service_handles["trigger_detail_image"].call(TriggerRequest())
        self.assertTrue(trigger_response_2.success, "Device failed to trigger a detail image")

        # Wait a bit for trigger image to process
        time.sleep(2.0)

        # Begin Static Pose
        begin_response = service_handles["begin_static_pose"].call(TriggerRequest())
        self.assertTrue(begin_response.success, "Device failed to begin a static pose")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING_STATIC,
                                                         timeout=5.0)

        # Trigger detail image when while capture running static, should be successful
        trigger_response_2 = service_handles["trigger_detail_image"].call(TriggerRequest())
        self.assertTrue(trigger_response_2.success, "Device failed to trigger a detail image")

        # Wait a bit for trigger image to process
        time.sleep(2.0)

        # End static pose
        end_response = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertTrue(end_response.success, "Device failed to end a static pose")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING,
                                                         timeout=5.0)

        # Stop scan, should be successful
        stop_response = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response, success_expected=True)

        # Wait till scanner has stopped capture completely
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Disconnect
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")

        # Trigger detail image when disconnected, should be unsuccessful
        trigger_response_3 = service_handles["trigger_detail_image"].call(TriggerRequest())
        self.assertFalse(trigger_response_3.success, "Device unexpectedly triggered a detail image")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_trigger_detail_image', TestTriggerDetailImage)
