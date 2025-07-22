#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import unittest

from ros_blkarc_msgs.srv import CaptureRequest, GetDeviceStateResponse
from std_srvs.srv import TriggerRequest


class TestGetDeviceState(unittest.TestCase):

    # Test if get device state service call behaves correctly
    def test_get_device_state(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # IDLE State, as assured by get_all_service_handles_and_ensure_device_is_idle
        test_helpers.assert_device_state(test_case=self,
                                         get_device_state_handle=service_handles["get_device_state"],
                                         expected_states=(GetDeviceStateResponse.STATE_IDLE,))

        # Disconnect device, expect UNKNOWN state
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_UNKNOWN,
                                                         timeout=3.0)

        # Connect, expect IDLE state
        connect_response = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response.success, "Sensor failed to connect")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_IDLE,
                                                         timeout=3.0)

        # Start capture, expect START_CAPTURE state, followed by CAPTURE_RUNNING later on
        start_response = service_handles["start_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=start_response, success_expected=True)
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_STARTING,
                                                         timeout=3.0)
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING,
                                                         timeout=60.0)

        # Begin static pose, expect CAPTURE_RUNNING_STATIC and then CAPTURE_RUNNING after ending static pose
        begin_response = service_handles["begin_static_pose"].call(TriggerRequest())
        self.assertTrue(begin_response.success, "Device failed to begin a static pose")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING_STATIC,
                                                         timeout=5.0)
        end_response = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertTrue(end_response.success, "Device failed to end a static pose")
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING,
                                                         timeout=5.0)

        # Stop capture, expect state CAPTURE_STOPPING
        stop_response = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response, success_expected=True)
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_CAPTURE_STOPPING,
                                                         timeout=3.0)

        # State will return to IDLE eventually after stop capture is called
        test_helpers.wait_and_assert_blk_arc_is_in_state(test_case=self,
                                                         get_device_state_handle=service_handles["get_device_state"],
                                                         state=GetDeviceStateResponse.STATE_IDLE,
                                                         timeout=60.0)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_get_device_state', TestGetDeviceState)
