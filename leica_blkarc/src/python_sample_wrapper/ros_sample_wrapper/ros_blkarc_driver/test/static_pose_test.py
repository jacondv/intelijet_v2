#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import unittest

from ros_blkarc_msgs.srv import CaptureRequest, GetDeviceStateResponse
from std_srvs.srv import TriggerRequest


class TestStaticPose(unittest.TestCase):

    # Test if begin/end static pose works correctly
    def test_static_pose(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Begin/End Static Pose when in IDLE state, should be unsuccessful
        begin_response_1 = service_handles["begin_static_pose"].call(TriggerRequest())
        self.assertFalse(begin_response_1.success, "Device unexpectedly began a static pose")
        end_response_1 = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertFalse(end_response_1.success, "Device unexpectedly ended a static pose")

        # Ensure scanner is ready to scan
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Start scan
        start_response = service_handles["start_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=start_response, success_expected=True)

        # Wait till scan is running
        test_helpers.wait_till_blk_arc_is_in_state(get_device_state_handle=service_handles["get_device_state"],
                                                   state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING)

        # End Static Pose before beginning one, should be unsuccessful
        end_response_2 = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertFalse(end_response_2.success, "Device unexpectedly ended a static pose")

        # Begin, then end the static pose, should be successful
        begin_response_2 = service_handles["begin_static_pose"].call(TriggerRequest())
        self.assertTrue(begin_response_2.success, "Device failed to begin a static pose")

        self.assertTrue(
            test_helpers.wait_till_blk_arc_is_in_state(get_device_state_handle=service_handles["get_device_state"],
                                                       state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING_STATIC),
            "Failed to achieve static capture state after beginning a static pose")

        end_response_3 = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertTrue(end_response_3.success, "Device failed to end a static pose")

        self.assertTrue(
            test_helpers.wait_till_blk_arc_is_in_state(get_device_state_handle=service_handles["get_device_state"],
                                                       state=GetDeviceStateResponse.STATE_CAPTURE_RUNNING),
            "Failed to achieve normal capture state after ending a static pose")

        # Calling End Static Pose again should be unsuccessful
        end_response_4 = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertFalse(end_response_4.success, "Device unexpectedly ended a static pose")

        # Disconnect
        disconnect_response_1 = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response_1.success, "Sensor failed to disconnect")

        # Begin/End Static Pose when disconnected to scanner, should be unsuccessful
        begin_response_3 = service_handles["begin_static_pose"].call(TriggerRequest())
        self.assertFalse(begin_response_3.success, "Device unexpectedly began a static pose")
        end_response_5 = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertFalse(end_response_5.success, "Device unexpectedly ended a static pose")

        # Connect
        connect_response = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response.success, "Sensor failed to connect")

        # Stop scan, should be successful
        stop_response_3 = service_handles["stop_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=stop_response_3, success_expected=True)

        # Wait till scanner has stopped capture completely
        test_helpers.wait_till_blk_arc_is_idle(get_device_state_handle=service_handles["get_device_state"])

        # Disconnect
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")

        # Begin/End Static Pose, should be unsuccessful
        begin_response_4 = service_handles["begin_static_pose"].call(TriggerRequest())
        self.assertFalse(begin_response_4.success, "Device unexpectedly began a static pose")
        end_response_6 = service_handles["end_static_pose"].call(TriggerRequest())
        self.assertFalse(end_response_6.success, "Device unexpectedly ended a static pose")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_static_pose', TestStaticPose)
