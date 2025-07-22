#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import unittest

from ros_blkarc_msgs.srv import CaptureRequest
from std_srvs.srv import TriggerRequest


class TestStartCaptureWhileDisonnected(unittest.TestCase):

    # Test if start capture service call behaves correctly when sensor is disconnected
    def test_start_capture(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Disconnect, then check if disconnection is successful
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success, "Sensor failed to disconnect")

        # Call start scan should not be successful, sensor needs to be connected in order to start scan
        response_1 = service_handles["start_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=response_1, success_expected=False)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_start_capture_while_disconnected', TestStartCaptureWhileDisonnected)
