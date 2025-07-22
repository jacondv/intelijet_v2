#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import time
import unittest

from ros_blkarc_msgs.srv import CaptureRequest


class TestStartCaptureWhileConnected(unittest.TestCase):

    # Test if start capture service call behaves correctly when sensor is connected
    def test_start_capture(self) -> None:

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Call start scan the first time, should be successful
        response_1 = service_handles["start_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=response_1, success_expected=True)

        # Wait for a bit
        time.sleep(1.0)

        # Call start scan the second time, should not be successful since it is already scanning
        response_2 = service_handles["start_capture"].call(CaptureRequest())
        test_helpers.assert_capture_service(test_case=self, response=response_2, success_expected=False)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_start_capture_while_connected', TestStartCaptureWhileConnected)
