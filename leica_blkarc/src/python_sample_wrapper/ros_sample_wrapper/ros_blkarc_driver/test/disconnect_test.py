#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import unittest

from std_srvs.srv import TriggerRequest


class TestDisconnect(unittest.TestCase):

    # Test if disconnect service call works as intended
    def test_disconnect(self) -> None:

        # Initialise ROS wrapper
        blkarc_ros_wrapper = test_helpers.setup_blkarc_node()
        self.assertTrue(blkarc_ros_wrapper._blk_arc.is_connected(), "Device failed to connect upon wrapper start up.")

        # Initialise services and ensure device is ready
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        device_is_ready, service_handles = test_helpers.get_all_service_handles_and_ensure_device_is_idle(
            sensor_name=sensor_name)
        self.assertTrue(device_is_ready,
                        "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Disconnect
        disconnect_response_1 = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response_1.success,
                        "Disconnect service call failed to disconnect sensor when it was supposed to succeed")
        self.assertFalse(
            blkarc_ros_wrapper._blk_arc.is_connected(),
            "BLK is connected after disconnect service is called and its disconnect service response indicated sensor is disconnected."
        )

        # Disconnect again, sensor should remain disconnected
        # Note: success in service response indicates that sensor is disconnected (true if sensor is disconnected after service call)
        disconnect_response_2 = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response_2.success,
                        "Disconnect service call failed to disconnect sensor when it was supposed to succeed")
        self.assertFalse(
            blkarc_ros_wrapper._blk_arc.is_connected(),
            "BLK is connected after disconnect service is called and its disconnect service response indicated sensor is disconnected."
        )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_disconnect', TestDisconnect)
