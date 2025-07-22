#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import test_helpers
import unittest

from std_srvs.srv import TriggerRequest


class TestConnect(unittest.TestCase):

    # Test if connect service call works as intended
    def test_connect(self) -> None:

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
        disconnect_response = service_handles["disconnect"].call(TriggerRequest())
        self.assertTrue(disconnect_response.success,
                        "Disconnect service call failed to disconnect sensor when it was supposed to succeed")
        self.assertFalse(
            blkarc_ros_wrapper._blk_arc.is_connected(),
            "BLK is connected after disconnect service is called and its disconnect service response indicated sensor is disconnected."
        )

        # Connect
        connect_response_1 = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response_1.success,
                        "Connect service call failed to connect sensor when it was supposed to succeed")
        self.assertTrue(
            blkarc_ros_wrapper._blk_arc.is_connected(),
            "BLK is disconnected after connect service is called and its connect service response indicated sensor is connected."
        )

        # Connect again, sensor should remain connected
        # Note: success in service response indicates that sensor is connected (true if sensor is connected after service call)
        connect_response_2 = service_handles["connect"].call(TriggerRequest())
        self.assertTrue(connect_response_2.success,
                        "Connect service call failed to connect sensor when it was supposed to succeed")
        self.assertTrue(
            blkarc_ros_wrapper._blk_arc.is_connected(),
            "BLK is disconnected after connect service is called and its connect service response indicated sensor is connected."
        )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_connect', TestConnect)
