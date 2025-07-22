#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import test_helpers
import unittest


class TestCloseWhileConnected(unittest.TestCase):

    # Test if close() disconnects sensor
    def test_close(self) -> None:
        # Initialise ROS wrapper
        blkarc_ros_wrapper = test_helpers.setup_blkarc_node()

        # Check sensor is connected
        self.assertTrue(blkarc_ros_wrapper._blk_arc.is_connected(), "Device failed to connect upon wrapper start up.")

        # close ROS wrapper, we expect it to be disconnected with the device in the end
        blkarc_ros_wrapper.close()
        self.assertFalse(blkarc_ros_wrapper._blk_arc.is_connected(), "Devices connected after ROS wrapper was closed")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_close_while_connected', TestCloseWhileConnected)
