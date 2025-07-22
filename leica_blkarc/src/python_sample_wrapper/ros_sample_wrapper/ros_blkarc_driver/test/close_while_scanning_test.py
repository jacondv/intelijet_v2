#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import test_helpers
import time
import unittest


class TestCloseWhileScanning(unittest.TestCase):

    def test_close(self) -> None:
        # Initialise ROS wrapper
        blkarc_ros_wrapper = test_helpers.setup_blkarc_node()
        self.assertTrue(
            blkarc_ros_wrapper._blk_arc.wait_till_idle(timeout=test_helpers.DEFAULT_WAIT_FOR_DEVICE_TO_REACH_STATE),
            "Device is not in IDLE state at the start of test, this may cause unexpected test results")

        # Check sensor is connected
        self.assertTrue(blkarc_ros_wrapper._blk_arc.is_connected(), "Device failed to connect upon wrapper start up.")

        # Start scanning
        blkarc_ros_wrapper._blk_arc.start_capture()
        self.assertTrue(blkarc_ros_wrapper._scanning_in_progress(), "Device failed to start scanning.")

        # Wait for a bit
        time.sleep(1.0)

        # Ensure sensor is still scanning
        self.assertTrue(blkarc_ros_wrapper._scanning_in_progress(), "Device failed to start scanning.")

        # Close ROS wrapper, we expect it to be disconnected from device
        blkarc_ros_wrapper.close()
        self.assertFalse(blkarc_ros_wrapper._blk_arc.is_connected(), "Devices connected after ROS wrapper was closed")

        # Ensure scanning stops (need to reconnect to ensure this)
        self.assertTrue(blkarc_ros_wrapper._attempt_connection(), "Device failed to connect")
        self.assertTrue(
            blkarc_ros_wrapper._blk_arc.wait_till_idle(timeout=test_helpers.DEFAULT_WAIT_FOR_DEVICE_TO_REACH_STATE),
            "Device is not in IDLE state after close() was called")
        self.assertFalse(blkarc_ros_wrapper._scanning_in_progress(), "Device failed to stop scanning.")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_close_while_scanning', TestCloseWhileScanning)
