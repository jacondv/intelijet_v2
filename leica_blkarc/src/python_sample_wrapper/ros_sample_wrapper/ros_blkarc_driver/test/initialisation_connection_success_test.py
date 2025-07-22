#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import test_helpers
import unittest


class TestInitialisationConnectionSuccess(unittest.TestCase):

    # Test if after successful initialisation of BLKARCROSWrapper, it connects to the sensor
    def test_initialisation_connection_success(self) -> None:
        # Initialise ROS wrapper
        blkarc_ros_wrapper = test_helpers.setup_blkarc_node()
        self.assertTrue(blkarc_ros_wrapper._blk_arc.is_connected(), "Device failed to connect upon wrapper start up.")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_initialisation_connection_success', TestInitialisationConnectionSuccess)
