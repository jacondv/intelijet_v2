#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import unittest

from ros_blkarc_driver import BLKARCROSWrapper


class TestInitialisationInvalidConnectionType(unittest.TestCase):

    # Test to check that exception is raised if an invalid value for the ROS param connection_type was passed
    def test_initialisation_invalid_connection_type(self) -> None:

        # Set the ROS param /[sensor_name]/connection_type to be an invalid value (e.g. A negative one)
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        invalid_connection_type_int = -1
        rospy.set_param(param_name=f"/{sensor_name}/connection_type", param_value=invalid_connection_type_int)

        # Initialise ROS wrapper, we expect an exception to be raised
        rospy.init_node("blkarc_ros_wrapper")
        self.assertRaises(ValueError, lambda: BLKARCROSWrapper())


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_initialisation_invalid_connection_type', TestInitialisationInvalidConnectionType)
