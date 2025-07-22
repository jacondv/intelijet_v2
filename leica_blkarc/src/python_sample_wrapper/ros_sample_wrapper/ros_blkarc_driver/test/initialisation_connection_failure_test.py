#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import rospy
import unittest

from ros_blkarc_driver import BLKARCROSWrapper


class TestInitialisationConnectionFailure(unittest.TestCase):

    # Test to check if exception is raised if initialisation of BLKARCROSWrapper fails
    def test_initialisation_connection_success(self) -> None:

        # Retrieve rosparam /sensor_name/connection_type
        sensor_name = rospy.get_param(param_name="sensor_name", default=None)
        connection_type_int = rospy.get_param(param_name=f"/{sensor_name}/connection_type", default=None)
        # Just make sure connection type value was retrieved successfully
        self.assertIsNotNone(connection_type_int, f"Unsuccessfully retrieved rosparam /{sensor_name}/connection_type")
        self.assertTrue(
            connection_type_int == 0 or connection_type_int == 1,
            f"Invalid value for rosparam /{sensor_name}/connection_type. Should be either 0 or 1, got {connection_type_int}"
        )

        # We change the /connection_type rosparam to be opposite of the one specified
        new_connection_type_int = 1 if connection_type_int == 0 else 0
        rospy.set_param(param_name=f"/{sensor_name}/connection_type", param_value=new_connection_type_int)

        # Initialise ROS wrapper, we expect an exception to be raised
        rospy.init_node("blkarc_ros_wrapper")
        self.assertRaises(RuntimeError, lambda: BLKARCROSWrapper())


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_initialisation_connection_failure', TestInitialisationConnectionFailure)
