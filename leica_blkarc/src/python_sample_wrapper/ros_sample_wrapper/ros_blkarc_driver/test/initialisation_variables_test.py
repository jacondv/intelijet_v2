#!/usr/bin/env python3

PKG = 'ros_blkarc_driver'

import os
import sys
import unittest

import rospy
import test_helpers

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '../../BLKARC-Sample-Wrapper/python/')

from pathlib import Path

from blk_arc_sample_wrapper.blk_arc_config import ConnectionType


class TestInitialisationVariables(unittest.TestCase):

    # Test if class variables have been initialised correctly
    def test_initiated_sensor_variables(self) -> None:

        # Initialise ROS wrapper
        blkarc_ros_wrapper = test_helpers.setup_blkarc_node()

        # Retrieve namespaced ROS params
        sensor_name_unscoped = rospy.get_param(param_name="sensor_name", default=None)
        expected_sensor_name = rospy.get_param(param_name=f"/{sensor_name_unscoped}/sensor_name", default=None)
        expected_connection_type_int = rospy.get_param(param_name=f"/{sensor_name_unscoped}/connection_type",
                                                       default=None)
        expected_connection_type = ConnectionType(expected_connection_type_int) if expected_connection_type_int in [
            0, 1
        ] else None
        expected_save_scan_directory = rospy.get_param(param_name=f"/{sensor_name_unscoped}/scan_save_directory",
                                                       default=None)
        expected_save_scan_path = Path(
            expected_save_scan_directory).expanduser() if expected_save_scan_directory else None

        wrapper_sensor_name = blkarc_ros_wrapper._sensor_name
        wrapper_connection_type = blkarc_ros_wrapper._blk_arc_connection_type
        wrapper_scan_save_path = blkarc_ros_wrapper._scan_save_path

        self.assertEqual(
            expected_sensor_name, wrapper_sensor_name,
            f"sensor_name in rosparam ({expected_sensor_name}) does not match the one initialised in wrapper ({wrapper_sensor_name})"
        )
        self.assertEqual(
            expected_connection_type, wrapper_connection_type,
            f"expected_connection_type in rosparam ({expected_connection_type}) does not match the one initialised in wrapper ({wrapper_connection_type})"
        )
        self.assertEqual(
            expected_save_scan_path, wrapper_scan_save_path,
            f"save_scan_directory (evaluated to full path) in rosparam ({expected_save_scan_path}) does not match the one initialised in wrapper ({wrapper_scan_save_path})"
        )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_initialisation_variables', TestInitialisationVariables)
