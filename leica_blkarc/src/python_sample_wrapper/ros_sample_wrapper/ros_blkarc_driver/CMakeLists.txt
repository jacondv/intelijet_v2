cmake_minimum_required(VERSION 3.0.2)
project(ros_blkarc_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy std_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_srvs
)

catkin_install_python(PROGRAMS nodes/blkarc_ros_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/initialisation_connection_success_test.test)
  add_rostest(test/initialisation_connection_failure_test.test)
  add_rostest(test/initialisation_variables_test.test)
  add_rostest(test/initialisation_invalid_connection_type_test.test)
  add_rostest(test/close_while_connected_test.test)
  add_rostest(test/close_while_scanning_test.test)
  add_rostest(test/connect_test.test)
  add_rostest(test/disconnect_test.test)
  add_rostest(test/start_capture_while_connected_test.test)
  add_rostest(test/start_capture_while_disconnected_test.test)
  add_rostest(test/stop_capture_test.test)
  add_rostest(test/download_scan_test.test)
  add_rostest(test/timed_scan_test.test)
  add_rostest(test/get_device_state_test.test)
  add_rostest(test/reboot_test.test)
  add_rostest(test/static_pose_test.test)
  add_rostest(test/trigger_detail_image_test.test)
endif()
