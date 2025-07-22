cmake_minimum_required(VERSION 3.0.2)
project(ros_blkarc_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg
  actionlib_msgs
  actionlib
)

add_action_files(DIRECTORY action FILES TimedScan.action)

add_service_files(DIRECTORY srv
  FILES
  Capture.srv
  DownloadScan.srv
  GetDeviceState.srv
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
