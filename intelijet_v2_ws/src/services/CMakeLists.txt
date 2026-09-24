cmake_minimum_required(VERSION 3.0.2)
project(services)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  message_generation
)

## Ensures modules/scripts declared in setup.py get installed into the
## devel space - see setup.py's own comment.
catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES
    ExportReport.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy actionlib actionlib_msgs std_msgs
)

# (no catkin_install_python() here - pps.launch runs scripts/*.py directly
# from the source tree via rosrun/roslaunch, no install-space copy needed -
# same convention as pps/CMakeLists.txt)
