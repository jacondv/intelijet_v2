#!/bin/bash

# [ "$PWD" != "/mnt/c/work/projects/intelijet_v2/leica_blkarc" ] 
cd /mnt/c/work/projects/intelijet_v2/leica_blkarc

export ROS_MASTER_URI=http://localhost:11311

source .venv/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.bash

if ! rostopic list > /dev/null 2>&1; then
    roscore &
    sleep 2 
else
    echo "[INFO] roscore running."
fi

roslaunch ros_blkarc_driver blkarc_bringup.launch


# export PYTHONPATH=$(pwd)/src/python_sample_wrapper:/opt/ros/noetic/lib/python3/dist-packages
# export ROS_PACKAGE_PATH=$(pwd)/src/python_sample_wrapper/ros_sample_wrapper:$ROS_PACKAGE_PATH
# rosrun ros_blkarc_driver blkarc_ros_node
