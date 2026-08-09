#!/bin/bash
echo ">>> Setup container ..."
cd /root/intelijet_v2/intelijet_v2_ws || exit 1
export DISABLE_ROS1_EOL_WARNINGS=1
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCREEN_SCALE_FACTORS=1.0
export QT_SCALE_FACTOR=1.0

if [ ! -f devel/setup.bash ]; then
    echo ">>> No devel/ yet, building workspace (first run only)..."
    source /opt/ros/noetic/setup.bash
    catkin build
fi

source devel/setup.bash
roslaunch pps pps.launch
exec bash
