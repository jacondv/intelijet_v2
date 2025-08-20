#!/bin/bash
echo ">>> Setup container ..."
cd /root/intelijet_v2/intelijet_v2_ws || exit 1
export DISABLE_ROS1_EOL_WARNINGS=1
source devel/setup.bash
roslaunch pps pps.launch
exec bash
