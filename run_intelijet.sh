#!/bin/bash
echo ">>> Setup trong container ..."
cd /root/intelijet_v2/intelijet_v2_ws || exit 1
source devel/setup.bash
roslaunch pps pps.launch
exec bash
