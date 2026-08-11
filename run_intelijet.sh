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

# dbus-run-session: starts a private D-Bus session bus and exports
# DBUS_SESSION_BUS_ADDRESS for everything run inside it - the container has
# none by default. Needed for ui/src/ui/keyboard.py's TouchKeyboard, which
# shows/hides the on-screen keyboard (onboard) via its D-Bus Show/Hide
# methods instead of killing/relaunching the process (see comment there),
# and for the one-time `gsettings` calls that dock it to the bottom edge.
# Without a bus, both silently no-op and onboard never appears.
dbus-run-session -- bash -c "roslaunch pps pps.launch; exec bash"
