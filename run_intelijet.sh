#!/bin/bash
echo ">>> Setup container ..."
cd /root/intelijet_v2/intelijet_v2_ws || exit 1
export DISABLE_ROS1_EOL_WARNINGS=1
# This is a fixed kiosk touchscreen, not a general desktop app - it must
# always fill whatever real screen it's given rather than growing/shrinking
# its own widget geometry with the host's accessibility scale setting.
# GNOME's Display scale slider on this hardware genuinely changes the RandR
# screen size Qt fullscreens into (confirmed via `xrandr`: e.g. 1920x1200 at
# one scale vs 2560x1600 at another - it's a real CRTC transform, not just a
# DPI hint), and app.py's showFullScreen() already tracks that correctly on
# its own now that the UI's fixed-pixel layout (see intelijet_ui.py) no
# longer forces a minimum size bigger than the screen (fixed in app.py's
# _sync_tab_size_policies). QT_AUTO_SCREEN_SCALE_FACTOR/QT_SCALE_FACTOR are
# left off/1.0 here so Qt doesn't ALSO apply its own widget-geometry
# multiplier on top of that already-real screen size (would double-scale).
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCREEN_SCALE_FACTORS=1.0
export QT_SCALE_FACTOR=1.0
# Independent of the above: Qt still converts the app's `pt`-based
# stylesheet font sizes (see intelijet_ui.py's many `font-size: ...pt`
# rules) to pixels using the host's font DPI (Xft.dpi), which GNOME also
# raises with the scale slider (e.g. 96 -> 144 at 150%, -> 192 at 200%).
# Left alone, that made text render up to 2x its designed pixel size and
# overflow the UI's fixed-width buttons/panels/labels at higher host
# scale. Pinning the font DPI to the standard 96 makes text render at a
# consistent, designed-for size regardless of the host's scale setting.
export QT_FONT_DPI=96

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
#
# `docker compose down`/`docker stop` sends SIGTERM to this script (PID 1
# in the container), then SIGKILLs everything left after its ~10s grace
# period - too abrupt for onboard/qpdfview to save anything not already
# flushed to their mounted config dirs (see docker-compose.yml). Plain bash
# doesn't forward signals to child processes on its own, so without the
# trap below they'd always hit that SIGKILL mid-write. Backgrounding the
# session + `wait` (instead of running it as a plain foreground command)
# is what lets the trap fire right away instead of only after the session
# exits.
_shutdown() {
    pkill -TERM -x qpdfview 2>/dev/null
    pkill -TERM -x onboard 2>/dev/null
    sleep 1
    exit 0
}
trap _shutdown TERM INT

dbus-run-session -- bash -c "roslaunch pps pps.launch; exec bash" &
wait "$!"
