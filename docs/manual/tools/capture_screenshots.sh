#!/bin/bash
# Re-capture the operator-manual screenshots (docs/manual/assets/*.png) from
# the real UI, then rebuild Intelijet_PPS_Manual.html. Runs the app's own
# Docker image with the repo mounted; the app must NOT be running (both
# would use the same container name / settings). Read-only on the data -
# see capture_screenshots.py's docstring.
#
# Usage: docs/manual/tools/capture_screenshots.sh   (needs the desktop session: the
#        app window shows full-screen for ~1 min - the 3D view needs a real display)
set -e

REPO_DIR="$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)"
IMAGE="jacondv/jacon-pps-noetic:latest"
QPA="${QPA:-xcb}"

if docker ps --format '{{.Names}}' | grep -qx intelijet; then
    echo "The Intelijet app is running - close it first." >&2
    exit 1
fi

xhost +local:docker >/dev/null

docker run --rm --network host \
    -e DISPLAY="${DISPLAY:-:0}" -e QT_QPA_PLATFORM="$QPA" -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/root/.Xauthority -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e QT_AUTO_SCREEN_SCALE_FACTOR=0 -e QT_SCALE_FACTOR=1.0 -e QT_FONT_DPI=96 \
    -e TZ="$(cat /etc/timezone 2>/dev/null || echo :/etc/localtime)" -e HOST_UTC_OFFSET="$(date +%z)" \
    -e PYTHONUNBUFFERED=1 \
    -v "$REPO_DIR":/root/intelijet_v2 \
    -v "${XAUTHORITY:-$HOME/.Xauthority}":/root/.Xauthority:ro \
    -v "$REPO_DIR/intelijet_v2_ws/src/config/.config":/root/.config/JaconEquipment:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /etc/localtime:/etc/localtime:ro \
    -v /dev/dri:/dev/dri \
    "$IMAGE" bash -c '
        source /root/intelijet_v2/intelijet_v2_ws/devel/setup.bash &&
        cd /root/intelijet_v2/docs/manual/tools &&
        python3 capture_screenshots.py /root/intelijet_v2/docs/manual/assets'

python3 "$REPO_DIR/docs/manual/tools/build_html.py"
