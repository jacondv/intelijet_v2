#!/bin/bash
# Desktop icon entry point (see install.sh, which generates intelijet.desktop
# pointing here). Every run: stop whatever's currently up, then start a
# fresh container from the already-built image - no rebuild, so this stays
# fast (a few seconds), and always starts clean instead of attaching to
# whatever state a previous run left behind.
set -e

REPO_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
cd "$REPO_DIR"

if command -v docker-compose >/dev/null 2>&1 && ! docker compose version >/dev/null 2>&1; then
    COMPOSE_CMD="docker-compose"
else
    COMPOSE_CMD="docker compose"
fi

if ! id -nG "$USER" | grep -qw docker; then
    echo "User '$USER' is not in the docker group yet - run install.sh first."
    exit 1
fi

# Only one Intelijet version should touch the real hardware/ROS master at
# once (every version uses --network host) - stop the other test-version
# container (see run_intelijet_v2_1.sh) first.
docker stop intelijet_v2_1 >/dev/null 2>&1 || true

# Let the container connect to this session's X server. Belt-and-suspenders
# alongside docker-compose.yml's Xauthority mount (the main mechanism) -
# some X server/session setups need this too. Not swallowed silently
# anymore: if DISPLAY isn't set or xhost fails, that's exactly why the app
# would fail with "Authorization required, but no authorization protocol
# specified" / "could not connect to display", so it needs to be visible.
if [ -z "$DISPLAY" ]; then
    echo "WARNING: \$DISPLAY is not set in this shell - the app will likely fail to show its window." >&2
elif ! xhost +local:docker; then
    echo "WARNING: 'xhost +local:docker' failed - the app may fail to connect to the X server." >&2
fi

# Tablet screen rotation - no-op (harmless) on machines/monitors without a
# DSI-1 output.
xrandr --output DSI-1 --rotate right 2>/dev/null || true
xrandr --output DSI1 --rotate right 2>/dev/null || true

# xrandr above only rotates the framebuffer - the touchscreen keeps
# reporting raw panel coordinates, so without this touches land where
# they'd be in the original (unrotated) orientation instead of where the
# rotated image now shows them. map-to-output derives the right
# transformation matrix from the output's current rotation instead of
# hardcoding one, so it keeps working if that ever changes. No-op
# (harmless) on machines without this touchscreen/output.
xinput map-to-output "pointer:Goodix Capacitive TouchScreen" DSI-1 2>/dev/null || true
xinput map-to-output "pointer:Goodix Capacitive TouchScreen" DSI1 2>/dev/null || true

# Lock orientation so GNOME's auto-rotate (accelerometer-driven, via
# iio-sensor-proxy on tablet hardware) doesn't undo the xrandr rotation
# above the moment the device is tilted. No-op (harmless) on machines/DEs
# without this GNOME schema.
gsettings set org.gnome.settings-daemon.peripherals.touchscreen orientation-lock true 2>/dev/null || true

trap 'xhost -local:docker >/dev/null 2>&1' EXIT

# `down` below removes the container outright - its logs go with it unless
# saved first. Dump whatever's there now (the previous run, e.g. a crash
# right before this restart) to a file that survives: logs/last_run.log
# for this run, logs/previous_run.log for the one before that. Under data/
# (== DATA_DIR, see commond.yaml) - same top-level dir the app already
# keeps its own runtime output (Projects/reports) in, not a new one-off
# location.
LOG_DIR="$REPO_DIR/data/logs"
mkdir -p "$LOG_DIR"
if docker inspect intelijet >/dev/null 2>&1; then
    [ -f "$LOG_DIR/last_run.log" ] && mv "$LOG_DIR/last_run.log" "$LOG_DIR/previous_run.log"
    docker logs intelijet > "$LOG_DIR/last_run.log" 2>&1
fi

$COMPOSE_CMD down

# Cleared before `up` so a leftover marker from the previous run can't be
# mistaken for this run's readiness signal below - see app.py's
# _mark_ui_ready(), which touches this file (over the repo bind mount)
# right before the main window shows.
READY_FILE="$REPO_DIR/data/.ui_ready"
rm -f "$READY_FILE"

$COMPOSE_CMD up -d

echo "Intelijet container restarted. Live logs: $COMPOSE_CMD logs -f"
echo "Previous run's log saved to: $LOG_DIR/last_run.log"

# Startup (ROS master + all nodes coming up) can take a while with nothing
# visible on screen - open a terminal tailing the container's logs so the
# user sees it's progressing, then close that terminal automatically once
# the UI's actually up (READY_FILE appears) instead of leaving it sitting
# on top of the app. Best-effort: a missing terminal emulator or a
# never-arriving marker (timeout) just means no progress terminal, not a
# failed launch - the app itself doesn't depend on any of this.
if command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -T "Intelijet - đang khởi động..." \
        -e bash -c "$COMPOSE_CMD logs -f" &
    LOGS_TERM_PID=$!

    (
        waited=0
        while [ ! -f "$READY_FILE" ] && [ "$waited" -lt 120 ] && kill -0 "$LOGS_TERM_PID" 2>/dev/null; do
            sleep 1
            waited=$((waited + 1))
        done
        kill "$LOGS_TERM_PID" 2>/dev/null
    ) &
else
    echo "WARNING: no x-terminal-emulator found - skipping the startup progress terminal." >&2
fi
