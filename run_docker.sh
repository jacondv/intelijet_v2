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
$COMPOSE_CMD up -d

echo "Intelijet container restarted. Live logs: $COMPOSE_CMD logs -f"
echo "Previous run's log saved to: $LOG_DIR/last_run.log"
