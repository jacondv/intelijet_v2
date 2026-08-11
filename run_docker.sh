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

# Let the container connect to this session's X server.
xhost +local:docker >/dev/null 2>&1

# Tablet screen rotation - no-op (harmless) on machines/monitors without a
# DSI-1 output.
xrandr --output DSI-1 --rotate right 2>/dev/null || true

trap 'xhost -local:docker >/dev/null 2>&1' EXIT

$COMPOSE_CMD down
$COMPOSE_CMD up -d

echo "Intelijet container restarted. Logs: $COMPOSE_CMD logs -f"
