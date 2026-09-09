#!/usr/bin/env bash
# One-shot setup on a fresh Linux machine: install Docker if missing, build
# the app image once, and drop a working desktop icon. After this, the icon
# is the only thing needed to run the app (see run_docker.sh).
#
# Usage (from a clone of this repo):
#   ./install.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

echo ">>> Checking for curl..."
if ! command -v curl >/dev/null 2>&1; then
    echo ">>> curl not found - installing (needs sudo)..."
    sudo apt-get update
    sudo apt-get install -y curl
fi

echo ">>> Checking for Docker..."
if ! command -v docker >/dev/null 2>&1; then
    echo ">>> Docker not found - installing via get.docker.com (needs sudo)..."
    curl -fsSL https://get.docker.com | sudo sh
else
    echo "Docker already installed: $(docker --version)"
fi

if ! docker compose version >/dev/null 2>&1 && ! command -v docker-compose >/dev/null 2>&1; then
    echo ">>> Docker Compose plugin not found - installing..."
    sudo apt-get update
    sudo apt-get install -y docker-compose-plugin
fi

if id -nG "$USER" | grep -qw docker; then
    echo "User '$USER' already in the docker group."
    NEEDS_RELOGIN=0
else
    echo ">>> Adding '$USER' to the docker group..."
    sudo usermod -aG docker "$USER"
    NEEDS_RELOGIN=1
fi

if command -v docker-compose >/dev/null 2>&1 && ! docker compose version >/dev/null 2>&1; then
    COMPOSE_CMD="docker-compose"
else
    COMPOSE_CMD="docker compose"
fi

echo ">>> Building the app image (one-time - later runs reuse it, no rebuild)..."
if [ "$NEEDS_RELOGIN" = "1" ]; then
    # Group membership only takes effect in a new login session - sg runs
    # the build with the docker group active in this one without requiring
    # that yet.
    sg docker -c "$COMPOSE_CMD build"
else
    $COMPOSE_CMD build
fi

echo ">>> Installing desktop icon..."
APPS_DIR="$HOME/.local/share/applications"
mkdir -p "$APPS_DIR"
DESKTOP_FILE="$APPS_DIR/intelijet.desktop"
cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Intelijet
Comment=Jacon Intelijet
Exec=$REPO_DIR/run_docker.sh
Icon=$REPO_DIR/PPSicon.png
Terminal=false
StartupWMClass=Intelijet
TryExec=$REPO_DIR/run_docker.sh
EOF
chmod +x "$DESKTOP_FILE"

# Also drop a copy on the Desktop, if one exists, so there's a clickable
# icon without hunting through the app menu.
if [ -d "$HOME/Desktop" ]; then
    cp "$DESKTOP_FILE" "$HOME/Desktop/intelijet.desktop"
    chmod +x "$HOME/Desktop/intelijet.desktop"
    # GNOME/most modern DEs refuse to trust a just-copied .desktop file
    # until this is set (shows as "Untrusted launcher" otherwise).
    command -v gio >/dev/null 2>&1 && gio set "$HOME/Desktop/intelijet.desktop" metadata::trusted true 2>/dev/null || true
fi

echo ""
echo ">>> Done."
if [ "$NEEDS_RELOGIN" = "1" ]; then
    echo "Log out and back in (so the 'docker' group applies to your login"
    echo "session), then use the Intelijet icon."
else
    echo "Use the Intelijet icon to start the app."
fi
