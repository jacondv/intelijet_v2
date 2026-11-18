#!/usr/bin/env bash
# One-shot Syncthing install/setup for ONE tablet (run on BOTH tablets
# separately - see intallation_gui.md's "Folder Sync Setup with Syncthing"
# section for the full manual pairing/folder steps this can't automate:
# exchanging Device IDs and confirming folder shares needs a human on each
# side, in the Syncthing web GUI).
#
# What this script does automate:
#   - install Syncthing from its official APT repo (idempotent - skips if
#     already installed)
#   - enable + start it as a per-user systemd service (not root)
#   - create the two folders it will sync, if they don't exist yet, so
#     "Add Folder" in the GUI has something real to point at
#   - print this tablet's Device ID and the exact folder paths to use in
#     the GUI, so there's nothing to mistype
#
# Usage (from a clone of this repo, on each tablet):
#   ./install_syncthing.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Mirrors commond.yaml's BASE_DIR=/root/intelijet_v2 + DATA_DIR=data,
# resolved to the equivalent HOST path (docker-compose.yml bind-mounts
# REPO_DIR itself as /root/intelijet_v2 inside the container) - keep this
# in sync with commond.yaml if BASE_DIR/DATA_DIR ever change.
CONFIG_FOLDER="$REPO_DIR/intelijet_v2_ws/src/config"
RESULTS_FOLDER="$REPO_DIR/data/Projects"

echo ">>> Checking for Syncthing..."
if ! command -v syncthing >/dev/null 2>&1; then
    echo ">>> Syncthing not found - installing from the official APT repo (needs sudo)..."
    curl -s https://syncthing.net/release-key.txt | sudo apt-key add -
    echo "deb https://apt.syncthing.net/ syncthing stable" | sudo tee /etc/apt/sources.list.d/syncthing.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y syncthing
else
    echo "Syncthing already installed: $(syncthing --version | head -n1)"
fi

echo ">>> Enabling syncthing@$USER as a per-user systemd service..."
sudo systemctl enable "syncthing@$USER" --now

echo ">>> Waiting for the service to come up..."
for _ in $(seq 1 10); do
    systemctl is-active --quiet "syncthing@$USER" && break
    sleep 1
done
systemctl status "syncthing@$USER" --no-pager --lines=0

echo ">>> Creating folders to sync (if missing)..."
mkdir -p "$CONFIG_FOLDER" "$RESULTS_FOLDER"

echo
echo "=================================================================="
echo " Syncthing is installed and running on this tablet."
echo
echo " Remaining steps are manual, in the web GUI - see intallation_gui.md"
echo " section \"Folder Sync Setup with Syncthing\" for full detail:"
echo
echo "   1. Open http://127.0.0.1:8384 on THIS tablet."
echo "   2. Actions -> Show ID, copy this tablet's Device ID."
echo "   3. On BOTH tablets: Remote Devices -> Add Remote Device, pair them."
echo "   4. Add these 2 folders (paths already created above), share both"
echo "      with the other tablet:"
echo "        scanner_config  -> $CONFIG_FOLDER"
echo "        scanner_results -> $RESULTS_FOLDER"
echo "   5. Confirm both show Connected / Up to Date on both tablets."
echo "=================================================================="
