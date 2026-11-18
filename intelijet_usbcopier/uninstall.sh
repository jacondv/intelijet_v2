#!/bin/bash
# ============================================================
# USB Data Copier - Uninstall Service
# ============================================================

APP_NAME="usb-copier"
SERVICE_FILE="/etc/systemd/system/${APP_NAME}.service"

echo "╔══════════════════════════════════════╗"
echo "║     USB DATA COPIER UNINSTALL       ║"
echo "╚══════════════════════════════════════╝"

# ------------------------------------------------------------
# Stop service
# ------------------------------------------------------------
echo "⚙ Stopping service..."
sudo systemctl stop ${APP_NAME}.service 2>/dev/null

# ------------------------------------------------------------
# Disable service
# ------------------------------------------------------------
echo "⚙ Disabling service..."
sudo systemctl disable ${APP_NAME}.service 2>/dev/null

# ------------------------------------------------------------
# Remove service file
# ------------------------------------------------------------
if [ -f "$SERVICE_FILE" ]; then
    echo "⚙ Removing service file..."
    sudo rm -f "$SERVICE_FILE"
fi

# ------------------------------------------------------------
# Reload systemd
# ------------------------------------------------------------
echo "⚙ Reloading systemd..."
sudo systemctl daemon-reload

# ------------------------------------------------------------
# Reset failed states
# ------------------------------------------------------------
sudo systemctl reset-failed

# ------------------------------------------------------------
# Done
# ------------------------------------------------------------
echo ""
echo "✅ USB Data Copier service removed"
echo ""