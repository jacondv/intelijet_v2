#!/bin/bash
# ============================================================
# USB Data Copier - Auto Install Service
# ============================================================

APP_NAME="usb-copier"
USER_NAME=$(whoami)

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYTHON_SCRIPT="$PROJECT_DIR/usb_copier.py"

SERVICE_FILE="/etc/systemd/system/${APP_NAME}.service"

echo "╔══════════════════════════════════════╗"
echo "║     USB DATA COPIER INSTALLER       ║"
echo "╚══════════════════════════════════════╝"

# ------------------------------------------------------------
# Check python
# ------------------------------------------------------------
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 chưa cài"
    sudo apt update
    sudo apt install -y python3
fi

# ------------------------------------------------------------
# Check tkinter
# ------------------------------------------------------------
python3 -c "import tkinter" 2>/dev/null || {
    echo "⚙ Installing tkinter..."
    sudo apt install -y python3-tk
}

# ------------------------------------------------------------
# Create service
# ------------------------------------------------------------
echo "⚙ Creating systemd service..."

sudo bash -c "cat > $SERVICE_FILE" <<EOF
[Unit]
Description=USB Data Copier Service
After=graphical.target

[Service]
Type=simple

WorkingDirectory=$PROJECT_DIR
ExecStart=/usr/bin/python3 $PYTHON_SCRIPT

Restart=always
RestartSec=3

Environment=DISPLAY=:0
Environment=XAUTHORITY=/home/$USER_NAME/.Xauthority

User=$USER_NAME

[Install]
WantedBy=graphical.target
EOF

# ------------------------------------------------------------
# Reload systemd
# ------------------------------------------------------------
echo "⚙ Reloading systemd..."

sudo systemctl daemon-reload

# ------------------------------------------------------------
# Enable service
# ------------------------------------------------------------
echo "⚙ Enabling service..."

sudo systemctl enable ${APP_NAME}.service

# ------------------------------------------------------------
# Start service
# ------------------------------------------------------------
echo "⚙ Starting service..."

sudo systemctl restart ${APP_NAME}.service

# ------------------------------------------------------------
# Done
# ------------------------------------------------------------
echo ""
echo "✅ INSTALL COMPLETED"
echo ""
echo "Service name:"
echo "   ${APP_NAME}.service"
echo ""
echo "Useful commands:"
echo "   sudo systemctl status ${APP_NAME}.service"
echo "   sudo systemctl restart ${APP_NAME}.service"
echo "   journalctl -u ${APP_NAME}.service -f"
echo ""