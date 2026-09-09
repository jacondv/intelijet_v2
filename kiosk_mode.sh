#!/usr/bin/env bash
# Toggle this machine between:
#   - normal desktop (GDM/LightDM/SDDM login screen -> GNOME/etc, click the
#     Intelijet icon manually)
#   - kiosk mode: boot straight to tty1 autologin -> minimal X (openbox) ->
#     run_docker.sh, no login screen, no desktop shell.
#
# Every change this script makes is tracked in a state file so `restore`
# can put things back exactly, even on a machine you've never seen before.
#
# Usage (needs root - it edits systemd units and another user's dotfiles):
#   sudo ./kiosk_mode.sh enable [username]   # username defaults to $SUDO_USER
#   sudo ./kiosk_mode.sh restore
#   sudo ./kiosk_mode.sh status
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_FILE="/etc/intelijet-kiosk.conf"
MARK_BEGIN="# >>> intelijet-kiosk-mode >>>"
MARK_END="# <<< intelijet-kiosk-mode <<<"
GETTY_OVERRIDE_DIR="/etc/systemd/system/getty@tty1.service.d"
GETTY_OVERRIDE_FILE="$GETTY_OVERRIDE_DIR/override.conf"

require_root() {
    if [ "$(id -u)" != "0" ]; then
        echo "Run this with sudo: sudo $0 $*" >&2
        exit 1
    fi
}

detect_display_manager() {
    if [ -L /etc/systemd/system/display-manager.service ]; then
        basename "$(readlink -f /etc/systemd/system/display-manager.service)" .service
    else
        echo ""
    fi
}

cmd_status() {
    echo "=== Current mode ==="
    if [ -f "$STATE_FILE" ]; then
        echo "Kiosk mode: ENABLED"
        echo "--- $STATE_FILE ---"
        cat "$STATE_FILE"
    else
        echo "Kiosk mode: not enabled (normal desktop)"
    fi
    echo ""
    echo "Display manager unit: $(detect_display_manager || echo '(none detected)')"
    echo "getty@tty1 autologin override: $([ -f "$GETTY_OVERRIDE_FILE" ] && echo present || echo absent)"
}

cmd_enable() {
    require_root
    if [ -f "$STATE_FILE" ]; then
        echo "Kiosk mode already enabled (see: $0 status). Run 'restore' first if you want to redo it."
        exit 1
    fi

    KIOSK_USER="${1:-${SUDO_USER:-}}"
    if [ -z "$KIOSK_USER" ]; then
        echo "Could not determine which user to set up - pass it explicitly:" >&2
        echo "  sudo $0 enable <username>" >&2
        exit 1
    fi
    KIOSK_HOME="$(getent passwd "$KIOSK_USER" | cut -d: -f6)"
    if [ -z "$KIOSK_HOME" ] || [ ! -d "$KIOSK_HOME" ]; then
        echo "User '$KIOSK_USER' not found or has no home directory." >&2
        exit 1
    fi

    DM_SERVICE="$(detect_display_manager)"
    echo ">>> Detected display manager: ${DM_SERVICE:-none}"

    echo ">>> Saving current state to $STATE_FILE (needed for 'restore' later)..."
    cat > "$STATE_FILE" <<EOF
KIOSK_USER=$KIOSK_USER
KIOSK_HOME=$KIOSK_HOME
DISPLAY_MANAGER=$DM_SERVICE
REPO_DIR=$REPO_DIR
EOF

    if [ -n "$DM_SERVICE" ]; then
        echo ">>> Disabling $DM_SERVICE (takes effect on next boot, current session untouched)..."
        systemctl disable "$DM_SERVICE"
    else
        echo ">>> No display-manager.service symlink found - skipping disable step."
    fi

    echo ">>> Ensuring openbox is installed..."
    if ! dpkg -s openbox >/dev/null 2>&1; then
        apt-get update
        apt-get install -y openbox
    fi

    echo ">>> Setting up tty1 autologin for '$KIOSK_USER'..."
    mkdir -p "$GETTY_OVERRIDE_DIR"
    cat > "$GETTY_OVERRIDE_FILE" <<EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $KIOSK_USER --noclear %I \$TERM
EOF
    systemctl daemon-reload

    BASH_PROFILE="$KIOSK_HOME/.bash_profile"
    if [ -f "$BASH_PROFILE" ] && grep -qF "$MARK_BEGIN" "$BASH_PROFILE"; then
        echo ">>> $BASH_PROFILE already has the kiosk autostart block, leaving it."
    else
        echo ">>> Backing up and updating $BASH_PROFILE..."
        [ -f "$BASH_PROFILE" ] && cp "$BASH_PROFILE" "$BASH_PROFILE.pre-kiosk.bak"
        {
            echo "$MARK_BEGIN"
            echo 'if [ -z "$DISPLAY" ] && [ "$(tty)" = "/dev/tty1" ]; then'
            echo '    exec startx'
            echo 'fi'
            echo "$MARK_END"
        } >> "$BASH_PROFILE"
        chown "$KIOSK_USER":"$KIOSK_USER" "$BASH_PROFILE"
    fi

    XINITRC="$KIOSK_HOME/.xinitrc"
    if [ -f "$XINITRC" ] && ! grep -qF "$MARK_BEGIN" "$XINITRC"; then
        cp "$XINITRC" "$XINITRC.pre-kiosk.bak"
        echo ">>> Existing $XINITRC backed up to $XINITRC.pre-kiosk.bak"
    fi
    cat > "$XINITRC" <<EOF
#!/bin/bash
$MARK_BEGIN
openbox &
xset s off
xset -dpms
xset s noblank
exec $REPO_DIR/run_docker.sh
$MARK_END
EOF
    chmod +x "$XINITRC"
    chown "$KIOSK_USER":"$KIOSK_USER" "$XINITRC"

    echo ">>> Setting TeamViewer to run as a systemd service (independent of desktop login)..."
    if command -v teamviewer >/dev/null 2>&1; then
        teamviewer daemon enable || true
    else
        echo "    teamviewer command not found - skipping (install/configure it separately if needed)."
    fi

    echo ""
    echo ">>> Kiosk mode configured. Reboot to apply:"
    echo "    sudo reboot"
    echo ""
    echo "To undo everything later: sudo $0 restore"
}

cmd_restore() {
    require_root
    if [ ! -f "$STATE_FILE" ]; then
        echo "Kiosk mode isn't enabled (no $STATE_FILE) - nothing to restore."
        exit 0
    fi
    # shellcheck disable=SC1090
    source "$STATE_FILE"

    if [ -n "${DISPLAY_MANAGER:-}" ]; then
        echo ">>> Re-enabling $DISPLAY_MANAGER..."
        systemctl enable "$DISPLAY_MANAGER"
    fi

    echo ">>> Removing tty1 autologin override..."
    rm -f "$GETTY_OVERRIDE_FILE"
    systemctl daemon-reload

    BASH_PROFILE="$KIOSK_HOME/.bash_profile"
    if [ -f "$BASH_PROFILE" ] && grep -qF "$MARK_BEGIN" "$BASH_PROFILE"; then
        echo ">>> Removing kiosk autostart block from $BASH_PROFILE..."
        sed -i "/$MARK_BEGIN/,/$MARK_END/d" "$BASH_PROFILE"
    fi

    XINITRC="$KIOSK_HOME/.xinitrc"
    if [ -f "$XINITRC" ] && grep -qF "$MARK_BEGIN" "$XINITRC"; then
        if [ -f "$XINITRC.pre-kiosk.bak" ]; then
            echo ">>> Restoring original $XINITRC from backup..."
            mv "$XINITRC.pre-kiosk.bak" "$XINITRC"
        else
            echo ">>> Removing $XINITRC (was created by kiosk mode, no prior version existed)..."
            rm -f "$XINITRC"
        fi
    fi

    echo ">>> Note: TeamViewer is left running as a systemd service (harmless alongside a normal desktop) - not reverted."

    rm -f "$STATE_FILE"

    echo ""
    echo ">>> Restored. Reboot to apply:"
    echo "    sudo reboot"
}

case "${1:-}" in
    enable)  shift; cmd_enable "$@" ;;
    restore) cmd_restore ;;
    status)  cmd_status ;;
    *)
        echo "Usage: sudo $0 {enable [username]|restore|status}" >&2
        exit 1
        ;;
esac
