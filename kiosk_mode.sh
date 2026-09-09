#!/usr/bin/env bash
# Toggle this machine between normal desktop (GDM/LightDM login screen,
# click the Intelijet icon manually) and kiosk mode (boot straight to tty1
# autologin -> minimal X (openbox) -> run_docker.sh, no login screen).
#
# Usage (needs root):
#   sudo ./kiosk_mode.sh                     # no args: interactive menu
#   sudo ./kiosk_mode.sh enable [username]   # username defaults to $SUDO_USER
#   sudo ./kiosk_mode.sh restore [username]  # username only needed if the
#                                             # state file from enable is gone
#   sudo ./kiosk_mode.sh status
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_FILE="/etc/intelijet-kiosk.conf"
MARK_BEGIN="# >>> intelijet-kiosk-mode >>>"
MARK_END="# <<< intelijet-kiosk-mode <<<"
GETTY_OVERRIDE_FILE="/etc/systemd/system/getty@tty1.service.d/override.conf"
KNOWN_DMS="gdm3 gdm lightdm sddm"

require_root() {
    [ "$(id -u)" = "0" ] || { echo "Run with sudo: sudo $0 $*" >&2; exit 1; }
}

# Display manager actually in use (symlink), or empty if none configured.
current_dm() {
    [ -L /etc/systemd/system/display-manager.service ] &&
        basename "$(readlink -f /etc/systemd/system/display-manager.service)" .service
}

# Any known display manager installed on this machine, symlinked or not -
# fallback for when current_dm finds nothing (e.g. it was left disabled by
# an earlier failed kiosk toggle).
any_installed_dm() {
    for dm in $KNOWN_DMS; do
        if [ -f "/lib/systemd/system/$dm.service" ] || [ -f "/usr/lib/systemd/system/$dm.service" ]; then
            echo "$dm"; return
        fi
    done
}

cmd_status() {
    echo "Kiosk mode: $([ -f "$STATE_FILE" ] && echo ENABLED || echo "not enabled (normal desktop)")"
    echo "Display manager: $(current_dm || echo none)"
    echo "getty@tty1 autologin override: $([ -f "$GETTY_OVERRIDE_FILE" ] && echo present || echo absent)"
    echo "getty@tty1 masked: $(systemctl is-enabled getty@tty1 2>/dev/null | grep -q masked && echo yes || echo no)"
}

cmd_enable() {
    require_root
    [ -f "$STATE_FILE" ] && { echo "Already enabled (see: $0 status). Run 'restore' first."; exit 1; }

    KIOSK_USER="${1:-${SUDO_USER:-}}"
    [ -n "$KIOSK_USER" ] || { echo "Usage: sudo $0 enable <username>" >&2; exit 1; }
    KIOSK_HOME="$(getent passwd "$KIOSK_USER" | cut -d: -f6)"
    [ -n "$KIOSK_HOME" ] && [ -d "$KIOSK_HOME" ] || { echo "User '$KIOSK_USER' has no home dir." >&2; exit 1; }

    DM="$(current_dm)"
    echo ">>> Display manager: ${DM:-none}. Saving state to $STATE_FILE..."
    cat > "$STATE_FILE" <<EOF
KIOSK_USER=$KIOSK_USER
KIOSK_HOME=$KIOSK_HOME
DISPLAY_MANAGER=$DM
REPO_DIR=$REPO_DIR
EOF

    if [ -n "$DM" ]; then
        echo ">>> Disabling $DM (takes effect next boot)..."
        systemctl disable "$DM"
    fi

    echo ">>> Ensuring openbox is installed..."
    dpkg -s openbox >/dev/null 2>&1 || { apt-get update && apt-get install -y openbox; }

    echo ">>> tty1 autologin for '$KIOSK_USER'..."
    mkdir -p "$(dirname "$GETTY_OVERRIDE_FILE")"
    cat > "$GETTY_OVERRIDE_FILE" <<EOF
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $KIOSK_USER --noclear %I \$TERM
EOF
    systemctl daemon-reload

    BASH_PROFILE="$KIOSK_HOME/.bash_profile"
    if [ -f "$BASH_PROFILE" ] && grep -qF "$MARK_BEGIN" "$BASH_PROFILE"; then
        echo ">>> $BASH_PROFILE already has the kiosk block, leaving it."
    else
        [ -f "$BASH_PROFILE" ] && cp "$BASH_PROFILE" "$BASH_PROFILE.pre-kiosk.bak"
        {
            echo "$MARK_BEGIN"
            echo 'if [ -z "$DISPLAY" ] && [ "$(tty)" = "/dev/tty1" ]; then exec startx; fi'
            echo "$MARK_END"
        } >> "$BASH_PROFILE"
        chown "$KIOSK_USER":"$KIOSK_USER" "$BASH_PROFILE"
    fi

    XINITRC="$KIOSK_HOME/.xinitrc"
    [ -f "$XINITRC" ] && ! grep -qF "$MARK_BEGIN" "$XINITRC" && cp "$XINITRC" "$XINITRC.pre-kiosk.bak"
    cat > "$XINITRC" <<EOF
#!/bin/bash
$MARK_BEGIN
openbox &
xset s off; xset -dpms; xset s noblank
"$REPO_DIR/run_docker.sh"
# run_docker.sh returns in a few seconds (docker compose up -d is detached)
# - without something after it, xinit tears down X the moment it returns,
# which restarts the whole autologin chain: an infinite boot loop.
exec tail -f /dev/null
$MARK_END
EOF
    chmod +x "$XINITRC"
    chown "$KIOSK_USER":"$KIOSK_USER" "$XINITRC"

    command -v teamviewer >/dev/null 2>&1 && teamviewer daemon enable || true

    echo ""
    echo ">>> Kiosk mode configured. Reboot to apply: sudo reboot"
    echo ">>> To undo: sudo $0 restore"
}

cmd_restore() {
    require_root
    if [ -f "$STATE_FILE" ]; then
        # shellcheck disable=SC1090
        source "$STATE_FILE"
    else
        echo ">>> No state file (already restored, or lost) - best-effort restore."
        KIOSK_USER="${1:-${SUDO_USER:-}}"
        [ -n "$KIOSK_USER" ] || read -rp "Username to clean up: " KIOSK_USER
        KIOSK_HOME="$(getent passwd "$KIOSK_USER" 2>/dev/null | cut -d: -f6)"
        DISPLAY_MANAGER=""
    fi

    # A manual `systemctl mask getty@tty1` (used to stop a boot loop) is a
    # stronger, separate state from `disable` that survives `enable` and
    # blocks tty1/GDM from starting until explicitly unmasked.
    echo ">>> Unmasking getty@tty1..."
    systemctl unmask getty@tty1 2>/dev/null

    DM="${DISPLAY_MANAGER:-$(current_dm)}"
    DM="${DM:-$(any_installed_dm)}"
    if [ -n "$DM" ]; then
        echo ">>> Enabling and starting $DM now (not just on next boot)..."
        systemctl enable -f "$DM"
        systemctl start "$DM"
    else
        echo ">>> WARNING: no display manager found to enable." >&2
    fi

    echo ">>> Removing tty1 autologin override..."
    rm -f "$GETTY_OVERRIDE_FILE"
    systemctl daemon-reload

    if [ -n "${KIOSK_HOME:-}" ] && [ -d "$KIOSK_HOME" ]; then
        BASH_PROFILE="$KIOSK_HOME/.bash_profile"
        [ -f "$BASH_PROFILE" ] && sed -i "/$MARK_BEGIN/,/$MARK_END/d" "$BASH_PROFILE"

        XINITRC="$KIOSK_HOME/.xinitrc"
        if [ -f "$XINITRC" ] && grep -qF "$MARK_BEGIN" "$XINITRC"; then
            if [ -f "$XINITRC.pre-kiosk.bak" ]; then
                mv "$XINITRC.pre-kiosk.bak" "$XINITRC"
            else
                rm -f "$XINITRC"
            fi
        fi
    else
        echo ">>> WARNING: unknown user home - .bash_profile/.xinitrc not cleaned up." >&2
    fi

    rm -f "$STATE_FILE"
    echo ""
    echo ">>> Restored. GDM should be active now; reboot if it isn't: sudo reboot"
}

cmd_menu() {
    require_root
    cmd_status
    echo ""
    echo "1) Enable kiosk mode  2) Restore normal desktop  3) Status  4) Exit"
    read -rp "Choose [1-4]: " choice
    case "$choice" in
        1) read -rp "Username to autologin as [${SUDO_USER:-$USER}]: " u
           cmd_enable "${u:-${SUDO_USER:-$USER}}" ;;
        2) cmd_restore ;;
        3) cmd_status ;;
        4) exit 0 ;;
        *) echo "Invalid choice." >&2; exit 1 ;;
    esac
}

case "${1:-}" in
    enable)  shift; cmd_enable "$@" ;;
    restore) shift; cmd_restore "$@" ;;
    status)  cmd_status ;;
    "")      cmd_menu ;;
    *) echo "Usage: sudo $0 [enable [username]|restore [username]|status]" >&2; exit 1 ;;
esac
