#!/usr/bin/env bash
# One-shot, no-questions-asked fix: force this machine back to a normal
# GUI desktop (GDM/LightDM login screen), undoing every kiosk-mode change
# regardless of what state it's currently in. Safe to run even if kiosk
# mode was never enabled, or was only partially undone.
#
# Usage: sudo bash force_restore_gui.sh
set -uo pipefail

echo ">>> Unmasking getty@tty1 (undoes 'systemctl mask getty@tty1' if it was run)..."
systemctl unmask getty@tty1 2>/dev/null || true

echo ">>> Ensuring boot target is graphical (not text-only multi-user.target)..."
systemctl set-default graphical.target 2>/dev/null || true

echo ">>> Enabling and starting a display manager now..."
for dm in gdm3 gdm lightdm sddm; do
    if systemctl enable -f "$dm" 2>/dev/null; then
        systemctl start "$dm"
        DM_FOUND=1
        break
    fi
done
[ -n "${DM_FOUND:-}" ] || echo "    WARNING: no known display manager (gdm3/gdm/lightdm/sddm) found." >&2

echo ">>> Removing tty1 autologin override..."
rm -f /etc/systemd/system/getty@tty1.service.d/override.conf
systemctl daemon-reload

echo ">>> Removing kiosk autostart block from ~/.bash_profile..."
sed -i '/# >>> intelijet-kiosk-mode >>>/,/# <<< intelijet-kiosk-mode <<</d' "$HOME/.bash_profile" 2>/dev/null || true

echo ">>> Removing ~/.xinitrc (kiosk-mode X session script)..."
rm -f "$HOME/.xinitrc"

echo ">>> Removing kiosk state file..."
rm -f /etc/intelijet-kiosk.conf

echo ""
echo ">>> Done. Rebooting in 5 seconds (Ctrl+C to cancel)..."
sleep 5
reboot
