#!/usr/bin/env bash
# Run this ONCE, right before handing the machine to a customer - wipes
# personal/developer credentials and history so the customer doesn't get
# your Gmail/browser logins, your Claude Code account, your VS Code
# session, or any trace of your work on this machine.
#
# Deliberately NOT touched (kept working for the customer):
#   - Saved WiFi profiles (/etc/NetworkManager/system-connections) - the
#     app needs WiFi to talk to the LIDAR, and the customer may be on the
#     same network you tested with.
#   - The `origin` git remote in intelijet_v2/.git/config - kept so you
#     can still `git pull` updates to this machine later. No push
#     credentials exist on this machine anyway (checked: no SSH keys, no
#     .git-credentials, no `gh` CLI, no Docker registry login) - only the
#     repo-local author identity (name/email) is cleared.
#   - Linux user password for 'nuc' - left as-is.
#   - Docker images/containers - the app needs them to run.
#
# Usage: bash clean.sh   (no sudo needed - everything here is user-owned)
set -uo pipefail

echo "=== Don dep du lieu ca nhan truoc khi ban giao may ==="
echo ""

# --- Claude Code: revoke login + wipe all project/session history -----
if [ -e "$HOME/.claude" ] || [ -e "$HOME/.claude.json" ]; then
    echo ">>> Xoa Claude Code (token dang nhap + toan bo lich su session/project)..."
    rm -rf "$HOME/.claude" "$HOME/.claude.json"
else
    echo ">>> Claude Code: khong co gi de xoa."
fi

# --- GNOME Keyring: where Firefox/VS Code/system saved passwords live -
if [ -d "$HOME/.local/share/keyrings" ]; then
    echo ">>> Xoa GNOME keyring (kho luu mat khau da luu cua Firefox/VS Code/he thong)..."
    rm -f "$HOME/.local/share/keyrings"/*.keyring "$HOME/.local/share/keyrings"/*.keystore
else
    echo ">>> GNOME keyring: khong co gi de xoa."
fi

# --- Firefox: wipe the whole profile (logins, cookies, history) -------
# Covers both the .deb install (~/.mozilla) and the snap install
# (~/snap/firefox/common/.mozilla) - harmless if one doesn't exist.
FOUND_FIREFOX=0
for MOZ_DIR in "$HOME/.mozilla/firefox" "$HOME/snap/firefox/common/.mozilla/firefox"; do
    if [ -d "$MOZ_DIR" ]; then
        FOUND_FIREFOX=1
        echo ">>> Xoa Firefox profile ($MOZ_DIR) - dang nhap Gmail, cookie, lich su..."
        rm -rf "$MOZ_DIR"/*.default* "$MOZ_DIR"/profiles.ini
    fi
done
[ "$FOUND_FIREFOX" = 1 ] || echo ">>> Firefox: chua tung tao profile nao, khong co gi de xoa."

# --- Chrome/Chromium, if ever installed --------------------------------
for DIR in "$HOME/.config/google-chrome" "$HOME/.config/chromium" "$HOME/snap/chromium/common/.config/chromium"; do
    if [ -d "$DIR" ]; then
        echo ">>> Xoa Chrome/Chromium profile ($DIR)..."
        rm -rf "$DIR"
    fi
done

# --- GNOME Online Accounts (Google/Gmail linked at OS level, if any) --
if [ -f "$HOME/.config/goa-1.0/accounts.conf" ]; then
    echo ">>> Xoa GNOME Online Accounts (tai khoan Google lien ket o muc he thong)..."
    rm -f "$HOME/.config/goa-1.0/accounts.conf"
fi

# --- VS Code: wipe session/auth state, keep the editor+extensions -----
if [ -d "$HOME/.config/Code/User" ]; then
    echo ">>> Xoa VS Code session/auth state (globalStorage, workspaceStorage, file gan day)..."
    rm -rf "$HOME/.config/Code/User/globalStorage" \
           "$HOME/.config/Code/User/workspaceStorage" \
           "$HOME/.config/Code/User/History"
    rm -f "$HOME/.config/Code/User/storage.json"
fi

# --- Git: clear YOUR author identity from this repo, keep the remote --
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -d "$REPO_DIR/.git" ]; then
    echo ">>> Xoa git identity (name/email) khoi repo - remote 'origin' giu nguyen..."
    git -C "$REPO_DIR" config --unset user.name 2>/dev/null || true
    git -C "$REPO_DIR" config --unset user.email 2>/dev/null || true
fi

# --- Shell history + recently-used files list --------------------------
echo ">>> Xoa lich su lenh terminal va danh sach file mo gan day..."
: > "$HOME/.bash_history"
history -c 2>/dev/null || true
rm -f "$HOME/.local/share/recently-used.xbel"

# --- Trash ---------------------------------------------------------------
if [ -d "$HOME/.local/share/Trash" ]; then
    echo ">>> Don thung rac..."
    rm -rf "$HOME/.local/share/Trash"/*
fi

echo ""
echo "=== Xong. Kiem tra lai truoc khi giao may: ==="
echo "  - Mo Firefox thu, xac nhan khong con dang nhap Gmail nao."
echo "  - Mo app (icon Intelijet / kiosk mode) thu, xac nhan van chay binh thuong."
echo "  - Neu ban dang dung Claude Code de chay script nay, phien lam viec nay"
echo "    se bi ngat ngay khi file .claude bi xoa - do la binh thuong."
