#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="${1:-$SCRIPT_DIR/intelijet_v2_ws/src}"

if [ ! -d "$TARGET_DIR" ]; then
    echo "Directory not found: $TARGET_DIR"
    exit 1
fi

echo "Scanning package directories in: $TARGET_DIR"

find "$TARGET_DIR" -mindepth 1 -maxdepth 1 -type d | sort | while read -r pkg_dir; do
    pkg_name="$(basename "$pkg_dir")"
    echo "[Package] $pkg_name"

    if [ -d "$pkg_dir/scripts" ]; then
        find "$pkg_dir/scripts" -type f -name "*.py" | sort | while read -r file; do
            chmod +x "$file"
            echo "  +x $file"
        done
    fi
done

echo "Done."