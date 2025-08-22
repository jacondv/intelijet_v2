#!/bin/bash

# X11: tắt màn hình
if command -v xset &> /dev/null; then
    xset dpms force off
else
    # Wayland GNOME: lock màn hình (Wayland không cho tắt hoàn toàn)
    loginctl lock-session
fi
