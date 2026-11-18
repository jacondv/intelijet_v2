#!/bin/bash

# ==========================
# CẤU HÌNH ĐƯỜNG DẪN
# ==========================

BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
LEICA_WS="$BASE_DIR/leica_blkarc"
GRPC_INTERFACE="$LEICA_WS/src/blk_grpc_interface"
TARGET_WS="$BASE_DIR/intelijet_v2_ws"

# ==========================
# LINK python_sample_wrapper
# ==========================

PY_WRAPPER_SRC="$LEICA_WS/src/python_sample_wrapper"
PY_WRAPPER_DST="$TARGET_WS/src/python_sample_wrapper"

if [ -L "$PY_WRAPPER_DST" ]; then
    echo "  Removing old symlink: $PY_WRAPPER_DST"
    rm "$PY_WRAPPER_DST"
elif [ -d "$PY_WRAPPER_DST" ]; then
    echo "  [WARNING] $PY_WRAPPER_DST đã tồn tại là thư mục thật. Bỏ qua."
    PY_WRAPPER_DST=""
fi

if [ -n "$PY_WRAPPER_DST" ]; then
    echo "  Linking python_sample_wrapper..."
    ln -s "$PY_WRAPPER_SRC" "$PY_WRAPPER_DST"
fi

# ==========================
# LINK grpc interface
# ==========================

GRPC_DST="$TARGET_WS/src/blk_grpc_interface"  # THÊM DÒNG NÀY

if [ -L "$GRPC_DST" ]; then
    echo "  Removing old symlink: $GRPC_DST"
    rm "$GRPC_DST"
elif [ -d "$GRPC_DST" ]; then
    echo "  [WARNING] $GRPC_DST đã tồn tại là thư mục thật. Bỏ qua."
    GRPC_DST=""
fi

if [ -n "$GRPC_DST" ]; then
    echo "  Linking blk_grpc_interface..."
    ln -s "$GRPC_INTERFACE" "$GRPC_DST"
fi
