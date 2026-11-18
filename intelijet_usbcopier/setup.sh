#!/bin/bash
# ============================================================
# USB Data Copier - Cài đặt và chạy
# ============================================================

echo "╔══════════════════════════════════════╗"
echo "║     USB DATA COPIER - SETUP          ║"
echo "╚══════════════════════════════════════╝"

# Kiểm tra Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 chưa được cài. Chạy: sudo apt install python3"
    exit 1
fi

# Kiểm tra tkinter
python3 -c "import tkinter" 2>/dev/null || {
    echo "⚙ Đang cài tkinter..."
    sudo apt-get install -y python3-tk
}

echo ""
echo "✅ Sẵn sàng!"
echo ""
echo "🚀 Cách chạy:"
echo "   python3 usb_copier.py                    # dùng đường dẫn mặc định"
echo "   python3 usb_copier.py /root/intelijet_v2/data/Projects    # chỉ định đường dẫn nguồn"
echo ""
echo "📌 Hoặc cấu hình đường dẫn mặc định trong file usb_copier.py dòng:"
echo '   SOURCE_DATA_PATH = "/root/intelijet_v2/data/Projects"'
echo ""
