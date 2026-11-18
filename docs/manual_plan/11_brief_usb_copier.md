# Brief 11 — USB Data Copier

**File output:** `docs/manual/11_usb_copier.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator (dùng hằng ngày để lấy dữ liệu ra USB) + kỹ thuật viên (cài đặt/service)
**Mức ưu tiên:** Thấp — nội dung nguồn đã gần như đầy đủ. Viết ngắn, chỉ phần "dùng hằng ngày" là quan trọng với operator; phần cài đặt/service dành kỹ thuật viên có thể để dạng liệt kê lệnh, không cần giải thích thêm.

## Nguồn thông tin / code cần đọc

- `intallation_gui.md` phần "USB Data Copier - Installation Guide" — cài đặt, kiểm tra service, các lệnh `systemctl` hữu ích, chạy thủ công không cần service, xử lý khi GUI không hiện (`xhost +local:`).
- `intelijet_usbcopier/` — `install.sh`, `setup.sh`, `uninstall.sh`, `usb_copier.py` — đọc nhanh để xác nhận mô tả trong `intallation_gui.md` còn khớp với code thật.

## Nội dung cần có (outline)

1. Chức năng: cắm USB vào, cửa sổ USB Data Copier tự hiện ra để copy dữ liệu — mô tả rõ copy dữ liệu **gì** (dữ liệu job/report từ đâu tới USB — kiểm tra `usb_copier.py` để biết nguồn/đích thật, đừng đoán).
2. Cách dùng hằng ngày cho operator: cắm USB → cửa sổ tự hiện → chọn gì để copy → rút USB an toàn thế nào.
3. Cài đặt (dành cho kỹ thuật viên): copy thư mục `intelijet_usbcopier` vào home, chạy `install.sh`, kiểm tra `systemctl status usb-copier.service`.
4. Các lệnh quản lý service: start/stop/restart, xem log (`journalctl -u usb-copier.service -f`).
5. Xử lý khi GUI không hiện: `xhost +local:` rồi restart service.
6. Gỡ cài đặt: `uninstall.sh`.

## Đã xác nhận từ code (không cần hỏi người dùng)

- `usb_copier.py` — nguồn dữ liệu là `data/Projects/` (đường dẫn cấu hình cứng trong file, dạng `/home/<user>/intelijet_v2/data/Projects/`); người dùng tự duyệt thư mục con và tick chọn mục muốn copy trong lúc dùng, không phải copy toàn bộ tự động. Viết đúng theo hành vi này.
  > ⚠️ Lưu ý nhỏ: đường dẫn này hard-code theo từng máy (giống lưu ý ở chương 10) — nếu máy thật có đường dẫn khác, kỹ thuật viên cần sửa lại trong `usb_copier.py`, không phải lỗi của operator.
