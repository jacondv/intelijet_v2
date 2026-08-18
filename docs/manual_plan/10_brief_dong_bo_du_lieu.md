# Brief 10 — Đồng bộ dữ liệu giữa 2 tablet (Syncthing)

**File output:** `docs/manual/10_dong_bo_du_lieu.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Kỹ thuật viên (thiết lập lần đầu) + operator (kiểm tra trạng thái hằng ngày)
**Mức ưu tiên:** Thấp — nội dung nguồn đã khá đầy đủ, chủ yếu biên soạn lại + tách phần nào operator cần biết vs phần nào chỉ kỹ thuật viên cần.

## Nguồn thông tin / code cần đọc

- `intallation_gui.md` phần "Folder Sync Setup with Syncthing (Tablet A ↔ Tablet B)" — toàn bộ quy trình cài đặt, ghép cặp thiết bị, thêm folder đồng bộ (`scanner_config`, `scanner_results`), versioning, firewall.

## Nội dung cần có (outline)

Tách làm 2 phần rõ ràng:

### Phần A — Dành cho kỹ thuật viên (thiết lập/khắc phục sâu)
1. Cài đặt Syncthing trên cả 2 tablet, chạy như systemd service.
2. Mở Web GUI (`127.0.0.1:8384` local, hoặc `0.0.0.0:8384` nếu cần truy cập từ xa — **nhắc lại cảnh báo bảo mật**: phải đặt mật khẩu ngay khi đổi sang `0.0.0.0`).
3. Ghép cặp 2 tablet bằng Device ID.
4. Thêm 2 folder đồng bộ: `scanner_config` (versioned, rescan nhanh ~10s) và `scanner_results` (rescan mặc định ~60s, versioning tùy chọn).
5. Yêu cầu firewall: TCP/UDP 22000 (sync), UDP 21027 (discovery LAN), TCP 8384 (web GUI, chỉ LAN).

### Phần B — Dành cho operator (kiểm tra hằng ngày)
1. Cách xem nhanh trạng thái đồng bộ đang "Up to Date" (xanh)/"Syncing" (xanh dương)/lỗi (đỏ) — không cần hiểu sâu, chỉ cần biết khi nào cần báo kỹ thuật viên.
2. Ý nghĩa file `*.sync-conflict-<ngày>-<thiết bị>.ext` xuất hiện: 2 tablet cùng sửa 1 file gần như đồng thời, Syncthing **không tự ghi đè**, tạo file conflict để không mất dữ liệu — cần xem lại thủ công (không tự xóa file conflict mà không kiểm tra).

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Đường dẫn thư mục trong ví dụ (`/home/jacon/...`) có phải đường dẫn chuẩn trên mọi máy hay chỉ là ví dụ của 1 máy cụ thể — nếu khác nhau giữa các máy, cần ghi chú rõ "đường dẫn thực tế xem trong Syncthing Web GUI của máy đó" thay vì hard-code một đường dẫn có thể sai với máy khác.
