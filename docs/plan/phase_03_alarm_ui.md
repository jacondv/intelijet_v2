# Phase 3 — Hệ thống cảnh báo/alarm chuyên nghiệp trên UI

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P2 (device monitor đã chuẩn hoá). Rủi ro: vừa.
> Phạm vi file: `ui/scripts/app.py` (phần hiển thị trạng thái + notification), `ui/scripts/ros_thread.py`, `shared/src/shared/log_status.py`, file mới trong `ui/src/ui/`. KHÔNG đụng pipeline point cloud/report (P4/P5 lo).

## Hiện trạng (đã khảo sát)

- Trạng thái 4 thiết bị hiển thị bằng text thường trên 4 label: `lblEncoderStatus`, `lblLidarStatus`, `lblPCANStatus`, `lblPLCStatus` (`app.py::update_data`, ~dòng 465-500; dữ liệu do `ros_thread.py` poll 1 Hz từ `StatusReader`). Không màu, không icon, thiếu key thì hiện chữ `"UNKNOWN"`.
- Thông báo vận hành: các node pps gọi `shared.log_status.log_status()` → JSON qua `/rosout` → `ros_thread.py` `unpack_log_status()` → đổ vào **một label duy nhất** `lblNotification`, bị ghi đè liên tục bởi mọi nguồn (status tick 1 Hz, progress compare, lỗi...). Mức độ nghiêm trọng hiện suy ra bằng cách dò chuỗi `"[WARN]"`/`"[ERROR]"` trong text (log_status.py:18-21) — fragile.
- Không có lịch sử cảnh báo; lỗi quan trọng biến mất trước khi người vận hành kịp đọc.

## Thiết kế đích

### 1. Chuẩn hoá severity ở nguồn (`shared/log_status.py`)
- Thêm tham số tường minh `level` cho `log_status(name, message, level="info")` với giá trị `"info" | "warning" | "error"`; đưa `level` vào JSON payload.
- **Giữ tương thích ngược**: nếu caller không truyền `level`, giữ nguyên cách suy từ `"[WARN]"`/`"[ERROR]"` trong message (không sửa hàng loạt caller trong phase này — sẽ chuẩn hoá dần khi đụng từng file ở phase khác).
- `unpack_log_status()` trả thêm field `level` (mặc định `"info"` nếu payload cũ không có).

### 2. Model cảnh báo phía UI — file mới `ui/src/ui/notification_center.py`
Class `NotificationCenter` (thuần Python + Qt signal, KHÔNG phụ thuộc ROS — nhận dữ liệu từ ngoài đẩy vào, để test được độc lập):
- Giữ danh sách tối đa N thông báo gần nhất (mặc định 50, hằng số đặt đầu file): mỗi item `(timestamp, level, source, message)`.
- Signal `notification_added(item)` để UI cập nhật.
- Quy tắc chống spam: thông báo trùng (cùng source + message) trong vòng X giây (mặc định 5s) không thêm bản ghi mới — chỉ cập nhật timestamp bản cũ.
- Trạng thái thiết bị đổi (CONNECTED ↔ DISCONNECTED) cũng sinh 1 notification (`level=error` khi mất kết nối, `info` khi kết nối lại) — nguồn cấp là `update_data` trong app.py.

### 3. Hiển thị
a. **Label trạng thái thiết bị**: viết 1 hàm dùng chung `set_device_label(label, state)` (đặt trong file mới hoặc `notification_center.py`) — set text + màu qua stylesheet: CONNECTED → nền/chữ xanh lá, DISCONNECTED → đỏ, UNKNOWN → xám. Thay 4 đoạn if/else lặp trong `update_data` bằng vòng lặp qua mapping `{tên device: label}`.
b. **`lblNotification`**: chỉ hiển thị thông báo **mới nhất**, kèm màu theo level (info: mặc định, warning: vàng/cam, error: đỏ). Thông báo `error` được "ghim" tối thiểu 10 giây — thông báo `info` đến sau không được ghi đè trong thời gian ghim (warning/error mới hơn thì được).
c. **Lịch sử cảnh báo**: thêm 1 dialog đơn giản (file mới `ui/src/ui/notification_history_dialog.py`, dùng `QListWidget`, mỗi dòng `HH:MM:SS [LEVEL] message`, màu theo level) mở khi **bấm/chạm vào `lblNotification`**. Không sửa file `.ui` generated — gắn event bằng code (eventFilter hoặc `mousePressEvent` trên label). Dialog phải đóng được dễ dàng bằng nút to (màn hình cảm ứng).

### 4. Nối dây trong `app.py` / `ros_thread.py`
- `ros_thread.py`: khi unpack log status → emit signal (đã có sẵn cơ chế signal sang GUI thread — dùng lại, không tạo cơ chế mới) kèm `level`.
- `app.py`: tạo `NotificationCenter` instance; các nguồn (log status, device state change, compare progress/done) đẩy vào đó thay vì set thẳng `lblNotification`.
- Progress compare (`on_compare_process`) KHÔNG đưa vào lịch sử (spam) — vẫn hiển thị trực tiếp như cũ hoặc qua nhánh riêng "transient" của NotificationCenter (không lưu history).

## Ràng buộc

- Không sửa file `*_ui.py` generated và file `.ui` Qt Designer trong phase này (dialog lịch sử tạo bằng code thuần).
- Không đổi format JSON của log_status theo cách phá vỡ payload cũ (chỉ THÊM field).
- Mọi cập nhật widget phải diễn ra trên GUI thread (qua signal/slot như hiện tại).

## Kiểm chứng

1. `py_compile` mọi file sửa/mới.
2. Test tay `NotificationCenter` không cần ROS: script nhỏ đẩy 100 thông báo → xác nhận giới hạn 50, chống trùng, ghim error. (Viết test này thành file `ui/src/ui/tests/test_notification_center.py` chạy bằng `python3 -m pytest` hoặc `python3 <file>` trực tiếp nếu không có pytest.)
3. Nếu có môi trường chạy app: mở app, rút kết nối 1 thiết bị → label đổi đỏ + 1 dòng vào lịch sử; bấm label mở dialog lịch sử.

## Tiêu chí nghiệm thu

- [ ] Label thiết bị có màu theo trạng thái; code hiển thị gộp thành 1 vòng lặp.
- [ ] `log_status` có `level` tường minh, tương thích ngược.
- [ ] `NotificationCenter` + dialog lịch sử hoạt động; error được ghim, không bị info ghi đè ngay.
- [ ] Test đơn giản cho NotificationCenter pass.
- [ ] Commit `[P3] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

_(chưa có)_

## Ghi chú phát sinh

_(chưa có)_
