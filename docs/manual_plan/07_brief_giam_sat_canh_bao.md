# Brief 07 — Giám sát thiết bị & hệ thống cảnh báo

**File output:** `docs/manual/07_giam_sat_canh_bao.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator
**Mức ưu tiên:** CAO NHẤT (cùng nhóm với chương 08) — nội dung có sẵn khá đầy đủ và rõ ràng trong code, rủi ro "bịa" thấp hơn chương 02. Đây là chương nền cho chương 08 (Xử lý sự cố): operator cần biết "đèn đỏ ở đây nghĩa là gì" trước khi tra được "phải làm gì". Viết dưới dạng bảng đèn/màu → ý nghĩa, không giải thích cơ chế giám sát bên trong (ping, CAN timeout...) — xem văn phong ở `00_INDEX.md` mục 1.4.

## Nguồn thông tin / code cần đọc

- `intelijet_v2_ws/src/config/devices.yaml` — danh sách đầy đủ thiết bị được giám sát và tiêu chí "connected": `lidar` (ping IP `192.168.82.121`, timeout 3s), `encoder` (còn nhận CAN frame trên topic `/encoder01/can_msg` trong 3s gần nhất), `pcan` (tương tự, topic `/pcan_received_messanges`), `pps` (tiến trình nội bộ, `StateEchoMonitor`), `plc` (CAN heartbeat, topic `/plc/heatbeat`).
- `intelijet_v2_ws/src/ui/src/ui/notification_center.py` — cơ chế hiển thị: 3 mức `info`/`warning`/`error` với màu tương ứng (xanh lá/cam/đỏ), quy tắc "pin" (cảnh báo lỗi giữ nguyên trên nhãn 10 giây, không bị info thường ghi đè ngay), lịch sử tối đa 50 mục, khử trùng lặp trong 5 giây.
- `intelijet_v2_ws/src/ui/src/ui/notification_history_dialog.py` — màn hình xem lại lịch sử cảnh báo.
- `docs/optimization_plan_overview.md` mục 3 — bối cảnh: đây là hệ thống **mới được làm lại** (Phase 3/10 theo `docs/plan/`), thay cho cách cũ (label đơn lẻ bị ghi đè) — nếu tại thời điểm viết, phần refactor này đã merge (kiểm tra qua code thật, không qua tài liệu kế hoạch), mô tả theo hành vi mới.
- `shared/msg/DeviceStatus.msg`, `shared/msg/Notification.msg` — cấu trúc dữ liệu trạng thái/thông báo (tham khảo nhanh, không cần giải thích kỹ thuật ROS cho operator).

## Nội dung cần có (outline)

1. Khu vực hiển thị trạng thái thiết bị trên UI: từng thiết bị (Lidar, Encoder, CAN/PCAN, PLC) hiển thị ở đâu, màu gì nghĩa là gì (connected = xanh, disconnected/lỗi = đỏ, không rõ = xám).
2. Ý nghĩa "connected" cho **từng loại thiết bị** — quan trọng, không dùng chung 1 định nghĩa mơ hồ:
   - Lidar: ping mạng thành công tới đầu quét — **không phải** đang có dữ liệu quét chảy về (khác biệt quan trọng, dễ gây hiểu nhầm nếu operator quen kiểu cũ — xem `docs/optimization_plan_overview.md` mục 3 để hiểu lịch sử thay đổi này).
   - Encoder/PCAN/PLC: còn nhận được gói tin CAN trong vài giây gần nhất.
3. Panel/lịch sử cảnh báo: cách mở xem lại các cảnh báo gần đây, phân biệt cảnh báo quan trọng (error, được giữ hiển thị ít nhất 10 giây) với thông báo thường (info).
4. Khi nào một cảnh báo cần operator hành động ngay vs chỉ cần biết thông tin — nối sang chương 08 (Xử lý sự cố) cho từng loại cảnh báo cụ thể.

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Xác nhận lại các giá trị cấu hình trong `devices.yaml` (IP lidar, timeout) khớp với máy thật đang bàn giao — có thể khác giữa các máy nếu cấu hình theo IP nội bộ riêng từng địa điểm.
