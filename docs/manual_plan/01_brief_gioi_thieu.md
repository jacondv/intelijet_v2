# Brief 01 — Giới thiệu hệ thống

**File output:** `docs/manual/01_gioi_thieu.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator + kỹ thuật viên (người mới, chưa từng thấy máy)
**Mức ưu tiên:** Thấp — viết THẬT NGẮN (nửa trang), đây không phải trọng tâm của manual (trọng tâm là an toàn + xử lý sự cố, xem `00_INDEX.md` mục 0 và 3). Chỉ cần đủ để người mới hiểu máy dùng để làm gì trước khi qua chương an toàn.

## Mục tiêu chương

Cho người đọc bức tranh tổng quát trước khi đi vào chi tiết vận hành: hệ thống dùng để làm gì, gồm những gì (phần cứng + phần mềm), chạy trên nền tảng nào.

## Nguồn thông tin / code cần đọc

- `docs/optimization_plan_overview.md` mục 0 — mô tả tổng quan hiện trạng (ROS1 Noetic + PyQt5 + Open3D/VTK + WeasyPrint, chạy trong Docker, khởi động qua icon Desktop).
- `intelijet_v2_ws/src/` — danh sách package: `pps` (xử lý quét/point cloud), `ui` (giao diện), `shared` (thư viện dùng chung, device monitor, message types), `encoder_process`, `pcan_ethernet_gateway`, `ui_can_interface` (giao tiếp CAN/PLC/encoder), `sick_scan` (driver lidar SICK), `ai_core_pkg` (image matcher cho compare), `config` (file cấu hình).
- `intelijet_v2_ws/src/pps/urdf/scanner_housing.urdf` — có mô hình housing (cấu trúc cơ khí quét).
- `intelijet_v2_ws/src/config/devices.yaml` — danh sách thiết bị vật lý hệ thống theo dõi: lidar (SICK, qua ping IP), encoder (CAN), pcan (CAN gateway), plc (CAN), pps (tiến trình nội bộ).
- `docker-compose.yml` — tên image, cách container chạy.

## Nội dung cần có (outline)

1. Intelijet là gì — mục đích sử dụng (quét/so sánh point cloud, phát hiện thay đổi/hao mòn — xác nhận đúng use case với người dùng, xem câu hỏi bên dưới).
2. Thành phần phần cứng chính: housing (vỏ mở/đóng chứa đầu quét), **cánh tay thủy lực (2 khớp + 1 xy-lanh) đưa Housing tới gần vị trí cần quét** (do người dùng cung cấp, không có trong code — xem `00_INDEX.md`), đầu quét SICK LMS511, encoder, PLC, gateway CAN (PCAN-Ethernet), máy tính/tablet chạy phần mềm, màn hình cảm ứng.
3. Thành phần phần mềm: ứng dụng chạy trong Docker container (ROS1 Noetic + giao diện PyQt5 kiosk), không cần cài ROS/Python trực tiếp lên máy — chỉ cần Docker.
4. Luồng làm việc tổng quát ở mức rất cao (chi tiết đầy đủ để ở chương 06): chọn/tạo job → quét trước (pre-scan) → quét sau (post-scan) → so sánh → xuất report PDF.
5. Sơ đồ kiến trúc đơn giản (ảnh/khối) — placeholder, có thể tham khảo `frames.pdf`/`frames.gv` (sơ đồ TF kỹ thuật) nhưng cần vẽ lại đơn giản hơn cho người không rành ROS.

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Mục đích sử dụng chính xác của hệ thống (loại kết cấu/bề mặt được quét, ngành ứng dụng) — code chỉ cho thấy cơ chế kỹ thuật (point cloud compare), không nói rõ ứng dụng thực tế (ví dụ: kiểm tra hao mòn đường ống/tunnel?). Xem thêm `intelijet_v2_ws/src/pps/src/pps/tunnel_processing.py` — tên file gợi ý ứng dụng liên quan tunnel/đường ống, cần xác nhận.
- Tên gọi chính thức của các bộ phận phần cứng (ví dụ "housing" có tên tiếng Việt/tên thương mại nào hay dùng tại hiện trường không) để dùng nhất quán trong toàn manual.
