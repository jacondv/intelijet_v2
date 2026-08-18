# Brief 08 — Xử lý sự cố

**File output:** `docs/manual/08_xu_ly_su_co.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator (bước đầu tự xử lý) + kỹ thuật viên (khi cần can thiệp sâu hơn)
**Mức ưu tiên:** CAO NHẤT — cùng với chương 02, đây là lý do chính người dùng cần manual này (xử lý lỗi khi hệ thống hoạt động không đúng). Viết dạng bảng "Thấy gì → Làm gì", câu trả lời phải là hành động cụ thể operator tự làm được ngay, không phải lời giải thích kỹ thuật (xem văn phong ở `00_INDEX.md` mục 1.4).

## Nguồn thông tin / code cần đọc

- `run_docker.sh` — các cảnh báo đã biết: `$DISPLAY` chưa được set, `xhost +local:docker` thất bại → app không hiện cửa sổ; log của lần chạy gần nhất/gần nhì lưu ở `data/logs/last_run.log`/`previous_run.log`.
- `run_intelijet.sh` — lần chạy đầu tiên sẽ build workspace (`catkin build`) nên **có thể chậm hơn hẳn bình thường** — operator cần biết đây không phải bị treo.
- `intelijet_v2_ws/src/config/devices.yaml` + chương 07 — ánh xạ mỗi thiết bị "disconnected" sang nguyên nhân/cách kiểm tra thực tế (ví dụ Lidar disconnected → kiểm tra cáp mạng/IP; PCAN disconnected → kiểm tra gateway CAN, dây CAN).
- `docs/optimization_plan_overview.md` — các vấn đề đã biết trong hệ thống (tổng hợp thành bảng "vấn đề đã biết" nếu còn tồn tại tại thời điểm viết, kiểm tra qua code/git log xem đã fix hay chưa, không copy nguyên tài liệu kế hoạch coi như hiện trạng chắc chắn còn đúng):
  - UI có thể "đứng hình" tạm thời khi xử lý report (mục 2) — nếu đã fix bằng threading, cập nhật lại mô tả cho đúng thời điểm viết.
  - Icon Desktop: mỗi lần bấm sẽ dừng phiên đang chạy và khởi động sạch — không phải "mở lại", nên nếu operator bấm nhầm giữa chừng, dữ liệu job đang làm dở chưa lưu có thể mất — cảnh báo rõ trong SOP (chương 06) và nhắc lại ở đây.
- `intallation_gui.md` — các lệnh hữu ích liên quan `usb-copier.service`, Syncthing (nếu lỗi liên quan đồng bộ/USB, trỏ sang chương 10/11 thay vì lặp lại ở đây).

## Nội dung cần có (outline) — dạng bảng "Triệu chứng → Nguyên nhân khả dĩ → Cách xử lý → Khi nào cần gọi kỹ thuật viên"

1. App không khởi động / cửa sổ không hiện ra.
2. Khởi động lần đầu rất chậm (bình thường, do build workspace).
3. Thiết bị hiển thị "disconnected" — theo từng loại thiết bị (Lidar/Encoder/PCAN/PLC), tham chiếu chương 07.
4. UI bị "đứng"/không phản hồi trong lúc xử lý report — có phải lỗi hay hành vi biết trước.
5. Không tạo được report / report lỗi.
6. Mất dữ liệu job do bấm icon khởi động lại giữa chừng.
7. Vấn đề đồng bộ dữ liệu giữa 2 tablet (Syncthing báo đỏ, file `.sync-conflict-*`) — trỏ sang chương 10.
8. Cách thu thập thông tin để báo lỗi cho kỹ thuật viên (vị trí file log, cách chụp màn hình lỗi).
9. Khi nào cần dừng vận hành ngay và gọi hỗ trợ thay vì tự xử lý (liên quan an toàn — phối hợp với chương 02).

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Danh sách lỗi/sự cố thường gặp thực tế tại hiện trường (không chỉ suy ra từ code) — nên phỏng vấn operator đã dùng máy để bổ sung các trường hợp thực tế đã gặp, code chỉ cho manh mối kỹ thuật chứ không cho biết lỗi nào hay xảy ra nhất.
- Thông tin liên hệ hỗ trợ kỹ thuật (số điện thoại/email/kênh báo lỗi) để đưa vào mục 8-9.
