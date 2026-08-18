# Brief 12 — Phụ lục

**File output:** `docs/manual/12_phu_luc.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator + kỹ thuật viên (tra cứu)
**Mức ưu tiên:** Thấp — phần thông số kỹ thuật cần xác nhận nhiều, phần còn lại có thể tổng hợp từ các chương trước. Đây là phần tra cứu, không phải phần đọc tuần tự — có thể kỹ thuật hơn một chút so với các chương khác (ví dụ bảng IP/topic dành riêng cho kỹ thuật viên), nhưng vẫn không đưa tên file code/hàm vào.

## Nguồn thông tin / code cần đọc

- `frames.pdf` / `frames.gv` — sơ đồ TF (frame) kỹ thuật ROS, gốc repo. Có thể dùng làm sơ đồ tham khảo cấu trúc tọa độ/lắp đặt nếu phù hợp, nhưng đây là sơ đồ kỹ thuật cho dev/ROS, **cần đơn giản hóa hoặc chú thích lại** nếu đưa vào manual cho operator — không copy nguyên vẹn không giải thích.
- `intelijet_v2_ws/src/pps/urdf/scanner_housing.urdf` — mô hình hình học housing, có thể tham khảo cho sơ đồ nhưng là file kỹ thuật, cần người biết đọc URDF hỗ trợ diễn giải nếu muốn trích thông số từ đây.
- `intelijet_v2_ws/src/config/devices.yaml`, `commond.yaml` — danh sách địa chỉ IP/topic dùng để tra cứu nhanh khi kỹ thuật viên cần đối chiếu (không phải nội dung cho operator thường).
- `blk360g2_ws/` — **QUYẾT ĐỊNH (2026-08-18): KHÔNG nhắc tới trong manual, kể cả ở phụ lục.** Đây là scaffold cho đầu scan tương lai chưa dùng thật, không liên quan tới vận hành hiện tại — bỏ hẳn, không viết bất kỳ dòng nào về nó, kể cả dạng "định hướng tương lai".

## Nội dung cần có (outline)

1. **Bảng thuật ngữ** — các từ dùng xuyên suốt manual (housing, pre-scan/post-scan, job, compare, v.v.) + định nghĩa ngắn, đảm bảo nhất quán với các chương trước.
2. **Thông số kỹ thuật phần cứng** (model chính xác của lidar, encoder, PLC, gateway CAN, dải nhiệt độ/độ ẩm hoạt động…).
   > ⚠️ CẦN XÁC NHẬN TOÀN BỘ — không có trong repo, cần lấy từ tài liệu nhà sản xuất hoặc người triển khai phần cứng.
3. **Danh sách địa chỉ IP/cổng mạng nội bộ** dùng trong hệ thống (từ `devices.yaml`) — dành cho kỹ thuật viên, ghi rõ đây có thể khác nhau giữa các máy đã triển khai ở hiện trường khác nhau, không phải giá trị cố định toàn cục.
4. **Bảng mã lỗi/cảnh báo** (nếu hệ thống có mã lỗi chuẩn hóa) — kiểm tra code (`notification_center.py`, `shared/msg/Notification.msg`) xem có mã lỗi dạng chuẩn (ví dụ `ERR_001`) hay chỉ có message tự do dạng text; nếu chỉ có text tự do, **không tự bịa ra một bộ mã lỗi không tồn tại trong code** — thay vào đó tổng hợp bảng "loại cảnh báo thường gặp" dựa trên chương 08.
5. **Thông tin liên hệ hỗ trợ kỹ thuật**.
   > ⚠️ CẦN XÁC NHẬN — không có trong repo.
6. **Lịch sử thay đổi manual** (changelog) — ngày cập nhật, phiên bản phần mềm tương ứng (tham chiếu commit/tag nếu dự án có gắn tag phiên bản).

> Không có mục "định hướng tương lai" — xem quyết định ở trên, BLK360G2 không được nhắc tới trong manual.

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Xem mục 2 và 5 ở trên — đây là phần cần input từ con người nhiều nhất trong toàn bộ phụ lục.
- Dự án có quy ước đánh số phiên bản (version tag) để tham chiếu trong changelog không, hay dùng ngày tháng làm mốc?
