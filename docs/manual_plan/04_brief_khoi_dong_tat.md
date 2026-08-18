# Brief 04 — Khởi động & tắt hệ thống hằng ngày

**File output:** `docs/manual/04_khoi_dong_tat.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator (thao tác mỗi ngày)
**Mức ưu tiên:** Cao — viết dạng các bước ngắn gọn (bấm gì, đợi gì, thấy gì là xong), không giải thích Docker/container/ROS bên trong (xem văn phong ở `00_INDEX.md` mục 1.4).

## Nguồn thông tin / code cần đọc

- `run_docker.sh` — logic icon Desktop: dừng container hiện tại (nếu có) → lưu log của lần chạy trước (`data/logs/last_run.log` → `previous_run.log`) → khởi động container mới sạch từ image đã build (không rebuild) → xoay màn hình tablet nếu có output `DSI-1` → `xhost`/`DISPLAY` warning nếu thiếu.
- `run_intelijet.sh` — script chạy bên trong container: set biến môi trường Qt, build workspace lần đầu nếu chưa có `devel/` (chỉ lần đầu, các lần sau không build lại), `roslaunch pps pps.launch`.
- `docs/optimization_plan_overview.md` mục 7 — hiện trạng nút Exit trên UI (`btnShutdown` → `on_shutdown` → `closeEvent`, dùng `rosnode kill -a`) — **lưu ý: đây là hành vi hiện tại, có thể thay đổi khi Hạng mục F trong kế hoạch tối ưu được triển khai** (chuyển hẳn sang Docker Compose lifecycle) — kiểm tra lại code UI thật tại thời điểm viết, đừng chỉ copy mô tả từ tài liệu kế hoạch.
- `data/.ui_ready` — file marker báo UI đã sẵn sàng (theo comment trong `run_docker.sh`), có thể dùng để giải thích "làm sao biết app đã khởi động xong".

## Nội dung cần có (outline)

1. Khởi động hằng ngày: chỉ cần bấm icon "Intelijet" trên Desktop/menu ứng dụng — **mỗi lần bấm sẽ dừng phiên đang chạy (nếu có) và khởi động lại sạch từ đầu**, không phải "mở lại" phiên cũ. Nêu rõ điều này để operator không hoang mang nếu đã có app đang chạy mà bấm icon lại.
2. Thời gian chờ khởi động thông thường và dấu hiệu nhận biết app đã sẵn sàng (giao diện chính hiện ra).
3. Cách tắt hệ thống đúng cách cuối ca làm việc: dùng nút Exit trong giao diện — mô tả **chính xác** hành vi hiện tại của nút này (đọc code, không suy đoán) và có dặn dò gì kèm theo không (ví dụ: đợi report đang xuất xong trước khi tắt?).
4. Cảnh báo lỗi thường gặp lúc khởi động (ví dụ cửa sổ không hiện ra do `$DISPLAY` không được set, hay `xhost` thất bại) — phần chi tiết cách xử lý để ở chương 08 (Xử lý sự cố), ở đây chỉ nêu triệu chứng để operator biết khi nào cần gọi kỹ thuật viên.
5. Vị trí file log của lần chạy gần nhất/gần nhì (`data/logs/last_run.log`, `data/logs/previous_run.log`) — hữu ích khi cần báo lỗi cho kỹ thuật viên.

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Có quy trình tắt máy tính/phần cứng vật lý (không chỉ phần mềm) cuối ngày không (ví dụ tắt nguồn housing, tắt PLC) — không có trong code, cần người vận hành thực tế xác nhận.
