# Brief 09 — Bảo trì & cập nhật phần mềm

**File output:** `docs/manual/09_bao_tri_cap_nhat.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Kỹ thuật viên
**Mức ưu tiên:** Trung bình — chỉ liệt kê lệnh cần chạy + khi nào chạy, không giải thích vì sao (image, cache, volume mount...) trừ khi ảnh hưởng trực tiếp tới việc dữ liệu có bị mất hay không.

## Nguồn thông tin / code cần đọc

- `docs/INSTALL.md` phần "Cập nhật code sau này" — `git pull` rồi `docker compose build`, lần bấm icon tiếp theo mới dùng image mới.
- `docker-compose.yml` — các volume mount quan trọng cần biết khi backup/troubleshoot: `data/.cache` (cache model AI — không cần backup, tự tải lại được nhưng cần internet), `intelijet_v2_ws/src/config/.config` (QSettings — cấu hình người dùng lưu ở đây, **nên backup**), `data/.config/qpdfview` (state UI xem PDF, không quan trọng).
- `intelijet_v2_ws/src/config/` — các file cấu hình cần biết vị trí khi backup: `devices.yaml`, `commond.yaml`, `runtime.yaml`, `last_used.yaml`, `lidar.yaml`.
- `data/Projects` — nơi lưu dữ liệu job/report thật (theo `commond.yaml`: `DATA_DIR: "data"`) — đây là dữ liệu quan trọng nhất cần backup định kỳ.
- Phần Syncthing trong `intallation_gui.md` — cơ chế đồng bộ/backup giữa 2 tablet đã có sẵn cho `scanner_config` và `scanner_results` — tham chiếu, không lặp lại chi tiết (để ở chương 10).

## Nội dung cần có (outline)

1. Cập nhật phần mềm lên phiên bản mới: `git pull` + `docker compose build` — ai được phép làm (kỹ thuật viên, không phải operator thường), khi nào nên làm (không nên làm giữa ca đang có việc).
2. Những gì cần backup định kỳ và ở đâu: `data/Projects` (dữ liệu job/report), `intelijet_v2_ws/src/config/*.yaml` (cấu hình).
3. Đồng bộ tự động qua Syncthing đã có sẵn cho 2 nhóm dữ liệu trên — nhắc ngắn, trỏ sang chương 10 để biết cách kiểm tra tình trạng đồng bộ.
4. Dọn dẹp định kỳ (nếu cần) — ví dụ log cũ trong `data/logs/`, dữ liệu job cũ không cần giữ.
5. Kiểm tra sức khỏe hệ thống định kỳ (nếu có quy trình) — ví dụ kiểm tra dung lượng ổ đĩa còn trống, vì dữ liệu quét point cloud có thể chiếm nhiều dung lượng.

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Có lịch bảo trì phần cứng định kỳ không (vệ sinh đầu quét, kiểm tra dây CAN, siết lại cơ khí housing…) — hoàn toàn không có trong code, cần thông tin từ nhà sản xuất/kỹ thuật viên phần cứng.
- Chính sách giữ/xóa dữ liệu job cũ (giữ bao lâu, ai quyết định xóa) — không có trong code.
