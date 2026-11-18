# Brief 05 — Tổng quan giao diện người dùng

**File output:** `docs/manual/05_giao_dien.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator (tra cứu nhanh "nút này để làm gì")
**Mức ưu tiên:** Trung bình — viết THẬT GỌN. Không phải tài liệu tour toàn bộ giao diện, chỉ cần đủ để operator nhận ra màn hình mình đang thấy và biết đèn/nút liên quan đến an toàn (chương 02) và sự cố (chương 08) nằm ở đâu. Chi tiết từng dialog phụ (compare, report, history…) không cần liệt kê nút bấm đầy đủ — để chương 06 (quy trình) dẫn dắt theo thao tác thật, chương này chỉ là bản đồ tổng quát 1 lần.

## Nguồn thông tin / code cần đọc

Các file `*_ui.py` (định nghĩa layout, tên đúng của label/nút) và `*_manager.py`/`*_dlg_manager.py` (xử lý hành vi) tương ứng trong `intelijet_v2_ws/src/ui/`:

- Màn hình chính: `ui/scripts/app.py` (class `App` — điểm vào chính, xem `docs/optimization_plan_overview.md` mục 4 để biết đây là "god object" đang được tách dần, nhưng vẫn là nơi đọc để hiểu toàn bộ luồng UI hiện tại).
- Chọn/tạo job: `jobnumber_page_manager.py`, `jobselect_dlg_ui.py`, `job_select_dlg_ui.py`, `job_item_ui.py`, `job_item_widget.py`, `project_dlg_manager.py`, `project_dlg_ui.py`.
- Cài đặt job/thiết lập quét: `setting_page_manager.py`, `setting_page_ui.py`, `jobsetting_page_ui.py`.
- So sánh (compare): `compare_dlg_manager.py`, `compare_dlg_ui.py`, `compare_cloud_worker.py` (worker chạy nền cho compare).
- Xem report: `reportselect_dlg_manager.py`, `reportselect_dlg_ui.py`, `report_view_dlg_manager.py`.
- Lịch sử: `history_page_manager.py`, `historyview_page_ui.py`.
- Bàn phím ảo trên màn hình cảm ứng: `keyboard.py`.
- Xem 3D point cloud: `vtk_viewer.py`.
- Trung tâm thông báo/cảnh báo: `notification_center.py`, `notification_history_dialog.py` (chi tiết hành vi để ở chương 07, ở đây chỉ mô tả vị trí/hình dạng trên UI).
- Bộ icon có sẵn: `intelijet_v2_ws/src/ui/icon/` (arrow, home, search, user, x-mark…) — tên file gợi ý chức năng nút tương ứng.

## Nội dung cần có (outline)

1. Bố cục màn hình chính: các vùng chính (khu vực xem 3D, khu vực thông tin job, khu vực trạng thái thiết bị, khu vực nút thao tác) — mô tả bằng chữ + `[ẢNH: màn hình chính]`.
2. Với mỗi màn hình/dialog phụ (chọn job, tạo job mới, cài đặt, compare, xem report, lịch sử): mục đích, cách mở, các nút chính và ý nghĩa — dùng đúng tên nhãn lấy từ code (không tự dịch/đặt tên khác).
3. Bàn phím ảo: khi nào tự hiện/tự ẩn.
4. Khu vực trạng thái thiết bị + thông báo (mô tả ngắn ở đây, chi tiết ý nghĩa màu sắc/mức độ để ở chương 07 — tránh trùng lặp nội dung).

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Ảnh chụp màn hình thật cho từng mục — chương này sẽ dùng placeholder `[ẢNH: mô tả]` cho tới khi có ảnh thật được cung cấp và chèn vào.
- Có phiên bản UI nào khác nhau giữa các máy/tablet (ví dụ Tablet A vs Tablet B nhắc tới trong `intallation_gui.md` phần Syncthing) hay tất cả máy chạy cùng 1 giao diện giống hệt nhau?
