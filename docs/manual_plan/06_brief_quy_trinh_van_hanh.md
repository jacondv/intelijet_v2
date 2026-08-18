# Brief 06 — Quy trình vận hành hằng ngày (SOP quét)

**File output:** `docs/manual/06_quy_trinh_van_hanh.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator
**Mức ưu tiên:** CAO NHẤT — đây là nội dung cốt lõi, được dùng nhiều nhất trong manual.

**Văn phong:** đọc code kỹ để hiểu ĐÚNG trình tự thao tác thật, nhưng khi viết ra chỉ trình bày các bước bằng ngôn ngữ thao tác tay chân ("bấm nút X", "đợi tới khi màn hình hiện Y", "nếu thấy Z thì..."). Không nhắc tên hàm/action/topic ROS trong bản viết ra — các tên đó chỉ để agent tự đối chiếu khi nghiên cứu (xem `00_INDEX.md` mục 1.4).

## Nguồn thông tin / code cần đọc

- `intelijet_v2_ws/src/pps/action/` — `PreScan.action`, `StartScan.action`, `CompareCloud.action` — định nghĩa rõ các bước nghiệp vụ chính (pre-scan, scan, compare) ở mức action ROS, đọc để hiểu đúng trình tự thật, không đoán.
- `intelijet_v2_ws/src/pps/scripts/hmi_scan_command_handler.py` — nơi nhận lệnh từ giao diện (HMI) và điều phối sang scan controller — đọc để biết chính xác lệnh nào tương ứng với nút nào trên UI.
- `intelijet_v2_ws/src/pps/src/pps/generic_scan_controller.py` và `sick_scan_eRob_controller.py` — luồng điều khiển housing + quét thật (mở housing theo bước, quét, ghép scan `assemble_scans2`, đóng housing) — xem thêm `docs/optimization_plan_overview.md` mục 5 để hiểu bối cảnh (đây là cách hoạt động hiện tại, gắn với đầu SickScan, có thể đổi kiến trúc sau nhưng SOP vận hành thật phải mô tả đúng cái đang chạy).
- `ui/scripts/app.py` — hàm `on_cloud_received` → luồng xử lý sau khi nhận cloud mới (convert, tô màu, lưu PLY, tính bề dày, xuất report) — xem `docs/optimization_plan_overview.md` mục 2 để biết đây là luồng đồng bộ trên GUI thread hiện tại (nghĩa là **UI có thể "đứng hình" một lúc trong bước này** — operator cần biết đây là bình thường, không phải lỗi, cho tới khi Hạng mục A trong kế hoạch tối ưu được triển khai xong).
- `history_page_manager.py`, `jobnumber_page_manager.py` — quy trình chọn job hiện tại vs chọn job khác (ghi chú trong `docs/optimization_plan_overview.md` mục 6: luồng này hiện "khá rối", cần đọc kỹ code thật thay vì mô tả lý tưởng).
- WeasyPrint report — tìm template report (tìm trong `pps/` hoặc `ui/`) để mô tả report cuối cùng trông như thế nào, xuất ra đâu.
- **Cánh tay thủy lực (2026-08-18, bổ sung từ người dùng, không có trong tài liệu trước đó)**: `ui_can_interface/src/ui_can_interface/command_handler.py` (lệnh `PLC_OPEN_HOUSING`/`PLC_CLOSE_HOUSING` gửi kèm `ScannerExtendSpeedInHz`/`ScannerRetractSpeedInHz` qua CAN) + `ui_can_interface/src/ui_can_interface/can_pdo_config.yaml` — đây rất có thể là cơ cấu điều khiển cánh tay/housing thật, nhưng **cần hỏi lại người dùng để xác nhận** đây có đúng là lệnh điều khiển cánh tay thủy lực hay là một cơ cấu khác trước khi mô tả bước "định vị cánh tay" trong SOP (xem câu hỏi ở `02_brief_an_toan.md` mục 3b — quan hệ giữa cánh tay và Housing).

## Nội dung cần có (outline) — dạng quy trình từng bước, có ảnh minh họa placeholder

1. Tạo job mới / chọn job có sẵn — 2 luồng khác nhau, mô tả rõ từng luồng riêng (không gộp chung gây nhầm).
2. Chuẩn bị trước khi quét: **định vị cánh tay thủy lực để đưa Housing tới gần bề mặt cần quét** (bước MỚI, bổ sung 2026-08-18 — xem nguồn ở trên), sau đó mở Housing — nhắc lại cảnh báo an toàn tương ứng (chương 02) ngay tại bước này, không chỉ để ở chương an toàn riêng.
3. Thực hiện Pre-scan (quét trước khi có tác động/thay đổi).
4. Thực hiện Post-scan (quét sau).
5. So sánh (Compare) — bấm nút nào, đợi bao lâu, kết quả hiển thị ra sao (dùng `compare_cloud_worker.py` để biết đây có chạy nền hay đồng bộ).
6. Xuất report PDF — tự động hay cần bấm nút, report gồm nội dung gì, lưu ở đâu (liên hệ `commond.yaml`: `BASE_DIR`, `DATA_DIR`).
7. Auto-report: theo `docs/optimization_plan_overview.md` mục 6, có 2 combobox điều kiện bật/tắt tính năng auto-report — mô tả đúng hành vi hiện tại (kể cả nếu hiện tại tắt thì "im lặng không làm gì" — operator cần biết để không tưởng nhầm là lỗi).
8. Xem lại report/job cũ trong lịch sử.
9. Lưu ý trong lúc UI "đứng hình" tạm thời khi xử lý report (mục 5 ở trên) — đây là hành vi hiện tại đã biết, không phải operator làm sai.

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Trình tự thao tác thực tế tại hiện trường có đúng như luồng suy ra từ code không? (Cách tốt nhất: quan sát/phỏng vấn 1 operator thực hiện quy trình thật 1 lần, đối chiếu với thứ tự bước viết ở đây trước khi phát hành.)
- Ý nghĩa nghiệp vụ của "pre-scan" vs "post-scan" (quét trước/sau cái gì cụ thể — trước/sau khi thi công? trước/sau khi lắp đặt?) — code chỉ cho biết cơ chế kỹ thuật, không cho biết ý nghĩa nghiệp vụ.
