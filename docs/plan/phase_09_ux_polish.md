# Phase 9 — Cải thiện trải nghiệm thao tác (UX polish)

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P3 (NotificationCenter) và P5 (UI không đơ) nên xong trước để phase này tận dụng. Rủi ro: thấp.
> Phạm vi file: `ui/scripts/app.py` và các dialog manager trong `ui/scripts/`. Đây là phase "đánh bóng" — từng việc nhỏ, độc lập, làm được đến đâu tốt đến đó.

## Việc 1 — Giảm hộp thoại xác nhận

Hiện có ~135 chỗ dùng `QMessageBox` trong 9 file. KHÔNG xoá máy móc — phân loại:
- **Giữ xác nhận**: hành động không đảo ngược được hoặc ảnh hưởng cơ khí/dữ liệu — prescan ghi đè dữ liệu cũ, Set home position, Shutdown/Exit, xoá job/report.
- **Bỏ xác nhận, thay bằng thông báo + hoàn tác được**: chuyển job (`on_job_changed` — chọn job là hành động chủ đích rồi, xác nhận thêm là thừa; chuyển luôn + hiện notification "Đã chuyển sang job X"), các thao tác chỉ-đọc/điều hướng.
- **Thay warning-box bằng NotificationCenter** (P3): lỗi lưu file, lỗi nhập liệu nhẹ — hiện notification đỏ thay vì modal chặn màn hình.

Với từng dialog bỏ đi, ghi 1 dòng vào báo cáo: vị trí, lý do phân loại.

## Việc 2 — Sửa luồng chọn job cho nhất quán

`on_job_changed` (~app.py:727-774) hiện: chọn lại đúng job hiện tại → ghi file im lặng; chọn job khác → hỏi xác nhận, chọn No → đọc lại JSON + blockSignals + reset index (cơ chế revert rối). Sau khi bỏ xác nhận (Việc 1): chọn job nào chuyển job đó, không còn nhánh revert; chọn lại job hiện tại → không làm gì (không ghi file thừa).

## Việc 3 — Auto-report im lặng

`on_cloud_received` hiện `return` im lặng nếu `cbbAutoCompare`/`cbbAutoReport` không cùng bật (~app.py:399-400) — người dùng quét xong không thấy gì và không biết vì sao. Sửa: khi cloud so sánh về mà auto-report tắt → notification info: "Auto report is off — report not generated" (đi qua NotificationCenter). Chỉ 1 thông báo, không dialog.

## Việc 4 — Rà soát nhỏ còn lại

- `on_shutdown`/Exit: nếu P8 đã làm thì bỏ qua; chưa thì giữ nguyên.
- Kiểm tra các dialog quản lý job/report mở đúng vị trí, nút đủ to cho cảm ứng — CHỈ ghi nhận vấn đề vào Ghi chú phát sinh, không tự ý redesign layout (đổi layout cần người dùng duyệt).

## Kiểm chứng

1. `py_compile` file sửa.
2. Chạy app: chuyển job không còn dialog, có notification; quét với auto-report tắt → thấy thông báo lý do; các hành động nguy hiểm (set home, prescan ghi đè, exit) VẪN hỏi xác nhận.

## Tiêu chí nghiệm thu

- [ ] Bảng phân loại dialog (giữ/bỏ/chuyển notification) có trong báo cáo; các dialog thuộc nhóm "bỏ" đã bỏ.
- [ ] Luồng chọn job không còn cơ chế revert blockSignals.
- [ ] Auto-report tắt không còn im lặng.
- [ ] Commit `[P9] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

_(chưa có)_

## Ghi chú phát sinh

_(chưa có)_
