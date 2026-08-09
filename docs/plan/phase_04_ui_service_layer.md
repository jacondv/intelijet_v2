# Phase 4 — Tách business logic khỏi god-object `App` (chưa threading)

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P1. Rủi ro: vừa.
> Phạm vi file: `ui/scripts/app.py` (838 dòng) + file mới trong `ui/src/ui/services/`. KHÔNG thêm threading ở phase này — đó là P5. Phase này chỉ **di chuyển code, giữ nguyên hành vi**, để P5 có ranh giới sạch mà đưa vào worker thread.

## Vì sao tách trước, thread sau

Nếu thread hoá code đang nằm lẫn trong `App` (widget access rải rác giữa logic), rất dễ tạo bug truy cập widget từ thread nền. Tách trước để mỗi service **không đụng widget nào** — sau đó P5 chỉ việc bọc service vào QThread.

## Hiện trạng `App` (đã khảo sát — các mốc dòng là xấp xỉ, xác định lại theo tên hàm)

- `on_cloud_received` (~315-415): convert PointCloud2→Open3D (`pointcloud2_to_o3d_tensor`), `assign_colors`, convert sang VTK + `vtk_viewer.update`, `save_job()` (ghi PLY), và nếu là cloud so sánh + auto bật → gọi `export_report` ngay trong slot.
- `export_report` (~538-606): gọi `ReportGenerator().export(...)` (tunnel_report) + `shutil.copy`.
- `save_job` (~717-...): ghi PLY qua `CloudConverter.o3d_to_ply`.
- Quản lý job: đọc/ghi `current_job.json`, `active_jobs.json` rải rác (~734, 757, 765, 788, 819) + monkeypatch `cbbJobSelect.showPopup` (~204-209) để reload file mỗi lần mở dropdown.
- Import cục bộ trong hàm (circular-import workaround): `from pps.data_converter import CloudConverter` bên trong nhiều hàm.
- `CloudConverter()` được khởi tạo lại mỗi lần dùng (~319, 506, 713).

## Thiết kế đích

Tạo `ui/src/ui/services/` với 3 file mới. Quy tắc sắt cho cả 3: **không import PyQt widget, không truy cập `self.ui.*`** — chỉ nhận/trả dữ liệu thuần (đường dẫn, dict, object Open3D). Được phép dùng `rospy.log*`.

### 1. `services/job_store.py` — class `JobStore`
- Nguồn sự thật duy nhất cho `current_job.json` / `active_jobs.json`: `load_active_jobs()`, `get_current_job()`, `set_current_job(...)`, `add_job(...)` (đối chiếu những thao tác thực tế đang có trong `app.py` và `project_dlg_manager.py` — chỉ cần phủ những gì `app.py` đang dùng; các dialog manager khác sẽ chuyển sang dùng dần, KHÔNG bắt buộc sửa hết trong phase này).
- Cache trong RAM + `reload()` tường minh. Ghi file qua pattern ghi-tạm-rồi-rename (`os.replace`) để không hỏng JSON khi mất điện giữa chừng (máy hiện trường).
- Xử lý file hỏng/thiếu: trả mặc định rỗng + log warning, không crash.

### 2. `services/cloud_pipeline.py` — class `CloudPipelineService`
Gom phần xử lý dữ liệu (không-widget) của `on_cloud_received`:
- `process_incoming(msg, highlight_range) -> ProcessedCloud` — convert PointCloud2→Open3D tensor, assign_colors; trả object nhỏ chứa `o3d_cloud` + metadata.
- `to_vtk(o3d_cloud) -> polydata` — convert sang VTK (App sẽ gọi và tự đưa vào viewer).
- `save_cloud(o3d_cloud, filepath)` — ghi PLY.
- Giữ **một** instance `CloudConverter` dùng lại. Import `pps.data_converter` ở **đầu file** service (module này chỉ được import lazily bởi App hiện tại do vòng import trong app.py — service file mới không dính vòng đó; nếu vẫn dính, giữ import trong hàm nhưng ghi chú lý do).

### 3. `services/report_service.py` — class `ReportService`
- `export(o3d_cloud, job_info, output_paths...) -> str (đường dẫn pdf)` — chuyển toàn bộ thân `export_report` hiện tại vào đây (gọi `ReportGenerator`, `shutil.copy`, dựng tên file từ job). Logic dựng tên/parse `basename.split("#")` cũng chuyển vào đây.
- Lỗi: raise exception có message rõ (không nuốt bằng print) — App bắt và đưa ra NotificationCenter (nếu P3 đã xong) hoặc QMessageBox (nếu chưa).

### 4. `App` sau khi tách
- `on_cloud_received` chỉ còn: gọi service → nhận kết quả → cập nhật `vtk_viewer` + label + (nếu điều kiện auto) gọi `report_service.export(...)` — **vẫn đồng bộ như cũ** (P5 mới chuyển async), nhưng giờ mỗi bước là 1 call service rõ ràng.
- Bỏ monkeypatch `showPopup`: thay bằng đọc từ `JobStore` (cache); refresh cache khi nào dữ liệu job thay đổi (sau khi dialog quản lý job đóng, sau khi add job) thay vì mỗi lần mở dropdown. Nếu có nguồn ghi file ngoài app (Syncthing đồng bộ giữa 2 tablet — có thật trong dự án này), thêm refresh định kỳ nhẹ (ví dụ mỗi 30s bằng QTimer) thay vì mỗi lần mở dropdown.
- Sửa các `except Exception: print(...)` trong vùng code được di chuyển → `rospy.logerr` + thông báo người dùng. (Chỉ trong vùng code phase này đụng tới.)

## Ràng buộc

- Hành vi nhìn thấy được phải y hệt trước (kể cả độ đơ — chưa sửa đơ ở phase này).
- Không sửa `ros_thread.py`, `vtk_viewer.py`, `tunnel_report/*` (trừ khi buộc phải thêm tham số — ghi chú nếu vậy).
- Không đổi format 2 file JSON.
- Nếu 838 dòng của `app.py` + 3 file mới quá dài cho một phiên: ưu tiên hoàn thành theo thứ tự `ReportService` → `CloudPipelineService` → `JobStore`; phần chưa xong ghi `[WIP]` theo quy tắc INDEX.

## Kiểm chứng

1. `py_compile` toàn bộ file sửa/mới; import-check các service (không cần ROS: mock rospy nếu tiện, không thì py_compile).
2. Test nhỏ cho `JobStore` (không cần ROS/Qt): tạo thư mục tạm, ghi/đọc/hỏng file → hành vi đúng. Lưu tại `ui/src/ui/tests/test_job_store.py`.
3. Nếu chạy được app: quét thử 1 chu trình đầy đủ (prescan → postscan → compare → report) xác nhận kết quả file/PDF y như trước.

## Tiêu chí nghiệm thu

- [ ] 3 service không import widget/`self.ui`; `App` chỉ còn điều phối + cập nhật widget.
- [ ] Monkeypatch `showPopup` đã bỏ, dropdown vẫn có dữ liệu đúng (kể cả khi file job thay đổi từ ngoài).
- [ ] Ghi JSON atomic (`os.replace`).
- [ ] Test JobStore pass; hành vi app không đổi.
- [ ] Commit `[P4] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

**Trạng thái: ✅ Xong.** Commit `f045736`.

### Đã làm
- `ui/src/ui/services/job_store.py`: `JobStore` — nguồn sự thật duy nhất cho `active_jobs.json`/`current_job.json`; ghi atomic (`.tmp` + `os.replace`); xử lý file thiếu/hỏng/sai kiểu bằng cách trả mặc định rỗng + log warning, không crash. **Đã chạy test thật** (không chỉ py_compile) — 7 test case pass 100% trong sandbox này (đây là service duy nhất không phụ thuộc ROS/Qt nên chạy được trực tiếp).
- `ui/src/ui/services/cloud_pipeline.py`: `CloudPipelineService` — gom `pointcloud2_to_o3d`, `assign_colors_for_highlight`, `to_vtk`, `save_ply`; giữ 1 instance `CloudConverter` dùng lại thay vì tạo mới 3 nơi như trước (đúng điểm kế hoạch nêu ở dòng ~319, 506, 713 gốc).
- `ui/src/ui/services/report_service.py`: `ReportService.export()` — chuyển nguyên thân `export_report` cũ, đổi từ "in lỗi rồi nuốt" sang **raise exception rõ ràng** khi thất bại (đúng yêu cầu thiết kế), App bắt và đẩy qua `NotificationCenter` (đã có từ Phase 3).
- `app.py`: `on_cloud_received`, `update_pointcloud_from_data`, `export_report` giờ chỉ còn điều phối (gọi service → cập nhật widget); bỏ hẳn `save_job()`, bỏ monkeypatch `showPopup`, bỏ toàn bộ JSON I/O rải rác trong `on_job_changed`/`load_current_job` — thay bằng gọi `JobStore`. Thêm `QTimer` refresh danh sách job mỗi 30s (giữ nguyên lựa chọn hiện tại khi refresh, dùng `blockSignals`) thay cho việc đọc lại file mỗi lần mở dropdown.
- Dọn 2 import không còn dùng (`ReportGenerator`, `delete_old_final_report`, `shutil`) sau khi logic chuyển sang `ReportService`.
- `py_compile` pass cho toàn bộ 6 file sửa/mới.

### Đối chiếu ràng buộc
- 3 service không import PyQt, không đụng `self.ui` — đã soát bằng mắt toàn bộ 3 file, chỉ dùng `rospy.log*`.
- Không sửa `ros_thread.py`, `vtk_viewer.py`, `tunnel_report/*` — đúng, không đụng.
- Không đổi format 2 file JSON — đúng, `JobStore` đọc/ghi cùng cấu trúc `{"current_job": "..."}` và list `[{"project":..., "job":...}]` như cũ.
- Grep xác nhận `project_dlg_manager.py`, `job_select_manager.py`, `report_view_dlg_manager.py`, `history_page_manager.py`, `compare_dlg_manager.py` đều có `ACTIVE_JOB_FILE`/hàm đọc job **riêng, độc lập**, không import từ `app.py` — refactor không ảnh hưởng các file này (đúng như kế hoạch cho phép).

### Sai khác nhỏ so với mô tả gốc trong kế hoạch (đã cân nhắc, ghi lại)
- Không tạo `ProcessedCloud` wrapper object như gợi ý — `process_incoming()` trả thẳng `o3d_cloud` vì không có metadata nào khác cần mang theo trong pipeline hiện tại; thêm wrapper sẽ là abstraction thừa. Có sẵn `process_incoming()` (gộp convert+color) lẫn 2 hàm rời `pointcloud2_to_o3d()`/`assign_colors_for_highlight()` — `app.py` dùng 2 hàm rời vì cần bước "convert trước, quyết định có tô màu hay không sau" theo đúng logic gốc (một số topic không tô màu).
- `JobStore` không có `add_job(...)` như liệt kê trong kế hoạch — vì `app.py` không có nơi nào gọi add-job (việc thêm job nằm ở `project_dlg_manager.py`, ngoài phạm vi phase), nên không thêm API chưa có nơi dùng.
- Không tìm cách "hook vào lúc dialog quản lý job đóng" để refresh cache — các trang quản lý job trong `app.py` là widget nhúng thường trực (tab), không phải dialog modal, nên không có sự kiện "đóng" rõ ràng để hook; dùng phương án dự phòng kế hoạch đã cho phép (QTimer 30s).

### Chưa kiểm chứng được trong phiên này
- `CloudPipelineService`, `ReportService` phụ thuộc `rospy`/`pps.data_converter`/Open3D/`ui.tunnel_report` — không chạy thật được trong sandbox này (không có ROS/Open3D), chỉ `py_compile`.
- Chưa chạy được app thật để quét 1 chu trình đầy đủ prescan→postscan→compare→report như mục 3 phần Kiểm chứng yêu cầu — cần Docker.

## Ghi chú phát sinh

1. `update_pointcloud_from_data` (được gọi từ `history_page_in_toolbox.polydataSignal` và `compare_dlg_manager` qua `jobcompare_dlg.polydataSignal`) nay dùng `self.cloud_pipeline.to_vtk(...)` — các signal này connect trong `__init__` TRƯỚC dòng khởi tạo `self.cloud_pipeline`, nhưng vì Qt signal chỉ thực thi khi có sự kiện thật (không đồng bộ ngay lúc connect), nên thứ tự này an toàn — đã kiểm tra kỹ, không phải bug.
2. `ui/services/`, `ui/tests/` không được liệt kê trong `packages=['ui']` của `setup.py` — nhưng các subpackage có sẵn từ trước (`ui.models`, `ui.tunnel_report`, `ui.widgets`) cũng không được liệt kê và vẫn chạy được nhờ `catkin_python_setup()` symlink cả cây thư mục trong devel space — giữ nguyên quy ước hiện có, không sửa `setup.py`.
3. Phát hiện phụ (không sửa, ngoài phạm vi): trong `report_service.py` (nguyên bản từ `export_report` cũ), fallback `applied_thickness` không nhất quán — nhánh có `job_info` dùng fallback `10`, nhánh không có `job_info` dùng fallback `40`. Đây là hành vi/giá trị hardcode có sẵn từ trước, giữ nguyên theo đúng nguyên tắc "chỉ di chuyển, không sửa hành vi"; có thể dọn khi làm Phase 6.
