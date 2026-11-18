# Phase 5 — Đưa pipeline cloud + report vào worker thread (hết đơ UI)

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: **P4 bắt buộc** (service đã tách, không đụng widget). Rủi ro: vừa-cao — đây là phase quan trọng nhất với người dùng.
> Phạm vi file: `ui/scripts/app.py`, file mới `ui/src/ui/scan_pipeline_worker.py`, `ui/scripts/vtk_viewer.py` (2 sửa nhỏ). Tham chiếu pattern mẫu: `ui/src/ui/compare_cloud_worker.py` (`CompareWorker(QThread)` — pipeline so sánh thủ công ĐÃ chạy nền đúng cách, bắt chước cấu trúc của nó).

## Vấn đề (đã khảo sát, root cause chắc chắn)

Khi cloud mới về, slot `App.on_cloud_received` (GUI thread) chạy đồng bộ toàn chuỗi: convert → tô màu → VTK render → ghi PLY → (nếu auto) `export_report`: dựng mesh tính bề dày 2 lần + matplotlib chart + Open3D OffscreenRenderer + WeasyPrint PDF + copy file. Tổng vài giây đến vài chục giây → UI đơ hoàn toàn. Sau P4, các bước này đã nằm gọn trong `CloudPipelineService` và `ReportService` — phase này chuyển nơi thực thi.

## Thiết kế đích

### 1. `scan_pipeline_worker.py` — class `ScanPipelineWorker(QThread)`
Mô phỏng cấu trúc `CompareWorker`:
- Input: `msg` (PointCloud2), cấu hình cần thiết (highlight_range, job_info, cờ auto_report...) — copy giá trị lúc submit, không đọc widget từ thread.
- `run()` thực hiện tuần tự: `process_incoming` → `to_vtk` → `save_cloud` → (nếu auto) `report_service.export`.
- Signals (chỉ mang dữ liệu thuần / polydata / đường dẫn):
  - `cloud_ready(polydata, metadata)` — GUI thread nhận và gọi `vtk_viewer.update(...)` + cập nhật label.
  - `report_done(pdf_path)` / `report_failed(error_message)`.
  - `progress(message)` — tuỳ chọn, đẩy qua NotificationCenter (P3) dạng transient.
- **Hàng đợi 1 chỗ**: nếu cloud mới đến khi worker đang chạy → giữ lại **cái mới nhất** và chạy tiếp sau khi xong (không xếp hàng dài, không chạy song song 2 worker — Open3D/VTK không cần tranh chấp). Cloud cũ bị thay thế thì log info.
- Worker là **thành viên của App** (không tạo mới mỗi lần rồi thả trôi — tránh QThread bị GC khi đang chạy). Dùng lại 1 instance + biến trạng thái, hoặc pattern `moveToThread`; chọn cách nào giống `CompareWorker` nhất để đồng bộ phong cách.

### 2. Sửa `App.on_cloud_received`
Chỉ còn: đóng gói input → submit vào worker → return ngay. Mọi cập nhật UI chuyển sang các slot nối với signal của worker. Trong lúc worker chạy: hiển thị trạng thái "Đang xử lý..." (qua NotificationCenter/label), KHÔNG khoá nút — người dùng vẫn thao tác được.

### 3. Chống xung đột khi đang xử lý
- Nếu người dùng bấm scan mới trong khi report đang xuất: cho phép (worker xử lý tuần tự theo hàng đợi 1 chỗ ở trên).
- `closeEvent`: chờ worker kết thúc tối đa ~5s (`worker.wait(5000)`), quá thì bỏ qua và thoát (ghi log) — không treo cửa sổ khi tắt.

### 4. Hai sửa nhỏ giảm lag render (`vtk_viewer.py`)
a. **Downsample trước khi render**: trong đường dẫn hiển thị (KHÔNG đụng dữ liệu lưu file/report — dữ liệu gốc giữ nguyên chất lượng), nếu số điểm > ngưỡng `MAX_RENDER_POINTS` (hằng số đầu file, mặc định 2_000_000) thì voxel-downsample bản hiển thị. Vị trí hợp lý: trong `CloudPipelineService.to_vtk` (thêm tham số `max_points`).
b. **Không dựng lại box widget mỗi lần update**: `VTKViewer.update()` hiện gọi `_enable_box_widget()` tạo mới `vtkBoxWidget` + observer mỗi lần cloud về (~dòng 260, 206-230). Sửa: tạo box widget **một lần**, các lần sau chỉ `PlaceWidget()` lại theo bound mới. Giữ nguyên hành vi nhìn thấy.

## Ràng buộc an toàn thread (đọc kỹ)

- **Tuyệt đối không** truy cập widget/`self.ui.*`/`vtk_viewer` từ trong `run()` của worker. Mọi thứ về UI đi qua signal.
- VTK render chỉ trên GUI thread (convert sang polydata trong worker thì được — chỉ là dữ liệu; `vtk_viewer.update()` gọi ở slot GUI).
- Signal-slot giữa QThread và GUI mặc định queued — không cần cấu hình thêm, nhưng không truyền object Open3D lớn qua nhiều signal liên tiếp không cần thiết.
- `rospy` callback (ros_thread) → `cloud_received_signal` → slot App → submit worker: chuỗi này giữ nguyên, chỉ thân slot đổi.

## Kiểm chứng

1. `py_compile` mọi file sửa/mới.
2. Chạy app (Docker): thực hiện scan + auto compare + auto report; trong lúc chờ report, xác nhận: xoay/zoom được point cloud, bấm được nút, label trạng thái vẫn nhảy 1 Hz. Đây là tiêu chí thành công CHÍNH của cả kế hoạch — mô tả kết quả cụ thể trong báo cáo.
3. Xuất 2 scan liên tiếp nhanh → không crash, kết quả cuối cùng là của scan mới nhất.
4. Đóng app khi đang xuất report → thoát trong ≤ ~6s, không zombie.
5. So sánh 1 file PDF report trước/sau phase → nội dung tương đương (không giảm chất lượng).

## Tiêu chí nghiệm thu

- [ ] UI không đơ trong toàn bộ chu trình scan→compare→report (kiểm chứng thật nếu có môi trường; nếu không, mô tả rõ phần chưa kiểm chứng được).
- [ ] Worker 1 instance, hàng đợi 1 chỗ, không truy cập widget từ thread.
- [ ] Box widget không bị tạo mới mỗi frame; bản hiển thị có giới hạn điểm, dữ liệu lưu/report giữ nguyên.
- [ ] `closeEvent` không treo.
- [ ] Commit `[P5] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

**Trạng thái: ✅ Xong (mặt code) — CHƯA kiểm chứng thật trong Docker.** Commit `445696c`.

### Đã làm
- `ui/src/ui/scan_pipeline_worker.py` (mới): `ScanPipelineWorker(QThread)` — 1 instance sống cùng vòng đời `App` (khác `CompareWorker` tạo mới mỗi lần bấm nút). `submit()` thread-safe, hàng đợi 1 chỗ (`_current_job`/`_pending_job` bảo vệ bằng `threading.Lock`): job mới đến khi đang chạy → thay thế job đang chờ (chưa chạy), job đang chạy luôn được chạy hết. `run()` là bản port gần như từng dòng của `on_cloud_received` cũ, chỉ đọc/ghi `job` dict + gọi `CloudPipelineService`/`ReportService`/`JobStore` (đều không đụng widget từ Phase 4) — không đụng `self.ui.*`/`vtk_viewer` bao giờ.
- `app.py::on_cloud_received`: giờ chỉ đóng gói state cần thiết (snapshot `isManualCompare`, `current_post_scan_path`, 2 giá trị combobox auto-compare/auto-report đọc ngay trên GUI thread) vào dict rồi `submit()` — return ngay, không chặn GUI thread. Thêm 3 slot mới (`_on_scan_cloud_ready`, `_on_scan_report_done`, `_on_scan_report_failed`) nhận kết quả qua signal và cập nhật `vtk_viewer`/`self.report_name`/`self.isManualCompare`/`self.current_post_scan_path`/`NotificationCenter` — **đã soát kỹ để giữ đúng điều kiện mutate gốc** (ví dụ `current_post_scan_path` reset khi `isManualCompare` true, độc lập với điều kiện reset `isManualCompare`, đúng như code gốc chứ không gộp chung thành 1 cờ).
- Xoá `App.export_report()`/`App.save_job()` — không còn call site sau khi logic chuyển vào worker (tương tự cách xử lý code chết ở Phase 4).
- `closeEvent`: chờ `scan_worker.wait(5000)` trước khi kill ROS node; hết 5s thì log cảnh báo và thoát tiếp (không treo).
- 2 sửa giảm lag render:
  - `data_converter.py::o3d_to_vtk_polydata`: thêm tham số `max_points` (mặc định `None` — không đổi hành vi các caller khác), subsample đều khi vượt ngưỡng. Chỉ áp dụng ở `CloudPipelineService.to_vtk()` (ngưỡng 2,000,000 điểm) — **`save_ply`/`ReportService.export` dùng cloud gốc không bị cắt**, đúng yêu cầu không giảm chất lượng lưu/report.
  - `vtk_viewer.py::_enable_box_widget`: tạo `vtkBoxWidget` + observer **một lần duy nhất**, các lần sau chỉ `SetProp3D`+`PlaceWidget()`+`On()`. Thêm None-check cho `self.current_actor` trong observer callback (phòng vệ nhẹ, không đổi hành vi khi actor luôn tồn tại như thực tế vận hành).
- `py_compile` pass cho cả 5 file sửa/mới.

### Chưa kiểm chứng được (không có ROS/Qt/Open3D/VTK trong sandbox)
- **Tiêu chí thành công chính của cả kế hoạch** — "UI không đơ trong lúc xuất report" — **chưa xác nhận bằng cách chạy app thật**. Đây là phase quan trọng nhất, cần ưu tiên chạy thử trong Docker trước khi tin tưởng.
- Chưa test: 2 scan liên tiếp nhanh (kiểm tra hàng đợi 1 chỗ hoạt động đúng, không crash), đóng app khi đang xuất report (kiểm tra `closeEvent` timeout), so sánh 1 file PDF trước/sau (đảm bảo không đổi chất lượng report).
- Chưa xác nhận trực quan box widget không bị giật khi tương tác nhiều lần liên tiếp.

## Ghi chú phát sinh

1. **Rủi ro race điều kiện đã được xử lý chủ động**: `isManualCompare`/`current_post_scan_path` được snapshot vào `job` dict ngay tại thời điểm `submit()` (GUI thread, đồng bộ) thay vì để worker đọc lại `self.*` — tránh trường hợp người dùng bấm "Compare" lần 2 trong lúc worker vẫn đang xử lý lần 1 làm lệch dữ liệu. Đây là điểm thiết kế quan trọng, khác một chút so với việc "chỉ di chuyển nguyên xi" nhưng bắt buộc phải làm để threading an toàn — đã ghi rõ trong code comment.
2. Vì exception trong `_process()` giờ được bắt bởi `try/except` bao ngoài trong `run()` (thêm mới, cần thiết cho threading) thay vì bởi `try/except` cũ nằm ngay trong `on_cloud_received`, một số lỗi trước đây chỉ `print` ra console giờ sẽ **hiện luôn lên NotificationCenter** (qua `notify` signal) — đây là cải thiện tự nhiên đi kèm, không phải mục tiêu chính nhưng có lợi, phù hợp hướng đi Phase 3.
3. Không tạo `ProcessedCloud`/wrapper gì thêm ngoài dict `job`/`metadata` đơn giản — giữ tối thiểu đúng tinh thần "không thêm trừu tượng khi chưa cần".
4. Việc `PROJECT_DIR`/`THICKNESS_DEFAULT`/`TOLERANCE_DEFAULT`/6 topic string được truyền qua constructor thay vì worker tự tính lại từ `cfg` — chủ đích để `app.py` vẫn là nguồn duy nhất định nghĩa các hằng số này (matching quyết định đã đưa ra ở Phase 4 khi tách service), tránh 2 nơi có thể lệch nhau nếu `cfg` đổi cấu trúc sau này.
