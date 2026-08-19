# Phase 11 — Bổ sung log lỗi thiếu trong PPS + Tab Diagnostics/Alarm (HMI-style)

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P3 (`NotificationCenter`), P10 (`shared.notify`/`Notification.msg`) — cả hai đã xong. Rủi ro: Thấp-Vừa.
> Phạm vi file: 7 file trong `pps/` (Phần A) + `ui/scripts/app.py`, `ui/src/ui/diagnostics_tab.py` (mới), `ui/src/ui/tests/test_diagnostics_tab.py` (mới) (Phần B). Không đổi `.msg`/`CMakeLists.txt` nào.

Branch: `feature/alarm`. Phát sinh từ yêu cầu trực tiếp của người dùng: cần 1 "alarm window" để xem/chẩn đoán lỗi cho app, kiểu HMI công nghiệp. Đây đúng là hạng mục "Chưa làm" đã ghi sẵn trong `docs/plan/phase_10_notification_channel.md` (tab Debug/Diagnostics không-modal).

## Hiện trạng (trước phase này)

Hạ tầng notification (`Notification.msg`, `shared/notify.py`, `NotificationCenter`, `StatusBinder`) đã có từ P3/P10, nhưng:
- Nhiều nhánh lỗi trong `pps/` chưa gọi `notify()` nên không lên được UI — chỉ nằm trong `rospy.logerr`/log file.
- 1 bug thật trong `compare_cloud_base_client.py`: message `notify()` luôn hardcode tiền tố `"[INFO]"` bất kể `result.success`/timeout, khiến compare fail vẫn hiện màu xanh info trên UI.
- 1 bug im lặng khác trong `sick_2d_assemble.py`: khi service `assemble_scans2` lỗi, scan "coi như xong" (chuyển IDLE) dù không có cloud thật.
- UI chỉ có 1 label + 1 dialog lịch sử phẳng, chưa có màn hình dạng bảng log alarm như HMI.

Quyết định đã chốt với người dùng (không hỏi lại):
- Alarm window là **1 tab cố định** trong màn hình chính (không phải popup/dialog).
- **Không** cần cơ chế acknowledge/clear kiểu ISA-18.2 — chỉ cần bảng log lỗi theo thời gian, lọc được theo level/nguồn: cột Timestamp, Level, Source, Message.

## Thiết kế đích

### Phần A — Bổ sung `notify()` còn thiếu trong PPS

Convention: khi biết chắc severity trong 1 nhánh except, truyền `level=` tường minh cho `notify()` (mẫu đã có ở `compare_pipeline.py:109-112`), không dựa vào suy luận `[WARN]`/`[ERROR]` từ chuỗi.

1. `pps/src/pps/generic_scan_controller.py` — `_scan_thread`/`_move_housing_thread`: thêm `notify(level="error")` trong except, cạnh `rospy.logerr` đã có.
2. `pps/src/pps/scan_strategies/sick_2d_assemble.py` — `Sick2DAssembleStrategy.acquire()`: khi `assemble_cloud_client()` trả `None`, thêm `notify(level="error")` + set `status_callback(*_ERROR)`; **guard** đoạn set `IDLE` cuối hàm bằng `and not assemble_failed` để trạng thái lỗi không bị ghi đè khi housing đóng thành công sau đó.
3. `pps/scripts/compare_cloud_action_server.py` + `compare_cloud_action_manual_server.py` — thêm import `notify`, gọi `notify(level="error")` trong except cạnh `set_aborted()`.
4. `pps/src/pps/cloud_compare/compare_cloud_base_client.py` — BUG: `_on_done()` và `_check_timeout()` sửa để `level`/prefix message phản ánh đúng `result.success`/timeout thay vì hardcode `[INFO]`.
5. `pps/scripts/dv_cloud_preprocess.py` — bọc try/except quanh `process_cloud()` trong `callback_pres`/`callback_post`, thêm `notify(level="error")`.
6. `pps/scripts/hmi_scan_command_handler.py` — `get_scanner_controller()`: `notify(level="error")` trước khi `raise ValueError`.

### Phần B — Tab Diagnostics trong UI chính

- File mới `ui/src/ui/diagnostics_tab.py`: class `DiagnosticsTab(QWidget)` — `QTableWidget` 4 cột (Timestamp/Level/Source/Message), filter theo level (`QComboBox`), nguồn (`QComboBox`, tự thêm nguồn mới), tìm theo message (`QLineEdit`). Nạp toàn bộ `notification_center.history()` khi tạo, nối `notification_added` signal để cập nhật realtime. Tô màu dòng theo `LEVEL_COLORS` (tái dùng từ `ui.notification_center`). Không có ack/clear.
- `ui/scripts/app.py`: thêm hằng `DIAGNOSTICS_HISTORY_CAP = 500`, truyền vào `NotificationCenter(max_history=...)` (dùng chung cho label/dialog/tab mới). Thêm `self.diagnostics_tab = DiagnosticsTab(...)` và `self.ui.tab_mainview.addTab(self.diagnostics_tab, "DIAGNOSTICS")` — tab thứ 5, không sửa file `.ui`/`_ui.py` generated (đúng ràng buộc từ P3).
- `tab_system` (SystemStatus hiện có) **không** gộp vào tab Diagnostics mới — tách riêng theo đúng yêu cầu đã chốt (chỉ log).

## Ràng buộc

- Không implement acknowledge/clear kiểu ISA-18.2, không đổi kiến trúc `DeviceStatus`.
- Không sửa file `*_ui.py` generated và file `.ui` Qt Designer.
- Không mở rộng phạm vi ra các nhánh `set_aborted()` khác chưa được liệt kê ban đầu (ghi lại trong "Ghi chú phát sinh").

## Kiểm chứng

Môi trường phiên này có sẵn **ROS Noetic + roscore + PyQt5 + pytest + open3d + ros_numpy** — đã tận dụng để verify thật, không chỉ `py_compile`:

1. `python3 -m py_compile` toàn bộ 10 file sửa/mới — pass.
2. **UI (Phần B)**: `python3 -m pytest ui/src/ui/tests/` — 17/17 pass (5 test mới `test_diagnostics_tab.py` + 12 test cũ không bị ảnh hưởng: `test_notification_center.py`, `test_status_binder.py`, `test_system_status.py`).
3. **PPS (Phần A) — verify thật qua roscore** (script tạm trong scratchpad, không commit vào repo):
   - `Sick2DAssembleStrategy.acquire()` với `assemble_cloud_client` giả lập trả `None`: xác nhận `notify(level="error")` phát đúng + `status_callback` cuối cùng vẫn là `PRESCAN_ERROR` (không bị ghi đè về `IDLE`).
   - `CompareBaseClient._on_done(success=True)` → level `info`; `_on_done(success=False)` → level `error` (trước đây luôn `info` — bug đã xác nhận sửa đúng).
   - `CompareBaseClient._check_timeout()` (giả lập quá timeout) → level `error` (trước đây luôn `info`).
   - `GenericScanController._scan_thread`/`_move_housing_thread` với strategy/housing giả lập ném exception → `notify(level="error")` phát đúng message.
   - `CloudProcessorNode.callback_pres`/`callback_post` với `process_cloud` giả lập ném exception → không crash, `notify(level="error")` phát đúng.
   - `get_scanner_controller(active_lidar="bogus_lidar")` → `notify(level="error")` phát trước khi `ValueError` raise.
4. Import-check thật (không chỉ `py_compile`) cho `generic_scan_controller`, `dv_cloud_preprocess`, `hmi_scan_command_handler` — tất cả import sạch với `rospy`/`open3d`/`ros_numpy` thật.

## Tiêu chí nghiệm thu

- [x] 6 điểm thiếu `notify()` trong Phần A đã bổ sung, verify thật qua roscore.
- [x] Bug severity hardcode `[INFO]` trong `compare_cloud_base_client.py` đã sửa, verify thật.
- [x] Bug trạng thái `*_ERROR` bị ghi đè về `IDLE` trong `sick_2d_assemble.py` đã sửa, verify thật.
- [x] Tab "DIAGNOSTICS" hiển thị đúng lịch sử + filter level/source, verify qua pytest thật (PyQt5).
- [x] Không đổi 4 tab hiện có, không sửa file `.ui`/`_ui.py`.
- [x] Commit riêng cho Phần A và Phần B.
- [x] Cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

**Trạng thái: ✅ Xong (code + verify thật qua roscore/pytest).**

### Đã làm
- Phần A: 6 điểm bổ sung/sửa `notify()` trong `pps/` như thiết kế, verify thật qua roscore (script tạm, không commit).
- Phần B: `ui/src/ui/diagnostics_tab.py` (mới), sửa `ui/scripts/app.py` (constant `DIAGNOSTICS_HISTORY_CAP`, tạo `NotificationCenter(max_history=...)`, thêm tab "DIAGNOSTICS"), `ui/src/ui/tests/test_diagnostics_tab.py` (mới, 5 test case).
- Toàn bộ test suite UI hiện có (17 test) pass thật qua pytest + PyQt5 thật trong sandbox.

### Chưa kiểm chứng được / cần làm khi deploy thật
- Chưa chạy được prescan/postscan/compare **thật trên phần cứng** (housing/PLC/lidar thật) — mọi verify Phần A dùng object giả lập (fake controller/housing/publisher) để cô lập đúng nhánh except/logic cần kiểm chứng, không phải toàn bộ pipeline thật.
- Chưa mở app thật (`app.py`) trong Docker/kiosk để xác nhận trực quan tab "DIAGNOSTICS" hiển thị đúng, filter hoạt động mượt trên màn hình cảm ứng thật, và performance khi có nhiều bản ghi (tới 500).
- Khuyến nghị: chạy 1 chu trình đầy đủ (login → prescan → postscan → compare → xem report) trên Docker/kiosk thật trước khi merge `feature/alarm`, đặc biệt để xác nhận A2 (bug status không bị ghi đè) và A4 (compare fail hiện đúng màu đỏ + ghim 10s) trong luồng nghiệp vụ thật.

## Ghi chú phát sinh

1. `compare_cloud_action_server.py`/`compare_cloud_action_manual_server.py` còn vài nhánh `set_aborted(..., "Timeout waiting for ...")`/`"No cloud"`/`"Missing file"` cũng thiếu `notify()` — không nằm trong phạm vi liệt kê ban đầu, không sửa trong P11, ghi lại cho phase sau nếu cần.
2. Không gộp `tab_system` (SystemStatus) vào tab Diagnostics mới dù `phase_10` từng gợi ý ý tưởng đó — quyết định lần này của người dùng là tách riêng (chỉ log), khác gợi ý cũ, ghi rõ để tránh hiểu nhầm là bỏ sót.
3. Môi trường sandbox phiên này khác hẳn tiền lệ P1-P10 (không có `rospy`/`PyQt5`) — lần đầu tiên có đủ ROS Noetic + roscore + PyQt5 + open3d + ros_numpy để verify thật thay vì chỉ `py_compile`. Ghi lại để các phase sau biết có thể kỳ vọng verify sâu hơn nếu môi trường tương tự.
