# Phase 10 — Kênh Notification riêng + SystemStatus đánh kiểu + StatusBinder hợp nhất

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P2 (`device_monitor.py`), P3 (`NotificationCenter`, `set_device_label`). Rủi ro: Vừa (đụng cả `pps/`, `shared/`, `ui/`, và định nghĩa message ROS mới).
> Phạm vi file: `shared/msg/Notification.msg` (mới), `shared/src/shared/notify.py` (mới), `shared/src/shared/log_status.py`, `shared/src/shared/device_monitor.py`, `shared/CMakeLists.txt`, `config/commond.yaml`, `config/last_used.yaml`, `ui/scripts/ros_thread.py`, `ui/scripts/app.py`, `ui/scripts/data_binder.py` (xoá), `ui/src/ui/update_data_utils.py`, `ui/src/ui/status_binder.py` (mới), `ui/src/ui/system_status.py` (mới), 4 file trong `pps/` gọi `log_status(name=cfg.NOTIFICATION, ...)`.

Phase này khác các phase P1-P9 ở chỗ nó **không nằm trong kế hoạch gốc** (`optimization_plan_overview.md`, 2026-08-09) — phát sinh từ yêu cầu trực tiếp của người dùng trong phiên làm việc, khi thấy luồng cập nhật trạng thái ROS→UI "khá rối". Ghi lại ở đây để giữ đúng quy ước tài liệu hoá của dự án, dùng branch `dev/v3.2`.

## Hiện trạng (trước phase này)

Sau P3, `NotificationCenter` (tầng hiển thị) đã sạch, nhưng **đường truyền phía dưới nó** vẫn rối:

1. **Hack `/rosout`-JSON**: `log_status()` (`shared/log_status.py`) JSON-encode một dict rồi gọi `rospy.logerr/logwarn/loginfo(json.dumps(...))` để "gửi" thông báo — mượn kênh debug-log của ROS. `ui/scripts/ros_thread.py` subscribe **toàn bộ** topic `/rosout` (log của mọi node trong hệ thống), `unpack_log_status()` thử `json.loads()` từng dòng log, dòng nào không phải JSON thì âm thầm bỏ qua. `log_status(name=cfg.NOTIFICATION, ...)` được gọi thật ở **16 chỗ** trong 4 file `pps/` (mở/đóng housing, scan completed, job cancel, compare done/failed...) — không phải code chết.
2. **`ui_data_update = pyqtSignal(dict)`**: `RosThread` gom mọi state (encoder, trạng thái 5 thiết bị, notification) vào 1 dict `data_store` dùng chung, emit nguyên khối 1Hz. `App.update_data()` unpack bằng chuỗi `if "key" in data:` — không có schema, thêm 1 field mới phải sửa cả 2 đầu (producer/consumer) một cách tự phát.
3. **3 cơ chế push widget không đồng bộ**: `app.py:update_data()` push cứng `lblEncoder`/`lblEncoderRawValue`/4 label thiết bị; `DataBinder` (`update_data_utils.py`) chỉ xử lý riêng nút bấm theo trạng thái `pps` bằng if/elif 60 dòng; `config_mapping` (cùng file) là bảng khai báo `{objectName: {get, set}}` nhưng chỉ dùng cho tab Setting, không áp dụng cho trạng thái ROS sống.
4. **Phát hiện thay đổi trạng thái thiết bị nằm sai lớp**: `app.py` tự so `self._prev_device_state` mỗi tick UI để biết thiết bị vừa đổi trạng thái rồi mới bắn notification — lẽ ra sự kiện "thiết bị X mất kết nối" phải phát sinh ngay tại nguồn (`device_monitor.py`), không phải suy luận lại ở tầng UI.
5. **Gotcha config đã xác minh**: `shared/config_loader.py` chỉ đọc `commond.yaml`/`runtime.yaml` khi `config/last_used.yaml` **chưa tồn tại**; nếu đã tồn tại (trường hợp thực tế), chỉ đọc `last_used.yaml`. Thêm key mới vào `commond.yaml` mà quên thêm vào `last_used.yaml` sẽ gây `AttributeError` khi chạy trên máy đã có sẵn file này.

## Thiết kế đích

- **Notification = message ROS thật** (`shared/msg/Notification.msg`: `source`, `level` (string "info"/"warning"/"error"), `message`, `node`, `stamp`) trên topic riêng `cfg.NOTIFICATION_TOPIC` — vì đây là giao tiếp **liên tiến trình** thật sự (`pps/` → `ui/`), không thể thay bằng object Python thuần. `log_status.py`/`log_status()` **giữ nguyên**, chỉ dùng cho debug log thật (rqt_console) — tách bạch hoàn toàn khỏi khái niệm "notification".
- **SystemStatus (encoder + trạng thái thiết bị) = dataclass Python thuần**, KHÔNG tạo message ROS mới — vì `StatusReader` và subscriber encoder đã chạy **cùng tiến trình** với UI (trong `RosThread`), không có consumer nào khác qua mạng cần dữ liệu này. `ui_data_update` đổi từ `pyqtSignal(dict)` → `pyqtSignal(object)` mang `SystemStatus`.
- **Phát hiện chuyển trạng thái thiết bị chuyển xuống `Monitor.update_status()`** (`device_monitor.py`) qua callback `on_transition(name, old, new)`, chỉ gọi khi trạng thái thật sự đổi (bỏ qua lần quan sát đầu tiên, giữ đúng hành vi cũ của `_prev_device_state`). `StatusReader` nối callback này vào `notify()`.
- **Hợp nhất 3 cơ chế push widget thành 1**: `ui/src/ui/status_binder.py` — `StatusBinder` (tái dùng cách cache widget theo `objectName()` của `DataBinder` cũ) + 2 bảng khai báo `STATUS_BINDINGS` (label/text ← `SystemStatus`) và `PPS_BUTTON_STAGE_TABLE` (trạng thái `pps` → nút nào Active/Error). `config_mapping` (tab Setting) giữ nguyên, không đụng.

## Triển khai theo 3 sub-phase (mỗi sub-phase tự chạy được, có thể dừng an toàn giữa chừng)

1. **Phase 0+1 — Kênh Notification**: thêm `Notification.msg`, `shared/notify.py`, migrate 16 call site trong `pps/`, subscriber mới trong `ros_thread.py` chạy **song song** với `/rosout` cũ để không có cửa sổ "mất hết notification", verify xong mới gỡ `/rosout`.
2. **Phase 2 — SystemStatus đánh kiểu**: `ui/src/ui/system_status.py` (dataclass, không import ROS/Qt — test được như `NotificationCenter`/`JobStore`), `Monitor.on_transition`, `ros_thread.py` build `SystemStatus` thay dict.
3. **Phase 3 — StatusBinder hợp nhất**: xoá `DataBinder`/`_update_control_button_style`/`set_control_button_stage`/`Status` khỏi `update_data_utils.py`, chuyển sang `status_binder.py`; `app.py:update_data()` còn lại đúng 1 dòng `self.status_binder.apply(status)`.

## Kiểm chứng

Môi trường làm việc phiên này **có sẵn ROS Noetic + ability chạy `roscore`** (khác các phase trước chỉ `py_compile` được) — đã tận dụng để kiểm chứng thật, không chỉ soát tay:

1. `catkin build shared` pass, `from shared.msg import Notification` import được.
2. Test end-to-end qua `roscore` thật: `notify()` → topic `/notification` → `RosThread.notification_callback` → Qt signal → nhận đúng `(source, message, level)`.
3. Test end-to-end: `RosThread` chạy thật → `ui_data_update` emit đúng `SystemStatus` với đủ 5 thiết bị (`lidar/encoder/pcan/pps/plc`).
4. Test end-to-end đầy đủ pipeline: ROS → `SystemStatus` → `StatusBinder.apply()` → widget Qt thật (`QLabel`/`QPushButton` trần), xác nhận text/style đúng.
5. Unit test mới: `shared/src/shared/tests/test_device_monitor.py` (hành vi `on_transition` — không bắn ở lần đầu, chỉ bắn khi thật sự đổi), `ui/src/ui/tests/test_system_status.py`, `ui/src/ui/tests/test_status_binder.py` (widget trần, không cần `QMainWindow`/rospy).
6. Toàn bộ suite hiện có (`test_notification_center.py`, `test_job_store.py`) + test mới: **22/22 pass**.

## Tiêu chí nghiệm thu

- [x] Không còn nơi nào subscribe `/rosout` hoặc gọi `unpack_log_status` (đã xoá hàm chết này).
- [x] Cả 16 call site `log_status(name=cfg.NOTIFICATION, ...)` trong `pps/` chuyển sang `notify(...)`, giữ nguyên message text + logic suy luận severity từ `[WARN]`/`[ERROR]`.
- [x] `ui_data_update` mang `SystemStatus` (object) thay vì `dict`; không còn `data_store` dùng chung trong `RosThread`.
- [x] `_prev_device_state` trong `app.py` đã xoá — phát hiện transition chuyển sang `device_monitor.py`.
- [x] `ui/scripts/data_binder.py` (stub chết 8 dòng) đã xoá.
- [x] `DataBinder`/if-elif 60 dòng đã thay bằng `StatusBinder` + 2 bảng khai báo.
- [x] Test mới + test cũ đều pass; có ít nhất 1 lượt verify qua `roscore` thật cho mỗi sub-phase.
- [x] Commit theo từng sub-phase, message tiếng Anh rõ ràng.
- [x] Cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

**Trạng thái: ✅ Xong (code + test thật qua roscore).** Branch `dev/v3.2`. Commit `750362b` (Phase 0+1), `eae49ae` (Phase 2), `925b1eb` (Phase 3).

### Đã làm — xem chi tiết trong message của từng commit
- `750362b`: `Notification.msg` + `shared/notify.py`; migrate 16 call site `pps/`; `ros_thread.py`/`app.py` có kênh notification đánh kiểu; gỡ `/rosout`/`unpack_log_status`; xoá `ui/scripts/data_binder.py` (dead stub).
- `eae49ae`: `ui/system_status.py` (`DeviceState`/`SystemStatus`); `Monitor.on_transition` (bỏ qua lần quan sát đầu); `StatusReader(on_transition=...)`; `ui_data_update` đổi kiểu; `update_data()` đọc attribute thay vì dict key.
- `925b1eb`: `ui/status_binder.py` (`StatusBinder`, `STATUS_BINDINGS`, `PPS_BUTTON_STAGE_TABLE`); dọn `update_data_utils.py` chỉ còn `config_mapping`/`load_config_to_ui`/`load_ui_to_config`.

### Khác biệt so với thiết kế ban đầu (không đáng kể, không đổi hành vi)
- Ban đầu định pin `kornia>=0.8.3` cho 1 việc không liên quan (matcher XFeat trong `ai_core_pkg`, từ đầu phiên làm việc) — phát hiện xung đột Python 3.8 nên đã revert, không liên quan phase này, ghi chú lại phòng nhầm lẫn khi đọc lịch sử commit branch `dev/v3.2`.

### Chưa kiểm chứng được / cần làm khi deploy thật
1. **`config/last_used.yaml` bị gitignore** (state runtime từng máy) — sửa cục bộ trong phiên làm việc để tự test được, nhưng **KHÔNG nằm trong commit**. Máy kiosk thật nếu đã có sẵn `last_used.yaml` cũ (chưa có `NOTIFICATION_TOPIC`) cần: (a) thêm thủ công dòng `NOTIFICATION_TOPIC: "/notification"`, hoặc (b) xoá file để hệ thống tự sinh lại từ `commond.yaml` (đã cập nhật, có key mới) — nếu không `cfg.NOTIFICATION_TOPIC` raise `AttributeError` ngay khi khởi động UI.
2. **Chưa chạy được prescan/postscan/compare thật trên phần cứng** — mọi verify trong phiên này dùng `roscore` trần + `notify()` gọi tay + `RosThread`/`StatusBinder` chạy thật nhưng không có node `pps` thật publish. Khuyến nghị chạy 1 chu trình đầy đủ (login → chọn job → prescan → postscan → compare → xem report) trên Docker/kiosk thật trước khi merge `dev/v3.2` vào `main`, đặc biệt để xác nhận cả 16 điểm gọi `notify()` trong `pps/` hoạt động đúng trong luồng nghiệp vụ thật (không chỉ đúng cú pháp).
3. **Cần `catkin build shared`** (hoặc `catkin_make`) trước khi chạy — đã verify build pass trong phiên này, nhưng image Docker triển khai thật cần rebuild để có `Notification.msg` mới sinh ra trong `devel/`.

### Chưa làm (để ngỏ, không thuộc phạm vi bắt buộc của phase này)
- **Phase 4** (đã ghi trong kế hoạch thảo luận với người dùng, chưa triển khai): dọn ~10 lambda gọi `ui_send_cmd_signal.emit(...)` rải rác trong `app.py.__init__` thành method đặt tên riêng; thêm tab Debug/Diagnostics không-modal hiển thị `SystemStatus` + lịch sử `NotificationCenter` đầy đủ (khác `NotificationHistoryDialog` hiện tại là dialog modal).
- **Gating prescan/postscan theo trạng thái sẵn sàng thiết bị** (Lidar/Encoder) — đã thảo luận với người dùng trước phase này nhưng chủ động để ngoài phạm vi; thiết kế `StatusBinder`/`SystemStatus.devices` hiện tại không cản trở việc thêm sau (vd thêm cờ `blocking: true` vào `devices.yaml` + `StatusReader.is_ready()`).

## Ghi chú phát sinh

1. `shared/src/shared/tests/` là thư mục test **mới** cho package `shared` (trước đây chỉ `ui/` có `tests/`) — **không thêm `__init__.py`** vào thư mục này: đã thử và phát hiện nó làm `pytest` (chế độ "prepend" import mode mặc định) đi ngược thư mục cha tìm root package, chèn `shared/src` lên đầu `sys.path` và **che mất** `shared.msg` (module sinh ra bởi `catkin`, chỉ tồn tại vật lý dưới `devel/lib/python3/dist-packages/shared/msg/`, không có trong `shared/src/shared/`). Nếu sau này thêm file test khác vào thư mục này, giữ nguyên không có `__init__.py`.
2. `Monitor.update_status()` được gọi từ callback ROS chạy trên thread nền (`rospy.Timer`/`rospy.Subscriber`), không phải thread UI — `_publish_device_transition` (`ros_thread.py`) gọi `notify()` trực tiếp từ thread đó; an toàn vì `rospy.Publisher.publish()` tự thread-safe, không cần khoá thêm.
3. `Notification.msg` dùng `level` kiểu `string` (không phải số như `type` cũ trong `log_status.py`) để tránh lặp lại việc định nghĩa bảng ánh xạ severity 2 lần độc lập (`_LEVEL_TO_TYPE` trong `log_status.py` vs `LEVEL_COLORS` trong `notification_center.py`) — `NotificationCenter.push()` nhận thẳng chuỗi "info"/"warning"/"error" từ `Notification.msg`, không cần chuyển đổi.
