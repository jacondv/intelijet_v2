# Phase 2 — Đơn giản hoá giám sát thiết bị (shared/device_monitor.py)

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P1 (đã dọn comment chết trong file này). Rủi ro: thấp.
> Phạm vi file: `shared/src/shared/device_monitor.py`, file config thiết bị `devices.yaml` (tìm trong `pps/config/` hoặc thư mục config mà `shared/config_loader.py` resolve tới), và điểm tiêu thụ duy nhất: `ui/scripts/ros_thread.py`, `ui/scripts/app.py` (chỉ đọc, xem Việc 5).

## Hiện trạng (đã khảo sát — tin cậy được, không cần khảo sát lại từ đầu)

`device_monitor.py` có:
- Class `Monitor` (base, ~dòng 45-81): đọc config 1 device, subscribe topic (import msg class động qua `importlib`), tạo `rospy.Timer` chu kỳ `timeout/2` gọi `check_status`. `handle_message`/`check_status` raise `NotImplementedError`.
- 4 subclass **giống hệt nhau từng byte**: `LidarMonitor`, `EncoderMonitor`, `PCANMonitor`, `PLCMonitor` — logic: nhận bất kỳ msg nào → `last_msg_time = now()`; timer check: quá `timeout` giây không có msg → `DISCONNECTED`, ngược lại `CONNECTED`.
- `PPSMonitor`: khác biệt — `device_state = msg.data` trực tiếp từ `/pps/state` (String), `check_status` là no-op → có thể kẹt trạng thái cũ vĩnh viễn nếu node pps chết (bug tiềm ẩn).
- `get_monitor_class()` (~dòng 145-150): dùng `eval(class_name)` để map chuỗi `type:` trong `devices.yaml` → class. Không an toàn.
- `StatusReader` (~dòng 152-192): singleton (`__new__`), đọc `devices.yaml`, tạo monitor cho từng device, expose `get_status()`.
- Bug: `Monitor.update_status(dev_state, proc_state, detail)` — mọi nơi chỉ truyền 1 tham số positional, nên `process_state`/`detail` luôn là `''`.

`devices.yaml` hiện có các entry dạng: `lidar` (topic `/cloud`, `sensor_msgs/PointCloud2`, timeout 3.0), `encoder` (`/encoder01/can_msg`, `can_msgs/Frame`), `pcan` (`/pcan_received_messanges`), `plc` (`/plc/heatbeat`), `pps` (`/pps/state`, `std_msgs/String`).

Người dùng chốt yêu cầu:
- **CAN connected** = còn nhận frame CAN trên topic trong `timeout` giây (giữ nguyên tiêu chí — chỉ gộp code).
- **Lidar connected** = **ping ICMP tới IP của lidar thành công** (đổi hành vi: không suy từ dữ liệu `/cloud` nữa — hiện lidar báo "mất kết nối" mỗi khi không quét dù thiết bị vẫn online).

## Thiết kế đích

Giữ nguyên: tên file, tên class `StatusReader`, chữ ký `get_status()`/`get_device(name)`, cấu trúc dict trạng thái trả về (UI đang đọc `data['<device>']['device_state']` — KHÔNG đổi key). Thay đổi bên trong:

```
Monitor (base)
 ├── TopicAliveMonitor   # thay cho Lidar/Encoder/PCAN/PLCMonitor cũ:
 │                       #   handle_message → last_msg_time = now()
 │                       #   check_status   → CONNECTED nếu (now - last_msg_time) <= timeout
 ├── PingMonitor         # MỚI: không subscribe topic; timer chạy ping ICMP tới cfg.ip
 │                       #   CONNECTED nếu ping thành công trong lần check gần nhất
 └── StateEchoMonitor    # thay PPSMonitor: device_state = msg.data, NHƯNG thêm staleness:
                         #   nếu quá timeout không nhận msg → DISCONNECTED (sửa bug kẹt trạng thái)
```

Chi tiết bắt buộc:
1. **Dispatch bằng dict**, không `eval()`:
   ```python
   MONITOR_CLASSES = {
       "TopicAliveMonitor": TopicAliveMonitor,
       "PingMonitor": PingMonitor,
       "StateEchoMonitor": StateEchoMonitor,
   }
   ```
   `type:` không có trong dict → `rospy.logerr` rõ ràng và **bỏ qua device đó** (không crash cả `StatusReader` — hiện nay 1 entry sai giết cả init).
2. **PingMonitor không được chặn thread**: chạy ping trong **thread nền riêng** (daemon) lặp mỗi `check_interval` giây, dùng `subprocess.run(["ping","-c","1","-W","1", ip], ...)` với timeout; timer ROS chỉ đọc kết quả gần nhất từ biến (có `threading.Lock` hoặc chỉ đọc/ghi biến bool đơn — an toàn với GIL). TUYỆT ĐỐI không gọi subprocess trong callback `rospy.Timer`.
   - Chống nhấp nháy: chỉ chuyển sang `DISCONNECTED` sau **2 lần ping fail liên tiếp** (1 lần fail đơn lẻ do mạng chập chờn không đổi trạng thái).
3. **Bảo vệ init**: bọc phần tạo từng monitor trong try/except — 1 entry cấu hình sai (msg_type import fail, thiếu field) chỉ log lỗi + bỏ qua device đó.
4. **Sửa `update_status`**: đổi thành chữ ký rõ ràng `update_status(self, dev_state, proc_state=None, detail=None)` — `None` nghĩa là giữ giá trị cũ (thay vì ghi đè thành `''`). Cập nhật mọi call site.
5. **Chặn timer quá dày**: chu kỳ check = `max(timeout / 2.0, 0.5)` giây.
6. `devices.yaml`:
   - Đổi `type:` các entry CAN (`encoder`, `pcan`, `plc`) → `TopicAliveMonitor`.
   - Entry `lidar`: đổi `type:` → `PingMonitor`, bỏ `topic`/`msg_type`, thêm `ip: <địa chỉ hiện tại của lidar>` — tìm IP thật trong config sẵn có (`lidar.yaml`, `commond.yaml`, hoặc launch file sick_scan có param `hostname`); nếu không tìm thấy, đặt placeholder `192.168.0.1` kèm comment `# TODO: set actual lidar IP` và ghi vào Ghi chú phát sinh.
   - Entry `pps`: `type: StateEchoMonitor`, giữ topic/timeout.
7. Docstring đầu file: 3–5 dòng mô tả 3 loại monitor và cách thêm device mới (đây là tài liệu duy nhất của cơ chế này).

## Việc 5 — Kiểm tra phía tiêu thụ (chỉ đọc, sửa tối thiểu)

`StatusReader` chỉ được dùng ở `ui/scripts/ros_thread.py` (~dòng 12, 40, 100) và `ui/scripts/app.py` (`update_data`, ~dòng 465-500). Xác nhận sau thay đổi:
- Key dict trạng thái (`lidar`, `encoder`, `pcan`, `plc`, `pps`) và field `device_state` không đổi → UI không cần sửa.
- Nếu buộc phải đổi gì đó ở phía UI, sửa tối thiểu và ghi rõ trong báo cáo.

## Kiểm chứng

1. `py_compile` + import-check `shared.device_monitor` (mock môi trường không ROS: chỉ cần py_compile nếu không có rospy).
2. Viết test thủ công nhỏ (nếu có ROS): chạy `StatusReader`, rút mạng lidar/tắt node → xác nhận trạng thái đổi đúng, ping fail 1 lần chưa đổi trạng thái, 2 lần mới đổi.
3. Xác nhận không còn `eval(` trong file.

## Tiêu chí nghiệm thu

- [ ] 4 class trùng lặp đã gộp còn `TopicAliveMonitor`; `PingMonitor` mới hoạt động không chặn thread; `StateEchoMonitor` có staleness check.
- [ ] `eval()` đã thay bằng dict; entry config sai không làm crash `StatusReader`.
- [ ] Bug `update_status` đã sửa.
- [ ] `devices.yaml` cập nhật; UI (`ros_thread.py`/`app.py`) không phải sửa (hoặc sửa tối thiểu, có ghi chú).
- [ ] Commit `[P2] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

_(chưa có)_

## Ghi chú phát sinh

_(chưa có)_
