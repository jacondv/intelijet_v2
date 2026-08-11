# Kế hoạch chi tiết tối ưu Intelijet v2 — INDEX

> Bộ tài liệu này chia toàn bộ công việc tối ưu thành **9 phase**, mỗi phase được thiết kế để một phiên làm việc của Claude Sonnet hoàn thành trọn vẹn (kể cả khi bị giới hạn token). Mỗi file phase là **tự chứa** (self-contained): có đủ hiện trạng, danh sách file, các bước, tiêu chí nghiệm thu — Sonnet KHÔNG cần đọc lại toàn bộ dự án, chỉ cần đọc file phase được giao + các file code nêu trong đó.
>
> Kế hoạch tổng quát (bối cảnh, lý do): xem `docs/optimization_plan_overview.md`.
> Ngày tạo: 2026-08-09.

---

## Quy tắc chung cho MỌI phiên làm việc (Sonnet đọc trước tiên)

1. **Chỉ làm đúng phase được giao.** Không sửa lan sang phạm vi của phase khác, kể cả khi thấy "tiện tay". Nếu phát hiện vấn đề ngoài phạm vi, ghi vào mục "Ghi chú phát sinh" cuối file phase rồi tiếp tục.
2. **Mục tiêu cốt lõi là ỔN ĐỊNH.** Khi phân vân giữa cách "gọn nhưng đổi nhiều" và cách "dài hơn nhưng đổi ít, dễ kiểm chứng" → chọn cách thứ hai. Không đổi hành vi người dùng nhìn thấy trừ khi phase yêu cầu rõ.
3. **Trước khi xoá bất kỳ file/hàm nào**: grep toàn repo (kể cả `*.launch`, `setup.py`, `CMakeLists.txt`, script `.sh`) xác nhận không còn tham chiếu. Kết quả grep phải ghi lại trong báo cáo cuối phiên.
4. **Kiểm chứng tối thiểu sau mỗi phase** (môi trường dev có thể không có ROS đầy đủ — làm được đến đâu ghi rõ đến đó):
   - `python3 -m py_compile <mọi file .py đã sửa>` phải pass.
   - Import-check: `python3 -c "import <module>"` cho các module sửa (với `PYTHONPATH` trỏ tới `intelijet_v2_ws/src/<pkg>/src`). Lưu ý: các import `rospy`/`rospkg` sẽ fail nếu máy không có ROS — khi đó chỉ cần py_compile và soát tay.
   - Nếu có môi trường ROS/Docker: `catkin_make` (hoặc `catkin build`) trong `intelijet_v2_ws` phải pass.
5. **Commit theo phase**: mỗi phase xong = 1 commit (hoặc vài commit nhỏ theo bước), message tiếng Anh ngắn gọn, có tiền tố `[P<n>]`, ví dụ `[P1] Remove dead code in pps/shared/ui`. Không commit khi py_compile fail.
6. **Cập nhật trạng thái**: cuối phiên, sửa bảng "Trạng thái" bên dưới trong file này (đổi ⬜ thành ✅ hoặc 🟡 nếu dở dang, kèm 1 dòng ghi chú), và điền mục "Báo cáo hoàn thành" ở cuối file phase.
7. **Nếu hết token giữa chừng**: ưu tiên đưa code về trạng thái chạy được (dù chưa xong hết bước), commit với `[P<n>][WIP]`, cập nhật trạng thái 🟡 và ghi rõ đã xong bước nào / còn bước nào.
8. **Ngôn ngữ**: code, tên biến, commit message — tiếng Anh. Comment mới trong code — tiếng Anh. Báo cáo cho người dùng — tiếng Việt.
9. **Không nâng cấp thư viện / đổi version** trong bất kỳ phase nào trừ khi phase yêu cầu rõ (môi trường ROS Noetic + Python 3.8, PyQt5, Open3D 0.13–0.17 — nhạy cảm với version).

## Bối cảnh kỹ thuật tối thiểu (đọc 1 lần)

- ROS1 Noetic, Python 3, chạy trong Docker trên Linux, màn hình cảm ứng (kiosk).
- 3 package chính trong `intelijet_v2_ws/src/`:
  - `pps` — backend: điều khiển scan (housing + SickScan), pipeline so sánh point cloud (Open3D), action server.
  - `shared` — thư viện dùng chung: `config_loader.py` (config YAML → `CONFIG` global), `device_monitor.py` (giám sát thiết bị), `pps_command.py` (enum lệnh), `log_status.py` (đẩy thông báo qua /rosout).
  - `ui` — PyQt5 + VTK: cửa sổ chính `scripts/app.py` (class `App`), luồng ROS nền `scripts/ros_thread.py`, viewer `scripts/vtk_viewer.py`, xuất PDF `src/ui/tunnel_report/` (WeasyPrint).
- Entry point thật của UI là `ui/scripts/app.py` (KHÔNG phải `main.py` — file đó là đồ cũ, sẽ xoá ở Phase 1).
- Luồng dữ liệu chính: HMI/UI gửi lệnh (enum `PPSCommand`) → `pps/scripts/hmi_scan_command_handler.py` → `SickScanErobController` quét → publish cloud → `compare_cloud_action_server.py` so sánh → UI nhận cloud qua `ros_thread.py` → hiển thị VTK + xuất report.

## Thứ tự phase & phụ thuộc

| # | File | Nội dung | Phụ thuộc | Rủi ro |
|---|------|----------|-----------|--------|
| P1 | `phase_01_dead_code.md` | Xoá code chết toàn dự án (~4000 dòng) | — | Thấp |
| P2 | `phase_02_device_monitor.md` | Gộp monitor CAN, Lidar chuyển sang ping IP, bỏ eval() | P1 | Thấp |
| P3 | `phase_03_alarm_ui.md` | Hệ thống cảnh báo/alarm chuyên nghiệp trên UI | P2 | Vừa |
| P4 | `phase_04_ui_service_layer.md` | Tách business logic khỏi god-object `App` (chưa threading) | P1 | Vừa |
| P5 | `phase_05_threading.md` | Đưa pipeline cloud + report vào worker thread; hết đơ UI | P4 | Vừa-cao |
| P6 | `phase_06_pps_cleanup.md` | Tách `helper.py`, hợp nhất API convert, chuẩn hoá lỗi trong `pps` | P1 | Vừa |
| P7 | `phase_07_scanner_abstraction.md` | Trừu tượng hoá scanner (chuẩn bị thay SickScan bằng scanner 3D) | P6 | Vừa |
| P8 | `phase_08_docker_compose.md` | Docker Compose, icon desktop, nút Exit | — (độc lập) | Thấp |
| P9 | `phase_09_ux_polish.md` | Giảm dialog xác nhận, sửa các điểm khó thao tác | P3, P5 | Thấp |

Ghi chú thứ tự:
- P1 làm **đầu tiên** (khác kế hoạch sơ bộ): giảm ~4000 dòng nhiễu trước khi đụng vào phần khó, giúp các phiên sau đỡ tốn token và đỡ nhầm lẫn giữa code sống/chết.
- P2→P3 và P4→P5 là hai chuỗi bắt buộc đúng thứ tự. P8 độc lập, có thể chen bất cứ lúc nào.
- P6→P7: interface scanner (P7) đã được thiết kế sẵn trong file phase — P6 chỉ dọn dẹp, không cần biết trước P7.

## Trạng thái

| Phase | Trạng thái | Ghi chú |
|-------|-----------|---------|
| P1 | ✅ Xong | Xoá 10 file chết (~4271 dòng) + dọn tham chiếu chết. Commit `761071c` (và các file bị xoá nằm lẫn trong commit `664290e` do lỗi staging — nội dung đúng, chỉ lệch message). py_compile pass. |
| P2 | ✅ Xong | Gộp TopicAliveMonitor/PingMonitor/StateEchoMonitor, bỏ eval(), sửa bug update_status, cập nhật devices.yaml (lidar ip=192.168.82.121), thêm iputils-ping vào Dockerfile. Việc 5 (ros_thread.py/app.py) đã soát — không cần sửa. Commit `9969317`, `2c6370e`, `e348971`. Chưa chạy test thật trong ROS/Docker — xem Ghi chú phát sinh trong phase_02. |
| P3 | ✅ Xong (code) | NotificationCenter (lịch sử+ghim+chống spam), set_device_label, dialog lịch sử, log_status có level. Commit `44b8abb`. **Chưa chạy được test thật** (không cài được PyQt5 trong sandbox) — chỉ py_compile. Ưu tiên chạy `test_notification_center.py` trong Docker trước khi tin tưởng hoàn toàn. |
| P4 | ✅ Xong | JobStore/CloudPipelineService/ReportService tách khỏi `App`. Commit `f045736`. Test JobStore chạy thật, pass 7/7 (service duy nhất không cần ROS/Qt). CloudPipelineService/ReportService mới chỉ py_compile — cần Docker để chạy 1 chu trình prescan→postscan→compare→report thật. |
| P5 | ✅ Xong (code), ⚠️ CHƯA test thật | ScanPipelineWorker (hàng đợi 1 chỗ) đưa convert/color/VTK/save/report ra khỏi GUI thread; box widget + max_points giảm lag render. Commit `445696c`. **Đây là phase quan trọng nhất — bắt buộc chạy thử trong Docker trước khi tin tưởng** (chưa có ROS/Qt/Open3D trong sandbox để tự kiểm chứng "UI không đơ khi xuất report"). |
| P6 | ✅ Xong (code), ⚠️ CHƯA import/run thật | Việc 3 (6 điểm lỗi/anti-pattern) + Việc 1+2 (helper.py 1688 dòng → shim 55 dòng + pps/cloud_utils/*, xoá 19 hàm chết, hợp nhất convert vào CloudConverter). Commit `f462b42`, `29ea074`. Không có numpy/scipy/ROS trong sandbox nên chỉ kiểm chứng bằng py_compile + soát tay grep call-site kỹ. |
| P7 | ✅ Xong (code), ⚠️ CHƯA chạy prescan/postscan thật | `ScanStrategy` interface + `Sick2DAssembleStrategy` (di chuyển thuần, đối chiếu diff khớp 100%) + `Direct3DScanStrategy` skeleton. `hmi_scan_command_handler.py` không đổi giao diện. Commit `78d8daa`. Quyết định có chủ đích: KHÔNG thêm try/finally ép đóng housing (giữ đúng hành vi cơ khí gốc) — xem Ghi chú phát sinh #1 trong phase_07. |
| P8 | 🟡 Một phần | Việc 1-3 xong (chưa test thật trên máy Linux có Docker): `docker-compose.yml` chính thức, `run_docker.sh` launcher mỏng, `install.sh` mới (tự cài Docker + build image 1 lần + sinh icon desktop đúng đường dẫn máy đó), `docs/INSTALL.md`. Lệch có chủ đích so với plan gốc: icon bấm lần nào cũng `down` rồi `up -d` lại (restart sạch), KHÔNG tái sử dụng container đang chạy như Việc 2 gốc đề xuất — quyết định của người dùng. Việc 4 (nút Exit tắt toàn bộ container từ trong app) CHƯA làm — ngoài phạm vi yêu cầu lần này. |
| P9 | ⬜ Chưa làm | |

## Quyết định đã chốt (Sonnet không cần hỏi lại)

- **Nút Exit**: mặc định tắt **toàn bộ** — app → ROS nodes → container (ưu tiên khởi động sạch cho lần sau, phù hợp mục tiêu ổn định). Nếu người dùng đổi ý sẽ cập nhật ở đây.
- **Lidar connected** = ping ICMP tới IP cấu hình thành công (thêm field `ip` vào `devices.yaml`).
- **CAN connected** = còn nhận `can_msgs/Frame` trên topic tương ứng trong `timeout` giây (giữ nguyên tiêu chí hiện tại).
- **Không tích hợp driver BLK360G2 thật** trong đợt này — chỉ chuẩn bị interface (P7).
- Cách hiển thị trạng thái thiết bị: chuẩn hoá 3 mức `OK / WARNING / ERROR` với màu xanh/vàng/đỏ (chi tiết trong P3).
