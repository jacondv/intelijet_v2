# Phase 1 — Xoá code chết toàn dự án

> Đọc `docs/plan/00_INDEX.md` (mục "Quy tắc chung") trước khi bắt đầu.
> Phụ thuộc: không. Rủi ro: thấp. Mục tiêu: giảm ~4000 dòng code chết để các phase sau làm việc trên codebase gọn, không nhầm code sống/chết.

## Nguyên tắc

- **Chỉ xoá, không refactor.** Không đổi tên, không di chuyển, không "tiện tay sửa". Hành vi runtime phải giữ nguyên 100%.
- Trước khi xoá TỪNG file: grep toàn repo tên module/file đó (kể cả `*.launch`, `setup.py`, `CMakeLists.txt`, `package.xml`, `*.sh`) để xác nhận không còn tham chiếu sống (tham chiếu trong dòng comment thì được phép xoá luôn dòng comment đó). Ghi kết quả grep vào báo cáo.
- Nếu một file trong danh sách hoá ra VẪN được tham chiếu ở đâu đó → KHÔNG xoá, ghi vào "Ghi chú phát sinh" và bỏ qua.

## Việc 1 — Xoá file chết

### Package `pps` (`intelijet_v2_ws/src/pps/`)
| File | Lý do đã khảo sát |
|------|-------------------|
| `src/pps/image_processing/keypoint_processing.py` (~1093 dòng) | Chỉ bản `_v3` được import (`cloud_processing/compare_pipeline.py`) |
| `src/pps/image_processing/keypoint_processing_v2.py` (~934 dòng) | Như trên |
| `src/pps/sick_scan_controller.py` (~211 dòng) | Đã thay bằng `sick_scan_eRob_controller.py`; nơi gọi duy nhất là dòng comment trong `scripts/hmi_scan_command_handler.py` |
| `scripts/compare_cloud_action_server_old.py` (~284 dòng) | Không xuất hiện trong launch file nào |
| `scripts/scan_to_cloud_msg_node.py` (~48 dòng) | Đã comment trong `launch/pps.launch` dòng 5 — xoá cả dòng comment đó trong launch |
| `src/pps/cloud_compare/compare_method_m3c2.py` (~234 dòng) | Chỉ còn 1 import bị comment ở `cloud_processing/compare_pipeline.py:9` — xoá cả dòng import comment |
| `src/pps/utils_report.py` (~62 dòng) | Không tìm thấy nơi gọi |

### Package `shared` (`intelijet_v2_ws/src/shared/`)
| File | Lý do |
|------|-------|
| `src/shared/config_manager.py` (~53 dòng) | Hệ thống config song song không ai import; hệ thống thật là `config_loader.py` |

### Package `ui` (`intelijet_v2_ws/src/ui/`)
| File | Lý do |
|------|-------|
| `scripts/main.py` (~387 dòng) | Entry point cũ; entry point thật là `scripts/app.py`. Kiểm tra kỹ `setup.py`, launch file, `.desktop`, `run_intelijet.sh` trước khi xoá |
| `src/ui/intelijet_ui(old).py` (~965 dòng) | File UI generated cũ, tên chứa `(old)` không import được |

Sau khi xoá, kiểm tra `setup.py`/`CMakeLists.txt` của từng package: nếu có liệt kê file vừa xoá (ví dụ trong `scripts=[...]` hay `catkin_install_python`) thì gỡ dòng đó.

## Việc 2 — Xoá khối comment chết lớn (chỉ những khối đã xác định)

| File | Vị trí (xấp xỉ — xác định lại theo nội dung) | Nội dung |
|------|------|----------|
| `shared/src/shared/device_monitor.py` | dòng ~137-143, ~197-287 | `DEVICE_CLASSES` map bị comment + toàn bộ class `DeviceMonitor`/`DeviceStatusReader` cũ + schema msg dump. **Lưu ý:** khối `DEVICE_CLASSES` (~137-143) sẽ được viết lại ở Phase 2 — cứ xoá, P2 tự viết mới |
| `shared/src/shared/config_loader.py` | dòng ~99-110 | Bản `reload_config()` cũ bị comment |
| `pps/scripts/hmi_scan_command_handler.py` | dòng ~69-150, ~175-189 | Code actionlib cũ song song với `compare_client` đang dùng |
| `pps/src/pps/cloud_processing/compare_pipeline.py` | dòng ~96-124 | Nhánh ceres refine + code path cũ bị comment. GIỮ lại các comment 1 dòng giải thích thuật toán |
| `ui/scripts/app.py` | dòng ~32, ~154-155, ~532-535, ~629-633 | Cơ chế compare singleton cũ bị comment |

Quy tắc: chỉ xoá **khối code bị comment** (code từng chạy được), KHÔNG xoá comment giải thích. Nếu không chắc một khối là code chết hay ghi chú → giữ lại.

## Việc 3 — Kiểm chứng

1. `python3 -m py_compile` cho mọi file `.py` đã sửa (các file chỉ bị xoá thì thôi).
2. Grep xác nhận không còn tham chiếu tới các module đã xoá: `keypoint_processing_v2`, `keypoint_processing` (chú ý phân biệt với `_v3`), `sick_scan_controller` (phân biệt với `sick_scan_eRob_controller`), `compare_method_m3c2`, `utils_report`, `config_manager`, `intelijet_ui(old)`.
3. Nếu có môi trường ROS: build workspace + chạy thử app trong Docker để xác nhận UI mở bình thường.
4. Đếm số dòng đã xoá (`git diff --stat`) đưa vào báo cáo.

## Tiêu chí nghiệm thu

- [ ] Tất cả file trong danh sách (trừ file phát hiện vẫn còn được dùng) đã xoá; grep sạch.
- [ ] Không có thay đổi hành vi nào (diff chỉ gồm xoá + gỡ tham chiếu chết).
- [ ] py_compile pass toàn bộ file sửa.
- [ ] Commit `[P1] ...`; cập nhật bảng Trạng thái trong `00_INDEX.md`.

## Báo cáo hoàn thành (Sonnet điền sau khi xong)

**Trạng thái: ✅ Xong.**

### Việc 1 — Xoá file (10/10, đúng danh sách, không có ngoại lệ)
Tất cả 10 file đều grep sạch trước khi xoá (không còn tham chiếu sống nào ngoài các dòng đã dọn ở Việc 2):
- `pps`: `keypoint_processing.py` (1093), `keypoint_processing_v2.py` (934), `sick_scan_controller.py` (211), `compare_cloud_action_server_old.py` (284), `scan_to_cloud_msg_node.py` (48), `compare_method_m3c2.py` (234), `utils_report.py` (62).
- `shared`: `config_manager.py` (53).
- `ui`: `scripts/main.py` (387), `src/ui/intelijet_ui(old).py` (965).
- Tổng 4271 dòng xoá (khớp ước lượng ~4000 trong kế hoạch).
- `ui/CMakeLists.txt`: gỡ block `catkin_install_python(PROGRAMS scripts/main.py ...)` (comment lại toàn khối vì không còn script nào để liệt kê). Không tìm thấy tham chiếu nào tới các file đã xoá trong `setup.py`/`CMakeLists.txt`/`package.xml` của `pps`, `shared`.

### Việc 2 — Xoá tham chiếu chết / khối comment chết
- `pps/scripts/hmi_scan_command_handler.py`: xoá `from pps.sick_scan_controller import SickScanController` (import sống nhưng class không dùng) + dòng comment gọi nó; xoá khối actionlib `CompareCloudAction` cũ bị comment (~6 dòng) trong `__init__`; xoá khối comment cũ (~26 dòng) trong nhánh `START_COMPARE`; xoá method `__send_scan_cmd` bị comment toàn bộ (~15 dòng) ở cuối class.
- `pps/launch/pps.launch`: xoá dòng comment `<node ... scan_to_cloud_msg_node ...>`.
- `pps/src/pps/cloud_processing/compare_pipeline.py`: xoá import comment `compare_method_m3c2`; xoá import comment `ceres_aligner` (tham chiếu module không tồn tại, gắn liền khối ceres bên dưới); xoá khối code chết cuối `run()` (ceres refine + `compute_heatmap_to_plane` cũ bị comment, ~16 dòng).
- `pps/src/pps/sick_scan_eRob_controller.py`: sửa dòng header comment sai tên file (`# scripts/sick_scan_controller.py` — sót lại từ copy-paste, trỏ tới file vừa xoá).
- `shared/src/shared/device_monitor.py`: xoá map `DEVICE_CLASSES` bị comment (~8 dòng), xoá 2 dòng comment gọi `DeviceMonitor`/`DEVICE_CLASSES` cũ trong `StatusReader.__init__`, xoá toàn bộ khối cuối file (~91 dòng: class `DeviceMonitor`/`DeviceStatusReader` cũ + dump schema `.msg`).
- `shared/src/shared/config_loader.py`: xoá bản `reload_config()` cũ bị comment (~12 dòng), giữ bản đang dùng.
- `ui/scripts/app.py`: xoá `# from ui.compare_cloud_worker import cloud_compare` (đầu file), 2 dòng `cloud_compare.compare_done*.connect` bị comment (init), khối `cloud_compare.set_prescan/set_postscan/align/compare` bị comment + 2 dòng comment tiếng Việt/emoji liền kề (trong `on_manual_compare`).

### Kiểm chứng
- `python3 -m py_compile` pass cho toàn bộ 8 file Python đã sửa (không xoá): `hmi_scan_command_handler.py`, `compare_pipeline.py`, `sick_scan_eRob_controller.py`, `device_monitor.py`, `config_loader.py`, `app.py`.
- Grep quét lại toàn repo (`.py .launch .xml .sh CMakeLists.txt`) cho từng tên đã xoá — 0 kết quả, kể cả trường hợp dễ nhầm (`keypoint_processing` gốc vs `_v2`/`_v3`, `sick_scan_controller` vs `sick_scan_eRob_controller`).
- Không có môi trường ROS trong phiên này → **chưa chạy được** `catkin_make`/mở app thật. Đây là phần chưa kiểm chứng — cần làm ở máy có Docker/ROS trước khi tin tưởng tuyệt đối.
- `git diff --stat` giữa trước/sau (chỉ path `intelijet_v2_ws`) cho số dòng thay đổi lớn hơn dự kiến ở vài file KHÔNG phải do tôi sửa nhiều hơn báo cáo — xem Ghi chú phát sinh bên dưới (CRLF).

### Commit
- `664290e` — "Add optimization plan overview and detailed phase docs (P1-P9)": commit này lẽ ra chỉ nên chứa `docs/`, nhưng do 10 file đã bị `git rm` (stage) từ bước xác nhận trước đó, chúng bị gộp vào commit này khi tôi `git add docs/` rồi commit. Nội dung đúng như kế hoạch (đúng 10 file, không thừa), chỉ sai ở chỗ nằm nhầm commit/message.
- `761071c` — `[P1] Remove dead code in pps/shared/ui`: chứa phần dọn tham chiếu chết (8 file sửa nội dung ở Việc 2).

## Ghi chú phát sinh

1. **CRLF/LF tồn tại sẵn trong working tree trước khi phiên này bắt đầu** (không phải do tôi gây ra — đã xác nhận bằng `git status`/`git diff --stat` lúc mới bắt đầu, ảnh hưởng ~597 file toàn repo, insertions == deletions cho từng file, tức chỉ đổi line-ending không đổi nội dung). 8 file tôi `git add` ở Phase 1 (`app.py`, `device_monitor.py`, `config_loader.py`, `hmi_scan_command_handler.py`, `compare_pipeline.py`, `sick_scan_eRob_controller.py`, `pps.launch`, `CMakeLists.txt`) đã có sẵn CRLF trong working tree, nên khi tôi stage+commit, phần line-ending đó bị cuốn theo cùng nội dung tôi sửa — khiến `git diff --stat` giữa các commit hiển thị số dòng đổi lớn hơn nhiều so với các đoạn tôi thực sự sửa. Không ảnh hưởng hành vi Python (CRLF/LF không đổi ngữ nghĩa), nhưng nếu muốn repo nhất quán line-ending, cần một phase riêng dọn toàn bộ (KHÔNG nằm trong phạm vi tối ưu code — đây là vấn đề tooling/editor, đề xuất người dùng xử lý bằng `.gitattributes` + normalize 1 lần, ngoài phạm vi 9 phase).
2. **`_guess_calling_package()` trong `config_loader.py`** hiện không còn được gọi ở đâu (hàm mồ côi) sau khi tôi xoá block `reload_config()` cũ tham chiếu gián tiếp tới ý tưởng đó — nhưng bản thân hàm này không nằm trong danh sách Việc 2 nên tôi **giữ nguyên**, không xoá (tránh vượt phạm vi phase). Đề xuất dọn ở Phase 6 (pps cleanup) hoặc một đợt dọn `shared` riêng.
3. **`ui/CMakeLists.txt`**: sau khi bỏ `main.py`, khối `catkin_install_python` không còn script nào để liệt kê nên tôi comment nguyên khối thay vì xoá hẳn (giữ lại như một điểm neo/ghi chú cho người sau, tránh xoá cấu trúc CMake có thể cần khi thêm script khác) — nếu muốn xoá hẳn, có thể làm ở phase dọn dẹp khác.
4. **Chưa kiểm chứng bằng chạy app/catkin_make thật** — phiên này không có môi trường ROS/Docker. Đề nghị người dùng chạy thử trong Docker trước khi bắt đầu Phase 2, dù rủi ro rất thấp (chỉ xoá code chết + gỡ tham chiếu, không đổi logic runtime nào).
