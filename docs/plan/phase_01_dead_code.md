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

_(chưa có)_

## Ghi chú phát sinh

_(chưa có)_
