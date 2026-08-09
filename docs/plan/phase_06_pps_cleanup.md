# Phase 6 — Dọn dẹp `pps`: tách helper.py, hợp nhất API convert, chuẩn hoá lỗi

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P1. Rủi ro: vừa (đụng nhiều import).
> Phạm vi: package `pps` (`intelijet_v2_ws/src/pps/`). KHÔNG đụng scanner/housing controller (`generic_scan_controller.py`, `sick_scan_eRob_controller.py`) — đó là P7. KHÔNG đổi thuật toán — chỉ di chuyển/hợp nhất/sửa lỗi kỹ thuật.

## Việc 1 — Tách `src/pps/helper.py` (1688 dòng, ~30 hàm)

Tạo package `src/pps/cloud_utils/` và chia hàm theo nhóm (tên gợi ý — Sonnet chốt theo nội dung thực tế):
- `conversion.py` — các hàm convert PointCloud2↔Open3D (SẼ hợp nhất ở Việc 2, xem trước khi chia).
- `geometry.py` — crop, PCA ground removal, boundary detection, surface area.
- `comparison.py` — `run_compare`, `run_compare_m3c2`, `compute_heatmap_to_plane`, clustering.
- `coloring.py` — assign_colors / color mapping / histogram.
- `io_utils.py` — PLY load/save và I/O khác.

Cách làm an toàn:
1. Grep toàn repo mọi nơi `import helper` / `from pps.helper import` / `from pps import helper` — lập danh sách call site TRƯỚC khi di chuyển (cả `ui`, `pps`, scripts test).
2. Di chuyển hàm theo nhóm; `helper.py` giữ lại làm **shim tương thích**: `from pps.cloud_utils.geometry import *  # noqa` v.v. + comment "Deprecated: import from pps.cloud_utils.* — this shim will be removed". Nhờ shim, KHÔNG cần sửa call site hàng loạt trong phase này (giảm rủi ro + tiết kiệm token); các phase sau đụng file nào thì đổi import file đó.
3. Hàm nào không có call site nào (dead) → xoá luôn, ghi danh sách vào báo cáo.

## Việc 2 — Hợp nhất API convert point cloud

Hiện có 2 hệ song song:
- `helper.py`: `convert_pointcloud2_to_o3d` (~768), `convert_open3d_to_pointcloud2` (~805), `convert_open3d_to_pointcloud2_v2` (~856), `convert_open3d_to_pointcloud2_with_diff` (~945), `convert_pointcloud2_to_pointcloud` (~1035).
- `data_converter.py`: class `CloudConverter` — `pointcloud2_to_o3d` (17), `pointcloud2_to_o3d_tensor` (86), `o3d_tensor_to_pointcloud2` (153), + PLY, crop, downsample, to-VTK. **UI đang dùng class này** → chọn `CloudConverter` làm chuẩn.

Cách làm:
1. Với từng hàm convert trong `helper.py`: tìm call site → nếu `CloudConverter` đã có hàm tương đương (so sánh kỹ: fields, màu, tensor/legacy) thì đổi call site sang `CloudConverter` và xoá hàm helper; nếu CHƯA có tương đương (ví dụ `with_diff`) thì **chuyển hàm đó vào `CloudConverter`** như một method rồi cập nhật call site.
2. Kết quả cuối: mọi convert PointCloud2↔Open3D đi qua `CloudConverter`; `cloud_utils/conversion.py` không tồn tại hoặc chỉ re-export.
3. So sánh đầu ra trước/sau với 1 cloud mẫu nếu có điều kiện chạy (file PLY test trong `pps/test/` nếu có) — ít nhất soát tay từng field (x,y,z,rgb/intensity) để chắc không lệch format.

## Việc 3 — Sửa các điểm lỗi/anti-pattern cụ thể (đã khảo sát)

| Vị trí | Vấn đề | Sửa |
|--------|--------|-----|
| `src/pps/tunnel_processing.py` ~dòng 630 | bare `except:` nuốt cả KeyboardInterrupt, không log | `except Exception as e:` + `rospy.logerr` (hoặc xử lý cụ thể hơn theo ngữ cảnh) |
| `helper.py` ~1371 (sau tách: file mới) | `print(f"❌ Can't load file...")` | `rospy.logerr` |
| `scripts/hmi_scan_command_handler.py` ~22-26 `get_scanner_controller()` | config `active_lidar` không khớp → trả `None` ngầm → AttributeError khó hiểu về sau | raise `ValueError` với message rõ ngay lúc khởi động |
| `scripts/compare_cloud_action_server.py` ~74-82 | busy-poll `for i in range(10): rospy.sleep(1)` chờ pre/post cloud | thay bằng `threading.Event` set trong `_pre_cb`/`_post_cb`, `event.wait(timeout=10)`; timeout thì `set_aborted` với message rõ |
| `cloud_processing/compare_pipeline.py` ~31-35, ~47-51, ~101-102 | crop bound, tham số keypoint, ngưỡng clustering hardcode | chuyển thành tham số đọc từ `shared.config_loader.CONFIG` với **default = giá trị hiện tại** (thêm key mới vào file YAML config tương ứng — tìm file phù hợp trong thư mục config, ví dụ `runtime.yaml`/`lidar.yaml`; nếu không chắc file nào, thêm vào `commond.yaml` kèm comment) |
| `cloud_processing/compare_pipeline.py` ~93-96 | `crop_cloud_by_hull` fail → nuốt lỗi, tiếp tục im lặng | vẫn tiếp tục (đúng ý đồ degrade) nhưng `log_status(..., level="warning")` để UI biết bước bị bỏ qua |

## Ràng buộc

- Không đổi hành vi thuật toán; tham số hoá phải giữ default = giá trị hardcode cũ.
- `CloudComparePipeline.run()` chưa cần thêm cancellation trong phase này (ghi nhận là cải tiến tương lai — tránh phình phase).
- Nếu phiên cạn token: hoàn thành theo thứ tự Việc 3 → Việc 1 → Việc 2 (Việc 3 nhỏ và giá trị ổn định cao nhất).

## Kiểm chứng

1. `py_compile` toàn bộ file sửa/mới; import-check `pps.helper` (shim) và `pps.cloud_utils.*` nếu môi trường cho phép.
2. Grep xác nhận không call site nào vỡ (mọi tên hàm cũ vẫn resolve qua shim).
3. Nếu chạy được: 1 chu trình compare đầy đủ, kết quả cloud so sánh giống trước.

## Tiêu chí nghiệm thu

- [ ] `helper.py` chỉ còn là shim re-export; code thật nằm trong `cloud_utils/` theo nhóm.
- [ ] Convert chỉ còn một đường `CloudConverter`.
- [ ] 6 điểm ở Việc 3 đã sửa.
- [ ] Không call site nào vỡ; py_compile pass.
- [ ] Commit `[P6] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

_(chưa có)_

## Ghi chú phát sinh

_(chưa có)_
