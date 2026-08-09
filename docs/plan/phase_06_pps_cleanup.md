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

**Trạng thái: ✅ Xong (mặt code) — CHƯA kiểm chứng bằng import/run thật.** Commit `f462b42` (Việc 3), `29ea074` (Việc 1+2).

### Việc 3 — 6/6 điểm đã sửa (commit `f462b42`)
1. `tunnel_processing.py::safe_bound_value`: bare `except:` → `except Exception as e:` + `rospy.logwarn` (thêm `import rospy` — file trước đó chưa có).
2. `helper.py::load_ply` (nay ở `cloud_utils/io_utils.py`): 2 dòng `print` (kể cả emoji) → `rospy.logwarn`/`rospy.logerr`.
3. `hmi_scan_command_handler.py::get_scanner_controller()`: `active_lidar` không khớp → `raise ValueError` rõ ràng thay vì trả `None` ngầm.
4. `compare_cloud_action_server.py`: 2 vòng busy-poll `for i in range(10): rospy.sleep(1)` → `threading.Event.wait(timeout=10)`. Giữ đúng ngân sách chờ (10s/cloud), `post_cloud_event` được `clear()` cùng lúc `post_cloud` reset (khớp hành vi gốc), `pre_cloud_event` không tự clear (khớp việc `pre_cloud` không bao giờ bị reset — cho phép tái dùng pre-scan cũ qua nhiều chu kỳ, đúng như code gốc).
5. `compare_pipeline.py`: crop bound, tham số keypoint, ngưỡng clustering → đọc từ `CONFIG` qua helper `_cfg()` (safe nested getattr, default = giá trị hardcode cũ). Thêm section `compare_pipeline:` vào `runtime.yaml`. **Quan trọng**: `last_used.yaml` (file thật đang chạy ở field, có giá trị đã tinh chỉnh khác `runtime.yaml`) **không được sửa** — vì `reload_config()` ưu tiên `last_used.yaml` toàn bộ khi nó tồn tại, nên `_cfg()` dùng fallback default để không bị vỡ trên máy có `last_used.yaml` cũ chưa có section mới này.
6. `crop_cloud_by_hull` fail trong `compare_pipeline.py`: thêm `log_status(..., level="warning")` bên cạnh `rospy.logerr` sẵn có, để lỗi này hiện lên `NotificationCenter` (Phase 3) thay vì chỉ nằm trong log server.

### Việc 1 + 2 — Tách `helper.py` + hợp nhất API convert (commit `29ea074`)
- **Khảo sát kỹ hơn dự kiến của kế hoạch**: grep toàn repo mọi `from pps.helper import`, rồi truy vết cây gọi hàm NỘI BỘ trong chính `helper.py` (một số hàm "trông như dùng nội bộ" hoá ra chỉ được gọi bởi 1 hàm khác cũng chết) → phát hiện chỉ **13/30 hàm** có người dùng thật (kể cả gián tiếp), **19 hàm hoàn toàn chết** (0 caller ở bất kỳ đâu) — nhiều hơn hẳn so với ước lượng "một vài hàm dead" trong kế hoạch gốc. Đã xoá thẳng 19 hàm này (không di chuyển) kèm ~140 dòng code comment chết đi theo (bản `assign_colors` cũ, `compute_heatmap_to_plane_old_version`).
- 13 hàm còn sống chuyển vào `pps/cloud_utils/` theo đúng 4 nhóm kế hoạch đề ra: `geometry.py` (crop_pointcloud_by_box, smooth_cloud, surface_area), `comparison.py` (compute_heatmap_to_plane, run_compare, check_transform, filter_pcd_by_distance, keep_largest_cluster), `coloring.py` (assign_colors, map_distances_to_colors), `io_utils.py` (load_ply, notify_one).
- `helper.py` giờ chỉ còn 55 dòng — shim re-export đúng 12 tên còn được import từ bên ngoài, kèm docstring liệt kê đầy đủ tên đã xoá và lý do. **Không cần sửa bất kỳ call site nào** (`compare_pipeline.py`, `icp_aligner.py`, `ransac_aligner.py`, `compare_cloud_worker.py`, `update_data_utils.py`, `cloud_pipeline.py`, `report_utils.py`, `dv_cloud_preprocess.py` đều vẫn import qua `pps.helper` y như cũ, trỏ tới shim).
- Việc 2: trong 6 hàm convert PointCloud2↔Open3D ở `helper.py`, chỉ `convert_open3d_to_pointcloud2` có người dùng thật (1 call site — `dv_cloud_preprocess.py`). Đã chuyển thành method `CloudConverter.legacy_o3d_to_pointcloud2()` trong `data_converter.py` (đặt cạnh `o3d_tensor_to_pointcloud2` đã có sẵn cho trường hợp tensor), **sửa trực tiếp** call site duy nhất đó (không tạo shim cho trường hợp chỉ 1 caller — hợp lý hơn). 5 hàm convert còn lại đã nằm trong nhóm 19 hàm chết bị xoá ở trên, nên kết quả cuối đúng như mục tiêu Việc 2: mọi convert đi qua `CloudConverter`.
- `pps/setup.py`/`CMakeLists.txt` không cần sửa cho package con `cloud_utils` mới — cùng lý do đã xác nhận ở Phase 4 cho `ui.services` (`catkin_python_setup()` symlink cả cây thư mục devel-space, các subpackage sẵn có như `pps.cloud_processing` cũng không được liệt kê trong `setup.py`).
- `py_compile` pass cho toàn bộ 8 file sửa/mới của Việc 1+2 (cộng 5 file Việc 3).

### Chưa kiểm chứng được (không có ROS/Qt/Open3D **và lần này không có cả numpy/scipy** trong sandbox)
- Không thể `import pps.helper`/`pps.cloud_utils.*`/`pps.data_converter` thật để xác nhận resolve đúng — chỉ kiểm chứng được bằng `py_compile` (cú pháp) + soát tay kỹ từng hàm bị di chuyển xem có tham chiếu tới hàm nào bị xoá không (đã làm cẩn thận, xem chi tiết ở trên).
- Chưa chạy được 1 chu trình compare đầy đủ để so sánh cloud đầu ra trước/sau (mục 3 phần Kiểm chứng của kế hoạch) — cần Docker.
- Nhánh `threading.Event` mới trong `compare_cloud_action_server.py` (Việc 3.4) chưa chạy thử thật với ROS actionlib.

## Ghi chú phát sinh

1. **Phạm vi xoá code chết lớn hơn dự kiến ban đầu của kế hoạch** (19 hàm thay vì "một vài") — đã ghi rõ bằng chứng grep cho từng hàm trong commit message, để người review có thể tự kiểm tra lại nếu nghi ngờ.
2. `last_used.yaml` là **config thật đang chạy ở hiện trường** (giá trị khác hẳn `runtime.yaml`, ví dụ `thickness.target=50` so với `30` trong `runtime.yaml`) — chủ động **không sửa file này**, chỉ dựa vào cơ chế fallback default trong code. Nếu người vận hành muốn áp dụng giá trị mới cho `compare_pipeline:` (crop bound/keypoint/clustering) trên máy đang chạy `last_used.yaml`, cần chủ động thêm section đó vào `last_used.yaml` hoặc xoá file này để hệ thống merge lại từ `commond.yaml`/`lidar.yaml`/`runtime.yaml` ở lần khởi động tiếp theo.
3. Dọn thêm 1 comment tham chiếu tới `remove_small_clusters` (hàm vừa xoá) còn sót trong `filter_pcd_by_distance` khi di chuyển sang `comparison.py` — comment mô tả 1 dòng code cũ bị comment, không phải code sống, xoá theo đúng tinh thần Phase 1.
4. Đổi tag log trong `load_ply` từ `[helper.load_ply]` (viết ở Việc 3, cùng phiên) thành `[cloud_utils.load_ply]` khi di chuyển sang `io_utils.py` — vì tag cũ do chính tôi viết trong Việc 3 của cùng phiên này, cập nhật theo vị trí mới là hợp lý, không phải sửa hành vi gốc của dự án.
