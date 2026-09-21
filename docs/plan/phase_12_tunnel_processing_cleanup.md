# Phase 12 — Tổ chức lại wall-removal logic trong `TunnelProcessing` (remove ground / remove back side / …)

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P6 (tiếp nối trực tiếp phong cách `_cfg()`/parameterize-via-CONFIG đã áp dụng cho `compare_pipeline.py`). Rủi ro: vừa (đụng logic hình học dùng trong mọi lần compare).
> Phạm vi: `pps/tunnel_processing.py` (wall-removal: ground/back/left/right/front) + dọn code chết liên quan trực tiếp phát hiện được trong lúc khảo sát. KHÔNG đổi thuật toán/kết quả crop cuối cùng của `run_processing_pipeline()`.

## Bối cảnh

`CloudComparePipeline.run()` (`cloud_processing/compare_pipeline.py`) gọi `TunnelProcessing(post_cloud).run_processing_pipeline()` ở bước "pre-process" để cắt bỏ nền/tường tunnel trước khi so sánh 2 cloud. Bên trong, một hàm dùng chung tên `get_plane()` — đặt tên/docstring chỉ nói "Remove ground plane" — được gọi 5 lần với 5 bộ box/tham số khác nhau để dò ground, back wall, left wall, right wall, front wall, khiến `run_processing_pipeline()` khó đọc (5 khối inline gần giống hệt nhau) và mỗi lần gọi tự build lại một `cKDTree` trên toàn bộ cloud gốc (giống hệt nhau ở cả 5 lần) — lãng phí.

## Việc 1 — Tổ chức lại: đặt tên rõ theo từng mặt tường, dùng chung 1 KDTree

1. Đổi tên `get_plane()` → `_detect_wall_plane()` (primitive nội bộ). Giữ nguyên 100% thuật toán và thứ tự các bước (crop box → voxel_down_sample → estimate_normals → lọc theo góc với reference axis → mở rộng theo `radius` qua KDTree → PCA normal của vùng mở rộng). Thêm tham số `tree: cKDTree = None`: nếu được truyền thì dùng luôn thay vì tự `cKDTree(self.pcd.points)` bên trong; nếu `None` thì build như cũ (không phá caller khác nếu có).
2. Thêm 5 method mỏng, tên rõ nghĩa nghiệp vụ, mỗi cái gọi `_detect_wall_plane()` với box/reference-plane cố định cho đúng mặt đó (đọc box từ CONFIG — xem Việc 2):
   - `detect_ground(tree)` — reference_plane="xy", box "bottom"
   - `detect_back_wall(tree)` — reference_plane="yz", box "back"
   - `detect_right_wall(tree)` — reference_plane="xz", box "right"
   - `detect_left_wall(tree)` — reference_plane="xz", box "left"
   - `detect_front_wall(tree)` — reference_plane="yz", box "front"
3. `run_processing_pipeline()`: build `tree = cKDTree(self.pcd.points)` một lần, gọi 5 method trên thay cho 5 khối `self.get_plane(...)` inline trước đây. Phần ghép `minbound/maxbound` qua `safe_bound_value()` + `self.crop(...)` phía sau **giữ nguyên không đổi**.
4. `TOP_BOX`/plane trần: **không** dùng trong bất kỳ lệnh `detect_*wall()` nào — xem mục "Vấn đề đã biết, chưa sửa" bên dưới.
5. Xoá `remove_point()` (0 caller — xem Kiểm chứng) và 2 import chỉ phục vụ nó (`scipy.spatial.ConvexHull`, `matplotlib.path.Path`).

## Việc 2 — Tham số hoá 6 box + ngưỡng detect qua CONFIG (nối tiếp đúng khuôn mẫu Phase 6)

- Thêm `_cfg()` local trong `tunnel_processing.py`, bản sao y hệt logic trong `compare_pipeline.py` (an toàn nested-getattr, default = giá trị hardcode cũ). Giữ làm bản riêng theo từng file (không tách shared util) — đúng tiền lệ Phase 6 đã chọn cho `compare_pipeline.py`.
- Thêm section `tunnel_processing:` vào `intelijet_v2_ws/src/config/runtime.yaml` (đặt trước `report:`), default = đúng 6 box + `normal_angle_threshold=5`/`radius=0.15` hardcode cũ. **Không sửa `last_used.yaml`** — cùng lý do đã ghi ở Phase 6 (file thật ngoài field, ưu tiên toàn bộ khi tồn tại; `_cfg()` fallback về default nên máy chưa có section mới vẫn chạy y hệt).
- Không đưa các offset trong `safe_bound_value()` (`-0.5, -1.0, +0.3, -0.1, +1.0` và các fallback `0.5/-5.0/0.0/10.0/5.0/15.0`) vào CONFIG — giữ hardcode, đúng mức độ chi tiết Phase 6 từng áp dụng (chỉ tham số hoá "núm vặn" chính).

## Việc 3 — Xoá code chết liên quan (grep xác nhận 0 caller trước khi xoá)

| Vị trí | Trạng thái | Hành động |
|---|---|---|
| `data_converter.py::CloudConverter.crop()` | 0 caller, trùng chức năng với `cloud_utils/geometry.py::crop_pointcloud_by_box` (đang dùng) và `TunnelProcessing.crop()` (đang dùng) | Xoá |
| `tunnel_processing.py::TunnelProcessing.remove_point()` | 0 caller | Xoá (gộp vào Việc 1.5) |
| `ui/src/ui/compare_cloud_worker.py`: `from pps.tunnel_processing import TunnelProcessing` | Import chết, không dùng trong file | Xoá dòng import |
| `cloud_processing/icp_aligner.py`: `from pps.helper import crop_pointcloud_by_box` | Import chết | Xoá dòng import |
| `cloud_processing/ransac_aligner.py`: `from pps.helper import crop_pointcloud_by_box` | Import chết | Xoá dòng import |

## Vấn đề đã biết, chưa sửa (theo yêu cầu người dùng — để lại đợt sau)

`TOP_BOX`/ceiling: box được khai báo trong config (`tunnel_processing.boxes.top`) để giữ đối xứng với 5 mặt còn lại, nhưng **không có `detect_*wall()` nào dùng nó** — `maxbound[2]` trong `run_processing_pipeline()` vẫn là hằng số cố định `15.0`, không phụ thuộc trần dò được thật. Đây là hành vi gốc trước khi sửa phase này, người dùng đã xác nhận **giữ nguyên, chưa sửa** trong đợt này — cần một phase riêng sau này nếu muốn bật crop theo trần thật (sẽ đổi kết quả crop, cần test lại trên Docker/phần cứng).

## Ràng buộc

- Không đổi hành vi thuật toán/kết quả crop cuối cùng của `run_processing_pipeline()` — chỉ đổi tổ chức code + nguồn tham số (config thay vì hardcode, default giữ nguyên).
- `cloud_processing/compare_pipeline.py` (caller duy nhất từ ngoài `tunnel_processing.py`) không cần sửa — `run_processing_pipeline()` giữ nguyên chữ ký.

## Kiểm chứng

1. `python3 -m py_compile` (qua `py_compile.compile(..., cfile=<scratch>)` để tránh lỗi quyền ghi `__pycache__` trong sandbox) cho cả 5 file sửa: `tunnel_processing.py`, `data_converter.py`, `ui/compare_cloud_worker.py`, `icp_aligner.py`, `ransac_aligner.py` — **pass**.
2. Grep repo-wide xác nhận: không còn `get_plane`/`remove_point`/`ConvexHull`/`Path(` trong `tunnel_processing.py`; không còn `CloudConverter.crop(` hay định nghĩa `crop(pcd, min_bound, max_bound)` trong `data_converter.py`; không còn `crop_pointcloud_by_box` trong `icp_aligner.py`/`ransac_aligner.py`; không còn `TunnelProcessing` trong `ui/compare_cloud_worker.py` — **tất cả pass, không còn call site nào vỡ**.
3. `runtime.yaml` được `yaml.safe_load()` parse thành công, section `tunnel_processing` đúng cấu trúc mong đợi (kiểm bằng script Python nhỏ) — **pass**.
4. Soát tay: công thức `minbound`/`maxbound`/`safe_bound_value()` trong `run_processing_pipeline()` sau khi sửa giống hệt bản trước (chỉ khác nguồn gọi `detect_*wall(tree)` thay vì `get_plane(...)` inline) — **đã đối chiếu, khớp**.

### Chưa kiểm chứng được (không có ROS/Open3D/numpy thật trong sandbox, giống mọi phase trước)

- Không chạy được `import pps.tunnel_processing` thật (thiếu ROS + Open3D + numpy trong môi trường này) — chỉ `py_compile` + soát tay.
- Chưa chạy 1 chu trình compare đầy đủ (pre-scan/post-scan thật) để xác nhận cloud sau `run_processing_pipeline()` giống hệt trước/sau về mặt số liệu — cần Docker/phần cứng, theo đúng tiền lệ Phase 6-11.
- Nhánh `tree=None` (fallback tự build KDTree) trong `_detect_wall_plane()` chưa có caller nào dùng thật ngoài `run_processing_pipeline()` (luôn truyền `tree`) — giữ lại chỉ để không phá vỡ nếu có code khác gọi trực tiếp trong tương lai, chưa test riêng nhánh này.

## Tiêu chí nghiệm thu

- [x] `get_plane()` đổi tên `_detect_wall_plane()`, dùng chung 1 `cKDTree` qua tham số `tree`.
- [x] 5 method `detect_ground/detect_back_wall/detect_left_wall/detect_right_wall/detect_front_wall` thay cho 5 khối inline trong `run_processing_pipeline()`.
- [x] 6 box + `normal_angle_threshold`/`radius` chuyển vào `runtime.yaml` qua `_cfg()`, default giữ nguyên hardcode cũ.
- [x] 5 điểm code chết ở Việc 3 đã xoá, không call site nào vỡ (grep xác nhận).
- [x] `TOP_BOX`/ceiling: giữ nguyên hành vi, ghi chú rõ trong code + phase doc.
- [x] py_compile pass toàn bộ file sửa.
- [ ] Commit `[P12] ...` — chưa thực hiện, chờ người dùng xác nhận trước khi `git commit` (theo quy tắc an toàn: chỉ commit khi được yêu cầu rõ ràng).

## Báo cáo hoàn thành

**Trạng thái: ✅ Xong (mặt code) — CHƯA kiểm chứng bằng import/run thật, CHƯA commit.**

Đã đọc trực tiếp toàn bộ file liên quan trong `pps/` (không qua subagent) để nắm luồng: `compare_cloud_action_manual_server.py` → `CloudComparePipeline.run()` → `TunnelProcessing.run_processing_pipeline()` (+ `cloud_utils/*`, `data_converter.py`, `cloud_processing/*aligner.py`). Việc 1+2+3 thực hiện trong 1 phiên, cùng lúc với việc tạo phase doc này.

Grep bằng chứng cho từng mục xoá ở Việc 3 (chạy trước khi xoá, không tìm thấy caller nào ngoài định nghĩa):
- `CloudConverter.crop(` / `cloudconverter.crop(` / `converter.crop(`: 0 kết quả ngoài định nghĩa hàm.
- `remove_point\b`: chỉ xuất hiện ở định nghĩa (`tunnel_processing.py`) và trong docstring liệt kê hàm đã xoá của `helper.py` (không phải call site).
- `from pps.tunnel_processing import TunnelProcessing` trong `compare_cloud_worker.py`: không có usage nào khác của tên `TunnelProcessing` trong file đó.
- `from pps.helper import crop_pointcloud_by_box` trong `icp_aligner.py`/`ransac_aligner.py`: không có lệnh gọi `crop_pointcloud_by_box(...)` nào trong 2 file đó.

## Ghi chú phát sinh

1. `TOP_BOX`/ceiling detection chưa từng hoạt động (xem mục "Vấn đề đã biết, chưa sửa") — người dùng đã xác nhận để lại cho một phase sau, không mở rộng phạm vi phase này.
2. `axis_idx` trong `_detect_wall_plane()` (được gán theo `reference_plane` nhưng không dùng ở đâu khác trong hàm) là biến chết có từ trước khi sửa phase này — giữ nguyên, không thuộc phạm vi "0-caller function/import" đã grep xác nhận ở Việc 3, và sửa nó không mang lại giá trị đủ lớn để đánh đổi rủi ro đụng vào phần thuật toán trong cùng 1 phiên.
