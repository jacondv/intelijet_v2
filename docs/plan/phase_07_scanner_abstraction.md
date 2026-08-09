# Phase 7 — Trừu tượng hoá scanner (chuẩn bị thay SickScan bằng scanner 3D trực tiếp)

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: P6. Rủi ro: vừa.
> Phạm vi file: `pps/src/pps/generic_scan_controller.py`, `pps/src/pps/sick_scan_eRob_controller.py`, `pps/scripts/hmi_scan_command_handler.py`, file mới `pps/src/pps/scan_strategies/`. KHÔNG tích hợp driver BLK360G2 thật — chỉ chuẩn bị interface + một strategy giữ hành vi hiện tại.

## Bối cảnh (người dùng đã chốt — quan trọng, đọc kỹ)

Hiện tại: SickScan là LiDAR **2D**. Quy trình: housing mở **theo nhiều bước/góc tăng dần** trong lúc SickScan liên tục quét lát 2D; kết thúc, service `assemble_scans2` (gói `laser_assembler`) ghép mọi lát trong khoảng `start_time→end_time` thành 1 point cloud 3D (`sick_scan_eRob_controller.py::run_workflow`). "3D" là sản phẩm của (chuyển động housing × thời gian × dữ liệu 2D).

Tương lai (đã có scaffold `blk360g2_ws/` — Leica BLK360 G2, chưa tích hợp): scanner trả **trực tiếp point cloud 3D** trong 1 lần chụp. Khi đó: housing chỉ mở **một góc cố định** → gọi scanner chụp → nhận cloud 3D → đóng housing. Không cần `assemble_scans2`, không cần đồng bộ thời gian với chuyển động housing.

Mục tiêu phase: sau này thêm scanner mới = **viết 1 class strategy mới**, không sửa controller/luồng job/report.

## Hiện trạng code

- `generic_scan_controller.py`: `HousingControl` (gửi lệnh PLC mở/đóng/stop/set-speed qua topic CAN) + abstract `GenericScanController` (chạy prescan/postscan trong `threading.Thread` daemon, có `cancel_job`, `_open_housing_sequence`/`_close_housing_sequence` — chuỗi mở nhiều bước gắn giả định quét 2D, `wait_until_target` theo encoder/joint_states).
- `sick_scan_eRob_controller.py::run_workflow`: log start → `start_time = now` → `_open_housing_sequence()` (housing quay + SickScan tự quét nền) → `end_time = now`, `housing.stop()` → `assemble_cloud_client(start_time, end_time)` → publish cloud → `_close_housing_sequence()`.
- `hmi_scan_command_handler.py::get_scanner_controller()`: chọn controller theo `cfg.active_lidar` (sau P6 đã raise lỗi rõ khi không khớp).

## Thiết kế đích

### Interface — file mới `pps/src/pps/scan_strategies/base.py`

```python
class ScanStrategy(ABC):
    """One scan acquisition method. Owns ONLY how the point cloud is obtained.
    Housing motion is passed in and driven by the strategy because motion
    and acquisition are coupled differently per scanner type."""

    @abstractmethod
    def acquire(self, housing, cancel_check) -> PointCloud2 | None:
        """Run one full acquisition: move housing as needed, obtain cloud,
        return it (housing NOT yet closed). Return None on failure.
        cancel_check: zero-arg callable; poll between steps, abort ASAP when True."""
```

Ghi chú thiết kế (đã cân nhắc): tách "chuyển động housing" và "thu cloud" thành 2 interface độc lập hoàn toàn nghe đẹp nhưng KHÔNG khớp thực tế — với SickScan, chuyển động housing chính LÀ một phần của phép đo (thời gian quay quyết định dữ liệu). Vì vậy strategy nhận `housing` (đối tượng `HousingControl` + các hàm chờ encoder) và tự quyết cách dùng: SickScan dùng kiểu "quay từng bước + cửa sổ thời gian", BLK sau này dùng kiểu "mở tới góc X rồi đứng yên". Phần dùng chung thật sự (mở/đóng/stop, chờ encoder, cancel) nằm ở `HousingControl`/helper của controller.

### Strategy hiện tại — `scan_strategies/sick_2d_assemble.py`

Class `Sick2DAssembleStrategy(ScanStrategy)`: chuyển nguyên phần thân `run_workflow` hiện tại (start_time → open sequence → stop → `assemble_scans2` → trả cloud) vào `acquire()`. `assemble_cloud_client` chuyển vào file này. Hành vi giữ nguyên 100%.

### Controller sau tái cấu trúc

- `GenericScanController` nhận `strategy: ScanStrategy` khi khởi tạo (inject), `run_workflow` trở thành code chung: gọi `strategy.acquire(housing, cancel_check=lambda: self.cancel_job)` → publish cloud + log + status callback → `_close_housing_sequence()` (đóng housing luôn chạy dù acquire fail — an toàn cơ khí, dùng try/finally).
- `SickScanErobController` teo lại thành: `GenericScanController` + `Sick2DAssembleStrategy` + các hằng số tốc độ/góc riêng (giữ tên class để `hmi_scan_command_handler.py` và mọi log không đổi).
- `get_scanner_controller()` giữ nguyên giao diện; thêm comment chỉ dẫn: thêm scanner mới = thêm strategy + 1 nhánh chọn (hoặc entry dict) tại đây.

### Skeleton cho tương lai — `scan_strategies/direct_3d.py`

Class `Direct3DScanStrategy(ScanStrategy)` — **skeleton có docstring, raise NotImplementedError**, mô tả rõ luồng dự kiến: mở housing tới góc cấu hình `cfg.housing_fixed_open_angle` → gọi driver scanner (topic/action của `blk360g2_ros` — để TODO) → nhận PointCloud2 → trả về. Mục đích: người sau nhìn vào là biết viết gì, ở đâu.

## Ràng buộc

- Hành vi scan hiện tại phải y hệt (từng log message, từng status callback — vì UI phụ thuộc vào chúng).
- Không đổi topic, không đổi message, không đổi enum `PPSCommand`.
- Try/finally bảo đảm housing được lệnh đóng kể cả khi acquire ném exception (hiện trạng cũng cần soát: nếu code cũ KHÔNG đóng housing khi lỗi thì giữ nguyên hành vi cũ và ghi chú lại — không tự ý "sửa" hành vi cơ khí ở phase này, chỉ ghi nhận).
- Cực kỳ cẩn trọng với luồng thread hiện có (`_run_in_thread`, `cancel_job`) — giữ nguyên cấu trúc, chỉ di chuyển thân workflow.

## Kiểm chứng

1. `py_compile` + import-check các file sửa/mới.
2. Diff logic: đối chiếu từng bước `acquire()` mới với `run_workflow` cũ — phải là di chuyển thuần, không thêm/bớt bước. Ghi bảng đối chiếu ngắn vào báo cáo.
3. Nếu có phần cứng/simulator: chạy 1 chu trình prescan/postscan thật, so log với trước.

## Tiêu chí nghiệm thu

- [ ] `ScanStrategy` interface + `Sick2DAssembleStrategy` (hành vi cũ) + `Direct3DScanStrategy` skeleton.
- [ ] `GenericScanController` không còn giả định 2D-assemble nào; nhận strategy inject.
- [ ] `hmi_scan_command_handler.py` không đổi giao diện; thêm scanner mới không phải sửa controller.
- [ ] Housing luôn được lệnh đóng khi kết thúc/lỗi (hoặc ghi chú rõ nếu giữ hành vi cũ khác).
- [ ] Commit `[P7] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

_(chưa có)_

## Ghi chú phát sinh

_(chưa có)_
