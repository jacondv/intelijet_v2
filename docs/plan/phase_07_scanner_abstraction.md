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

**Trạng thái: ✅ Xong (mặt code) — CHƯA chạy thử prescan/postscan thật.** Commit `78d8daa`.

### Đã làm
- `pps/scan_strategies/base.py`: interface `ScanStrategy.acquire(controller, publisher)` — 1 method duy nhất ôm toàn bộ acquisition (chuyển động housing + lấy cloud + publish), đúng như "Ghi chú thiết kế" trong kế hoạch đã cân nhắc trước (không tách 2 interface riêng vì với SickScan chuyển động housing chính là 1 phần phép đo).
- `pps/scan_strategies/sick_2d_assemble.py`: `Sick2DAssembleStrategy` — chuyển nguyên thân `run_workflow`/`_open_housing_sequence`/`_close_housing_sequence`/`assemble_cloud_client` cũ, chỉ đổi `self.` → `controller.` ở đúng những chỗ thuộc về controller. **Đã đối chiếu bằng diff chuẩn hoá** giữa bản cũ và bản mới — khớp 100% ngoại trừ đúng phần thay thế có chủ đích, không sót/thêm bước nào.
- `pps/scan_strategies/direct_3d.py`: skeleton `Direct3DScanStrategy`, `raise NotImplementedError`, docstring mô tả rõ luồng 5 bước dự kiến (mở góc cố định → gọi BLK360G2 → publish → đóng 1 lần → status_callback) và ghi rõ phần chưa biết (tên topic/action thật của `blk360g2_ros`). Chưa nối vào `get_scanner_controller()`.
- `generic_scan_controller.py`: `GenericScanController.__init__` nhận `strategy`; `run_workflow()` chỉ còn `return self.strategy.acquire(self, publisher)` — **cố tình KHÔNG bọc try/finally ép đóng housing** dù "Thiết kế đích" gợi ý vậy, vì code gốc vốn chỉ gọi `housing.stop()` (không phải `close()`) khi mở thất bại — Ràng buộc yêu cầu giữ đúng hành vi cơ khí hiện tại, không "cải thiện" nó trong phase này (xem Ghi chú phát sinh #1). `reset()` chuyển từ abstract sang concrete mặc định (`self.housing.stop()`) vì vốn đã scanner-agnostic. Bỏ import `abstractmethod` không còn dùng.
- `sick_scan_eRob_controller.py`: teo còn 6 dòng, chỉ inject `Sick2DAssembleStrategy()`, giữ nguyên tên class + chữ ký constructor.
- `hmi_scan_command_handler.py`: chỉ thêm comment chỉ dẫn tại `get_scanner_controller()`, **không đổi giao diện/hành vi** — đã grep xác nhận không còn file nào khác tham chiếu `GenericScanController`/`SickScanErobController`.
- `py_compile` pass cho toàn bộ 7 file sửa/mới.

### Chưa kiểm chứng được (không có ROS/numpy/scipy trong sandbox)
- Chưa chạy được 1 chu trình prescan/postscan thật để so log với bản trước (mục 3 phần Kiểm chứng) — cần Docker + phần cứng/simulator.
- Chưa xác nhận `wait_until_target`/`joint_state_cb`/encoder callback hoạt động đúng qua lớp `controller` gián tiếp (logic không đổi, nhưng chỉ soát tay được, chưa chạy thật).

## Ghi chú phát sinh

1. **Quyết định có chủ đích, khác với gợi ý trong mục "Thiết kế đích"**: không thêm try/finally ép `housing.close()`/`housing.stop()` bao quanh `strategy.acquire()` trong `GenericScanController.run_workflow()`. Lý do: code gốc hiện tại, khi `_open_housing_sequence()` thất bại, chỉ gọi `housing.stop()` (dừng tại chỗ) chứ không chủ động đóng lại — đây là hành vi cơ khí hiện tại. Mục "Ràng buộc" của chính kế hoạch này ghi rõ: "nếu code cũ KHÔNG đóng housing khi lỗi thì giữ nguyên hành vi cũ và ghi chú lại — không tự ý sửa hành vi cơ khí ở phase này". Tôi ưu tiên Ràng buộc (rõ ràng, cụ thể) hơn phần Thiết kế đích (mang tính gợi ý/aspirational) khi 2 phần mâu thuẫn nhau. Nếu về sau muốn có an toàn "luôn đóng khi lỗi", cần bàn với người vận hành trước vì đây là thay đổi hành vi cơ khí thật, ảnh hưởng an toàn thiết bị — không nên tự ý quyết trong 1 phase refactor.
2. Giá trị trả về của `acquire()`/`run_workflow()` hiện **không được ai sử dụng thật sự** — `_scan_thread()` gọi `cloud = self.run_workflow(publisher)` nhưng biến `cloud` không dùng tiếp (cloud thật đã được publish trực tiếp bên trong `acquire()` rồi). Đã ghi chú rõ điều này trong docstring `ScanStrategy.acquire()` để người viết strategy mới (vd Direct3D) không nhầm tưởng cần trả cloud ra ngoài để ai đó publish hộ — bản thân strategy phải tự publish.
3. `self.start_time`/`self.end_time` (thuộc tính lưu trên controller trong code gốc) đã đổi thành biến cục bộ trong `acquire()` — đã grep xác nhận không nơi nào khác trong `pps` đọc 2 thuộc tính này ngoài chính hàm đó, nên an toàn.
4. Interface thực tế lệch nhẹ so với chữ ký gợi ý trong kế hoạch (`acquire(self, housing, cancel_check)`) — dùng `acquire(self, controller, publisher)` vì `wait_until_target`/`status_callback`/`cancel_job` đều là thành viên của `controller` (không phải riêng `housing`), và `publisher` bắt buộc phải truyền vào vì chính strategy tự publish (xem ghi chú #2). Đã ghi rõ lý do trong docstring `base.py`.
