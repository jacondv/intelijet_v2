# Kế hoạch tổng quát tối ưu Intelijet v2

> **➡ Kế hoạch chi tiết để triển khai (9 phase, mỗi phase = 1 phiên làm việc của Claude Sonnet) nằm ở `docs/plan/` — bắt đầu từ `docs/plan/00_INDEX.md`.** Tài liệu này là bản tổng quát (bối cảnh, lý do, mục tiêu); khi triển khai, làm theo bộ file trong `docs/plan/`.
>
> Tài liệu này là bản kế hoạch **tổng quát** (high-level), dùng làm đầu vào để Fable 5 xây dựng kế hoạch **chi tiết** (thiết kế, task breakdown, ước lượng). Không chứa code, chỉ mô tả phạm vi, hiện trạng, mục tiêu và thứ tự ưu tiên.
>
> Phạm vi module: `pps`, `shared`,`ui`,`config`,  (trong `intelijet_v2_ws/src/`), cộng thêm hạ tầng triển khai (Docker/desktop launcher).
>
> Ngày tạo: 2026-08-09

---

## 0. Bối cảnh & hiện trạng chung

Dự án ROS1 Noetic + PyQt5 (giao diện chạm/kiosk) + Open3D/VTK (xử lý & hiển thị point cloud) + WeasyPrint (xuất PDF report), chạy trong Docker trên máy Linux tại hiện trường, khởi động qua icon Desktop gọi `run_docker.sh`.

Khảo sát 3 module cho thấy các vấn đề lặp lại nhất quán:
- **Code trùng lặp / code chết** tồn đọng nhiều nơi (nhiều bản gần giống nhau của cùng một logic, file cũ không còn được dùng nhưng chưa xoá, khối code comment lớn).
- **God object / god module**: vài class/file gánh quá nhiều trách nhiệm (UI + business logic + I/O trộn lẫn), khó test, khó sửa an toàn.
- **Blocking trên GUI thread**: pipeline xử lý point cloud tự động và xuất report chạy đồng bộ ngay trên luồng giao diện → gây đơ ("đứng hình") khi thao tác.
- **Giám sát thiết bị phức tạp không cần thiết**: logic hiện tại đã gần với yêu cầu đơn giản hoá của người dùng ở phần CAN, nhưng phần Lidar dùng sai tiêu chí (suy ra kết nối từ dữ liệu quét thay vì kiểm tra khả năng kết nối thực).
- **Hạ tầng triển khai thủ công**: Dockerfile/run script viết tay, chưa dùng `docker compose`, quy trình cài đặt/khởi động/tắt còn thô.

---

## 1. Mục tiêu tối ưu

1. **Hiệu năng & trải nghiệm UI**: loại bỏ hoàn toàn tình trạng giao diện đơ khi xử lý point cloud và xuất report; giảm giật/lag khi render; giảm số bước thao tác không cần thiết.
2. **Đơn giản hoá giám sát thiết bị & chuyên nghiệp hoá cảnh báo**: CAN = còn nhận gói tin gần đây; Lidar = ping tới IP thành công. Bỏ các lớp logic thừa, đồng thời đưa toàn bộ cảnh báo/alarm lên UI theo một hệ thống thông báo rõ ràng, nhất quán.
3. **Code sạch hơn**: loại bỏ code thừa/chết/trùng lặp, tách các god object/god module thành các phần có trách nhiệm rõ ràng, dễ bảo trì và test.
4. **Hạ tầng triển khai qua Docker Compose**: cài đặt nhanh, nhất quán, khởi động từ icon Desktop, có nút Exit đáng tin cậy để tắt toàn bộ hệ thống (container + tiến trình liên quan) gọn gàng.
5. **Kiến trúc sẵn sàng thay thế đầu scan 3D**: tách phần điều khiển housing/quy trình quét ra khỏi giả định "SickScan 2D quét nhiều lớp rồi ghép thành 3D", để sau này thay bằng một đầu scan trả trực tiếp point cloud 3D (ví dụ Leica BLK) mà không phải viết lại toàn bộ `pps`.

Mục tiêu cốt lõi xuyên suốt toàn bộ kế hoạch: **ứng dụng chạy ổn định** — mọi đề xuất bên dưới nên được cân nhắc trước tiên theo tiêu chí này, hơn là theo hướng thêm tính năng mới.

Không đặt mục tiêu: đổi framework GUI, đổi hệ điều hành robot (ROS1), thay đổi thuật toán xử lý point cloud hiện có (trừ khi cần để hỗ trợ threading), tự triển khai driver cho đầu scan mới ngay trong đợt này (chỉ chuẩn bị kiến trúc để việc đó dễ làm sau).

---

## 2. Hạng mục A — Sửa tình trạng giao diện bị đơ (ưu tiên cao nhất)

### Hiện trạng
Toàn bộ chuỗi xử lý sau khi nhận cloud mới — convert point cloud, tô màu, dựng lại actor VTK, ghi file PLY, tính toán bề dày (dựng mesh), vẽ biểu đồ, render ảnh 3D offscreen, và xuất PDF (WeasyPrint) — chạy **tuần tự, đồng bộ, trên chính GUI thread** (`ui/scripts/app.py`, hàm `on_cloud_received` → `export_report`). Không có luồng nền nào cho pipeline tự động này.

Ngoài ra còn các điểm gây giật/lag phụ:
- VTK dựng lại toàn bộ actor và box widget từ đầu ở mỗi lần nhận cloud mới, không giới hạn/giảm mẫu số điểm trước khi render.
- Combobox chọn job đọc lại file JSON và dựng lại toàn bộ danh sách mỗi lần người dùng mở dropdown.
- Có 2 pipeline convert/point-cloud song song (trong `pps/helper.py` và `pps/data_converter.py`) không thống nhất, gây khó kiểm soát hiệu năng.

### Mục tiêu
- Người dùng có thể tiếp tục thao tác (xoay/zoom point cloud, bấm nút, xem trạng thái) trong lúc hệ thống đang lưu file, tính toán và xuất report ở nền.
- Không giảm chất lượng report/point cloud hiển thị — chỉ thay đổi *nơi* các tác vụ nặng được thực thi.

### Hướng tiếp cận (để Fable 5 thiết kế chi tiết)
- Xem xét giới hạn/giảm mẫu (downsample) số điểm trước khi đẩy vào VTK để giảm chi phí render.
- Tránh dựng lại toàn bộ box widget VTK mỗi khung hình; chỉ cập nhật vị trí khi cần.
- Đưa việc đọc/ghi file cấu hình, job list ra khỏi đường thao tác nóng (hot path) của UI, hoặc cache hợp lý thay vì đọc lại đĩa mỗi lần mở dropdown.
- Cân nhắc thống nhất một API convert point cloud duy nhất (loại bỏ trùng lặp giữa `helper.py` và `data_converter.py`) để dễ tối ưu về sau.

---

## 3. Hạng mục B — Đơn giản hoá giám sát thiết bị (CAN, Lidar) & chuyên nghiệp hoá cảnh báo/alarm trên UI

> Gộp 2 việc vì cùng một mạch dữ liệu: đơn giản hoá **nguồn** (logic xác định trạng thái thiết bị) phải đi kèm làm lại **đích** (cách hiển thị cảnh báo/alarm đó cho người vận hành) — nếu chỉ sửa nguồn mà giữ nguyên cách hiển thị hiện tại (nhãn text đơn giản, dễ bị ghi đè — xem thêm hiện trạng bên dưới) thì người dùng vẫn không thấy rõ giá trị của việc đơn giản hoá.

### Hiện trạng — logic xác định trạng thái thiết bị
Logic nằm ở `shared/device_monitor.py`, điều khiển qua `devices.yaml`:
- **CAN** (`PCANMonitor`, `EncoderMonitor`, `PLCMonitor`): đã đúng tinh thần "còn nhận gói tin CAN trong N giây gần nhất = connected" — chỉ khác là bị cài đặt lặp lại 3 lần với code giống hệt nhau thay vì dùng chung một lớp.
- **Lidar** (`LidarMonitor`): hiện đang suy ra kết nối từ việc **có dữ liệu quét (point cloud) đang chảy về hay không**, chứ không phải kiểm tra khả năng kết nối mạng tới thiết bị. Hệ quả: Lidar sẽ báo "mất kết nối" bất cứ lúc nào không có lệnh quét đang chạy, dù phần cứng vẫn hoạt động bình thường — nhiều khả năng đây là nguồn gốc của các cảnh báo/lỗi vặt gây khó chịu.
- Không có cơ chế ping (ICMP/TCP) nào cho Lidar tồn tại sẵn trong codebase — đây sẽ là phần **viết mới**, không phải chỉnh sửa từ cái có sẵn.
- Phát hiện phụ: một hàm cập nhật trạng thái (`update_status`) đang bị gọi thiếu tham số ở mọi nơi, khiến một phần trạng thái chi tiết (`process_state`, `detail`) không bao giờ thực sự được cập nhật — nên sửa cùng đợt.
- Cơ chế chọn class giám sát theo tên trong file cấu hình đang dùng `eval()` — không an toàn, nên đổi sang tra cứu bằng bảng ánh xạ (dict).

### Hiện trạng — hiển thị cảnh báo/alarm trên UI
- Trạng thái thiết bị (CAN/Lidar/Encoder/PLC) hiện chỉ hiển thị dưới dạng text đơn giản trên các label riêng lẻ (`lblEncoderStatus`, `lblLidarStatus`, `lblPCANStatus`, `lblPLCStatus`), không có phân cấp mức độ (info/warning/error), không có màu sắc/icon nhất quán theo mức độ nghiêm trọng.
- Kênh thông báo lỗi/log vận hành (`log_status`/`unpack_log_status`, cơ chế "giả pub/sub" qua `/rosout`) đổ chung vào **một label duy nhất** trên giao diện — thông báo mới ghi đè thông báo cũ ngay lập tức, không có lịch sử, không phân biệt mức độ nghiêm trọng bằng màu sắc/icon. Một cảnh báo quan trọng (ví dụ mất kết nối Lidar) có thể bị một dòng log thông thường ghi đè trước khi người vận hành kịp đọc.
- Không có nơi nào tổng hợp "tất cả cảnh báo đang active" để người vận hành xem lại — mỗi lỗi chỉ thoáng qua rồi mất.

### Mục tiêu
- **CAN connected** = còn nhận được gói tin CAN trong khoảng thời gian gần đây (giữ nguyên tiêu chí, chỉ gộp code trùng lặp).
- **Lidar connected** = ping tới IP cấu hình của Lidar thành công (thay đổi hành vi thực sự, không phải suy ra từ dữ liệu quét).
- Loại bỏ các lớp giám sát trùng lặp, thay bằng cấu trúc chung, dễ mở rộng thêm thiết bị khác sau này nếu cần.
- Có một **khu vực/panel cảnh báo chuyên nghiệp** trên UI: hiển thị trạng thái từng thiết bị rõ ràng bằng màu/icon theo mức độ (bình thường/cảnh báo/lỗi), giữ lại lịch sử cảnh báo gần đây (không bị mất khi có thông báo mới), và tách biệt cảnh báo quan trọng khỏi log/thông báo trạng thái thông thường.

### Hướng tiếp cận
- Gộp `EncoderMonitor`/`PCANMonitor`/`PLCMonitor` thành một lớp giám sát theo kiểu "còn nhận tin trong N giây" dùng chung, tham số hoá qua `devices.yaml`.
- Thêm cơ chế theo dõi Lidar bằng ping (không chặn luồng ROS timer — chạy ping ở nền/kèm timeout ngắn), cần bổ sung trường IP của Lidar vào cấu hình thiết bị.
- Thay `eval()` bằng bảng tra cứu tên lớp.
- Sửa lỗi thiếu tham số ở `update_status`.
- Thiết kế lại khu vực hiển thị trạng thái thiết bị/cảnh báo trên UI: chuẩn hoá theo 1 mô hình mức độ (severity) dùng chung cho cả trạng thái thiết bị lẫn log/thông báo hệ thống, thay cho việc mỗi nơi tự quy ước một kiểu hiển thị.
- Cân nhắc thêm một panel/khu vực "lịch sử cảnh báo" (danh sách ngắn các cảnh báo gần đây, có thể mở rộng xem chi tiết) thay cho label đơn lẻ bị ghi đè liên tục — nội dung này thay thế bullet "kênh thông báo" từng liệt kê riêng ở Hạng mục D, nay gộp về đây vì cùng một hệ thống hiển thị.

---

## 4. Hạng mục C — Dọn dẹp code thừa & tái cấu trúc

### Danh sách xoá (đã xác nhận không còn được import/launch ở đâu)
- `pps`: `keypoint_processing.py`, `keypoint_processing_v2.py` (~2000 dòng — chỉ bản `_v3` đang dùng), `sick_scan_controller.py` (đã thay bằng `sick_scan_eRob_controller.py`), `compare_cloud_action_server_old.py`, `scan_to_cloud_msg_node.py` (đã tắt trong file launch), `compare_method_m3c2.py` (chỉ còn 1 dòng import bị comment), `utils_report.py` (không có nơi gọi).
- `shared`: `config_manager.py` (hệ thống config song song, không ai dùng — hệ thống chính là `config_loader.py`).
- `ui`: `scripts/main.py` (entry point cũ, trùng ~70% logic với `app.py` đang chạy thật), `src/ui/intelijet_ui(old).py`.
- Toàn bộ các khối code comment lớn còn sót lại rải rác (device_monitor.py, hmi_scan_command_handler.py, sick_scan_controller.py, app.py, compare_pipeline.py, v.v.) — dọn theo từng khu vực khi refactor khu vực đó, không cần một đợt riêng.

> Lưu ý khi thực hiện: xác nhận lại (grep/tìm kiếm) trước khi xoá từng file để chắc chắn không có nơi nào còn tham chiếu, tránh xoá nhầm code đang dùng ở nhánh khác.

### Tái cấu trúc
- **`pps/helper.py`** (1688 dòng, ~30 hàm không liên quan): tách theo nhóm chức năng (thuật toán so sánh, crop/hình học, tô màu, PCA/biên dạng, I/O).
- **Thống nhất API convert point cloud**: chọn một trong hai (`helper.py` các hàm rời rạc, hoặc `data_converter.CloudConverter`) làm chuẩn duy nhất, loại bỏ bản còn lại.
- **`ui/scripts/app.py` — class `App`** (838 dòng, god object): tách UI thuần khỏi logic nghiệp vụ — ví dụ tách riêng: quản lý job/lưu file JSON, xuất report, xử lý point cloud — thành các service/class riêng có thể test độc lập, đồng thời là bước nền tảng bắt buộc để đưa các phần nặng vào worker thread ở Hạng mục A.
- Chuẩn hoá xử lý lỗi: thay các `print()`/khối `except: pass` im lặng bằng logging nhất quán (`rospy.log*`), đặc biệt các chỗ lỗi bị nuốt hoàn toàn khiến người dùng không biết report thất bại vì sao.

---

## 5. Hạng mục D — Trừu tượng hoá lớp điều khiển scanner (chuẩn bị thay thế SickScan)

> Mục này khác các mục khác ở chỗ: không sửa lỗi hiện có, mà **thay đổi kiến trúc để tránh nợ kỹ thuật lớn trong tương lai gần**. Người dùng đã xác nhận đây là mục quan trọng, cần Fable 5 hiểu rõ trước khi thiết kế chi tiết các phần khác trong `pps`, vì nó ảnh hưởng đến ranh giới module khi tái cấu trúc ở Hạng mục C.

### Bối cảnh
Hiện tại quy trình quét gắn chặt vào cách hoạt động riêng của SickScan (LiDAR 2D):
- Housing được mở **theo từng bước/góc quay tăng dần** (`_open_housing_sequence` trong `pps/generic_scan_controller.py`), đồng thời SickScan liên tục quét các lát cắt 2D (`LaserScan`).
- Khi kết thúc, dịch vụ `assemble_scans2` (gói `laser_assembler`) **ghép nhiều lát 2D thu được trong một khoảng thời gian** (`start_time` → `end_time`, gắn với lúc housing đang quay) thành một point cloud 3D duy nhất (`sick_scan_eRob_controller.py::run_workflow`).
- Nói cách khác: "3D" hiện nay là kết quả suy ra từ việc **kết hợp thời gian quay housing với dữ liệu 2D**, không phải dữ liệu 3D gốc từ cảm biến.

Dự án đã có sẵn scaffold cho một đầu scan 3D khác trong workspace `blk360g2_ws` (SDK + gói ROS `blk360g2_ros`) — cho thấy hướng thay thế bằng một scanner kiểu Leica BLK (hoặc tương tự) trả **trực tiếp point cloud 3D đầy đủ trong một lần chụp** đã được tính đến, nhưng chưa được tích hợp vào luồng nghiệp vụ chính (`pps`, `ui`).

Với loại scanner mới, quy trình sẽ đơn giản hơn về nguyên tắc nhưng **khác hẳn** về luồng điều khiển:
- Housing chỉ cần mở ra **một góc cố định** (không cần quay theo nhiều bước để "quét" như hiện tại).
- Scanner tự chụp và trả về point cloud 3D hoàn chỉnh — không cần bước ghép (`assemble_scans2`), không cần đồng bộ thời gian bắt đầu/kết thúc với chuyển động housing.

### Vấn đề với kiến trúc hiện tại nếu không chuẩn bị trước
`GenericScanController`/`HousingControl` hiện đang **gắn cứng** giả định "housing quay nhiều bước ăn khớp thời gian với dữ liệu 2D" ngay trong lớp điều khiển chung (`generic_scan_controller.py`), còn phần "ghép 2D → 3D" nằm trong lớp con `SickScanErobController`. Nếu sau này đổi sang scanner 3D trực tiếp mà không tái cấu trúc trước, nhiều khả năng phải viết lại phần lớn `pps` thay vì chỉ thêm một lớp điều khiển mới.

### Mục tiêu
- Tách rõ 2 mối quan tâm hiện đang trộn lẫn:
  1. **Chuyển động housing** (mở theo từng bước để quét vs. mở một góc cố định).
  2. **Cách lấy point cloud** (ghép nhiều lát 2D theo thời gian vs. nhận trực tiếp 1 point cloud 3D từ thiết bị).
- Định nghĩa một giao diện/điểm mở rộng chung (kiểu "chiến lược quét") mà `SickScanErobController` là một cách triển khai, để sau này thêm một triển khai khác (ví dụ dựa trên `blk360g2_ros`) chỉ cần viết một class mới tuân theo cùng giao diện, không đụng vào phần lõi điều khiển housing/luồng job/report.
- Không yêu cầu tích hợp thật driver Leica/BLK360G2 trong đợt tối ưu này — chỉ đảm bảo kiến trúc **sẵn sàng** cho việc đó.

### Hướng tiếp cận
- Khi tái cấu trúc `pps` ở Hạng mục C, thiết kế `GenericScanController` sao cho: kiểu chuyển động housing (nhiều bước / một góc cố định) và cách thu point cloud (ghép theo thời gian / nhận trực tiếp) đều là các thành phần có thể thay thế độc lập (ví dụ inject vào controller thay vì hard-code trong `run_workflow`), chứ không bắt buộc đổi cấu trúc lớp thừa kế mỗi khi đổi loại scanner.
- Tài liệu hoá rõ ràng ranh giới này để Fable 5 thiết kế `pps` theo hướng "thêm class mới khi đổi scanner", không phải "sửa lại code hiện có".
- Việc chuẩn hoá này nên làm **cùng đợt** với Hạng mục C (dọn dẹp/tách `helper.py`, hợp nhất API convert point cloud) vì đụng chung vùng code, tránh phải tái cấu trúc 2 lần.

---

## 6. Hạng mục E — Cải thiện trải nghiệm thao tác (UI/UX)

- Giảm số hộp thoại xác nhận (`QMessageBox`) cho các thao tác thường ngày trên màn hình cảm ứng (chuyển job, set home...) — hiện có tới 135 lần dùng dàn trải, gây cảm giác thao tác rườm rà.
- Khi tính năng auto-report bị tắt (qua 2 combobox điều kiện), hệ thống hiện im lặng không làm gì — cần hiển thị rõ trạng thái/thông báo để người dùng không nhầm tưởng bị lỗi (thông báo này nên đi qua cùng hệ thống cảnh báo/thông báo được làm lại ở Hạng mục B).
- Rà soát lại luồng chọn job (hiện có nhánh xử lý khác nhau khá rối giữa "chọn lại job hiện tại" và "chọn job khác") để thao tác nhất quán, dễ đoán hơn.

---

## 7. Hạng mục F — Chuyển sang Docker Compose & cải thiện vòng đời khởi động/tắt ứng dụng

### Hiện trạng
- `Dockerfile` ở gốc repo cài ROS Noetic + PyQt5 + Open3D + VTK + WeasyPrint, chạy `docker run` thủ công qua script `run_docker.sh`, mount thư mục project, X11, `--network host`, thiết bị `/dev/dri`.
- Icon Desktop (`intelijet.desktop`) gọi `run_docker.sh`: script tự kiểm tra container đã tồn tại/đang chạy hay chưa, tạo mới nếu cần, rồi exec vào script khởi động ứng dụng bên trong container (`run_intelijet.sh`).
- Nút Exit trên giao diện (`btnShutdown` → `on_shutdown` → `closeEvent`) hiện tắt bằng `rosnode kill -a` — chỉ dừng các node ROS, không quản lý vòng đời container Docker một cách rõ ràng; không chắc chắn dọn sạch mọi tiến trình liên quan mỗi lần thoát.
- Chưa có `docker-compose.yml` trong repo — mọi tham số (mount, biến môi trường, thiết bị) đang nằm rải rác trong script bash viết tay.

### Mục tiêu
- Cài đặt trên máy mới chỉ cần `docker compose up`/tương đương, không cần chỉnh sửa script thủ công.
- Icon Desktop khởi động toàn bộ hệ thống (bằng Compose) với một cú nhấp.
- Nút Exit trên app tắt gọn gàng **toàn bộ** hệ thống (ứng dụng, các ROS node, container liên quan) — không để lại tiến trình/container "mồ côi" chạy nền, và có thể khởi động lại sạch ở lần mở tiếp theo.

### Hướng tiếp cận
- Viết `docker-compose.yml` thay thế `docker run` thủ công trong `run_docker.sh`: khai báo service, volume mount (project dir, `/tmp/.X11-unix`, `/dev/dri`, `/etc/localtime`), biến môi trường (`DISPLAY`, `QT_X11_NO_MITSHM`), và các thiết lập tương đương `--network host`, `--cap-add=SYS_TIME` hiện có.
- Đơn giản hoá `run_docker.sh` (hoặc thay thế bằng script gọi `docker compose up -d` + attach) để icon Desktop chỉ cần gọi 1 script mỏng.
- Rà soát và cải thiện nút Exit: đảm bảo trình tự tắt rõ ràng — dừng các node ROS đúng cách → đóng ứng dụng Qt → dừng (hoặc giữ) container theo đúng ý đồ vận hành (cần thống nhất: Exit có nghĩa là chỉ đóng app, hay tắt luôn cả container?) — nên làm rõ yêu cầu này trước khi Fable 5 thiết kế chi tiết.
- Xem xét thêm cơ chế healthcheck/log tập trung của Docker Compose để dễ chẩn đoán khi hệ thống không khởi động được, giảm nhu cầu debug thủ công tại hiện trường.

> Câu hỏi cần chốt trước khi thiết kế chi tiết: khi người dùng bấm Exit, có nên tắt hẳn container (để lần sau khởi động lại từ đầu, sạch sẽ) hay chỉ đóng ứng dụng và giữ container chạy nền (khởi động lại nhanh hơn)? Quyết định này ảnh hưởng trực tiếp đến thiết kế script và nút Exit.

---

## 8. Thứ tự ưu tiên đề xuất

Nguyên tắc xếp thứ tự: ưu tiên cao nhất cho những gì ảnh hưởng trực tiếp đến **sự ổn định vận hành hằng ngày**; các thay đổi kiến trúc "chuẩn bị cho tương lai" (Hạng mục D) xếp cùng đợt với việc tái cấu trúc `pps` (Hạng mục C) vì đụng chung vùng code, nhưng không chặn các hạng mục khác.

1. **Hạng mục A** — Sửa đơ giao diện (ảnh hưởng trực tiếp và rõ rệt nhất đến trải nghiệm vận hành hằng ngày).
2. **Hạng mục B** — Đơn giản hoá giám sát CAN/Lidar + làm lại hiển thị cảnh báo trên UI (rủi ro thấp, giá trị cao, phần CAN gần như chỉ cần gộp code; phần hiển thị cảnh báo tác động trực tiếp đến cảm nhận "chuyên nghiệp" và độ tin cậy của hệ thống).
3. **Hạng mục C + D (làm cùng đợt)** — Dọn code thừa, tách god object, và tách kiến trúc scanner khỏi giả định SickScan. Gộp chung vì cùng đụng vào `pps`/`generic_scan_controller.py`; tách `App` cũng là nền tảng bắt buộc để đưa xử lý report vào thread nền ở Hạng mục A.
4. **Hạng mục F** — Chuyển Docker Compose + cải thiện Exit (độc lập về mặt kỹ thuật, có thể làm song song với các mục trên, nhưng cần chốt trước hành vi mong muốn của nút Exit).
5. **Hạng mục E** — Cải thiện UX thao tác (giá trị tốt nhưng không cấp thiết bằng các mục trên, có thể làm sau).

> Lưu ý: dù xếp sau về thời điểm bắt đầu, Hạng mục D (kiến trúc scanner) cần được **thiết kế/thống nhất trước** khi bắt tay tái cấu trúc chi tiết `pps` ở Hạng mục C — vì nó quyết định ranh giới class, tránh phải sửa lại lần hai.

---

## 9. Ghi chú cho Fable 5

- Đây là bản kế hoạch định hướng phạm vi và mức độ ưu tiên; **chưa** bao gồm: thiết kế chi tiết class/module, breakdown task, ước lượng thời gian, hay kế hoạch kiểm thử.
- **Mục tiêu cốt lõi xuyên suốt là sự ổn định vận hành** — khi phải đánh đổi giữa "làm nhanh, gọn" và "làm chắc, ổn định", ưu tiên phương án làm cho app ổn định, không bị lỗi.

- Trước khi xoá bất kỳ file "code chết" nào đã liệt kê ở Hạng mục C, cần xác nhận lại bằng tìm kiếm tham chiếu trong toàn repo (kể cả launch file, script cài đặt) để tránh xoá nhầm.
- Hạng mục A nên tách thành 2 bước: (1) tách business logic ra khỏi `App` (Hạng mục C) để có ranh giới rõ ràng, (2) đưa phần logic đó vào worker thread — làm thứ tự ngược lại sẽ khó kiểm soát hơn.
- Hạng mục D không yêu cầu tích hợp driver scanner mới thật sự trong đợt này — chỉ yêu cầu thiết kế `pps` sao cho việc thêm driver mới sau này là "thêm class", không phải "sửa lại kiến trúc". Nếu cần, có thể tham khảo scaffold sẵn có ở `blk360g2_ws/` (SDK + gói ROS `blk360g2_ros`) để hiểu hình dạng dữ liệu/API của loại scanner 3D tương lai, dù workspace đó hiện chưa được tích hợp vào `pps`/`ui`.
- Cần người vận hành/khách hàng xác nhận câu hỏi mở ở Hạng mục F (hành vi nút Exit) trước khi thiết kế chi tiết.


