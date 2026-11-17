# File compared (.ply) - ý nghĩa dữ liệu và quy tắc tính độ dày

Tài liệu cho phần mềm/bộ công cụ khác đọc file compared của Intelijet và muốn
ra **cùng số liệu** với report PDF của Intelijet.

## 1. File chứa gì

File `.ply` của cloud compare (tên dạng `job#YYYYMMDD_HHMMSS#type#scan.ply`) là
cloud **post-scan**, mỗi điểm có thêm:

| Trường | Ý nghĩa |
|---|---|
| `positions` | Toạ độ điểm, đơn vị mét |
| `distances` | Độ dày của điểm, đơn vị **mm**, **có dấu**, chưa lọc nhiễu |
| `colors` | Màu đã tô sẵn theo độ dày (chỉ để hiển thị) |

Timestamp trong tên file là thời điểm quét post-scan (múi giờ của tablet).

## 2. `distances` được tính thế nào

Với mỗi điểm post-scan: tìm điểm gần nhất trong prescan, rồi lấy khoảng cách từ
điểm post-scan đến mặt phẳng cục bộ của prescan tại điểm đó (point-to-plane),
đo dọc pháp tuyến. Pháp tuyến được hướng về phía cảm biến. Kết quả làm tròn mm.

**File không bị lọc nhiễu.** Điểm quá mỏng, quá dày hay âm đều còn nguyên.
Quy tắc lọc nằm trong code report, không có trong file - phần mềm khác phải
tự áp dụng (mục 3).

## 3. Quy tắc để ra cùng số liệu với report

Đầu vào: `target` (độ dày thiết kế, mm), `tolerance` (dung sai, mm) - hai giá trị
này lấy từ thông số của job, không có trong file PLY.
`max_measurable` = 300 mm (`thickness.max_measurable` trong `runtime.yaml`).

1. `d = |distances|` (lấy giá trị tuyệt đối).
2. Đếm `out_of_range` = số điểm có `d > max_measurable`.
3. Làm sạch: `d < 12` -> `d = 0` (không hư hỏng); `d > max_measurable` -> `d = 0`
   (ngoài tầm đo, nghi nhiễu hoặc prescan/postscan lệch). Điểm đúng bằng 300 được giữ.
4. Điểm **đạt**: `d > target - tolerance`. Điểm có `d = 0` không bao giờ đạt.
5. **Độ dày trung bình** = trung bình `d` của các điểm đạt; nếu có ít hơn 1000
   điểm đạt thì ghi 0.
6. **Diện tích tổng** = diện tích mesh BPA (bán kính 0.1 m và 0.15 m) dựng từ
   **toàn bộ điểm**, kể cả điểm ngoài tầm đo (bỏ chúng sẽ tạo lỗ, diện tích hụt).
7. **Diện tích đạt** = diện tích các tam giác của mesh đó mà phần lớn đỉnh
   (biểu quyết đa số) là điểm đạt.
8. **Thể tích** = diện tích đạt x độ dày trung bình / 1000 (m3).
9. **Biểu đồ phân bố**: vẽ trên `d` đã làm sạch, các cột chia bởi hai mốc
   `target - tolerance` và `target + tolerance`. Điểm không hư hỏng và điểm
   ngoài tầm đo đều bằng 0 nên cùng nằm ở cột nhỏ nhất.
10. **Dòng cảnh báo** dưới biểu đồ (chỉ hiện khi `out_of_range > 0`):
    `N points (P%) exceed the measurable range (300 mm) and were excluded from
    the thickness figures - possibly noise.`, với `P = out_of_range / tổng số điểm x 100`,
    làm tròn 1 chữ số. Tỉ lệ cao thường là dấu hiệu prescan/postscan lệch nhau.

## 4. Màu trong `colors` (chỉ để hiển thị)

`d > 300`: xanh nhạt (0.678, 0.847, 0.902) - ngoài tầm đo. `d < 20`: đỏ sẫm
(0.5, 0, 0). Từ 20 đến mức trên của `highlight_range` (mặc định 40): xanh lá.
Trên đó: xanh dương. Phần mềm khác nên tính lại từ `distances`, không suy ngược
từ màu.

## 5. Nguồn trong code

- Tính `distances`: `pps/src/pps/cloud_utils/comparison.py` (`run_compare`)
- Tô màu: `pps/src/pps/cloud_utils/coloring.py`
- Làm sạch và số liệu report: `services/src/services/report/report_utils.py`
  (`compute_thickness_metrics`)
- Diện tích: `pps/src/pps/cloud_utils/geometry.py` (`surface_area_split`)
