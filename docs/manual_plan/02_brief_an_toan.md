# Brief 02 — Cảnh báo an toàn

**File output:** `docs/manual/02_canh_bao_an_toan.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Operator (bắt buộc đọc trước khi vận hành lần đầu)
**Mức ưu tiên:** CAO NHẤT — nhưng phần lớn nội dung KHÔNG THỂ viết chỉ từ code.

## Văn phong bắt buộc cho chương này

Mỗi mục an toàn viết theo đúng công thức, **không giải thích cơ chế kỹ thuật bên trong**:

> **Nguy cơ**: <điều gì có thể xảy ra, bằng ngôn ngữ đời thường>
> **Khi xảy ra, làm ngay**: <hành động cụ thể, ngắn>

Ví dụ mẫu đã được người dùng (chủ dự án) xác nhận trực tiếp — dùng làm khuôn cho các mục còn lại:

> **Housing (nắp che đầu quét) — nguy cơ kẹp tay**
> Housing đóng/mở bằng động cơ điện (DC motor). Không đưa tay vào vùng Housing đang chuyển động.
> Nếu bị kẹt: **ngắt nguồn điện của Housing ngay**, sau đó **có thể di chuyển Housing bằng tay** để lấy tay/vật ra.

Không thêm chi tiết kỹ thuật không cần thiết (loại động cơ, điều khiển qua đâu, thông số điện...) trừ khi operator thực sự cần biết để xử lý — mục tiêu là hành động đúng trong vài giây, không phải hiểu bản chất vấn đề.

## Trạng thái (2026-08-18): đã viết bản nháp

Chương đã có bản nháp đầy đủ tại `docs/manual/02_canh_bao_an_toan.md`, gồm cả 2 mục Housing và Cánh tay thủy lực, đã chốt: class laser (Class 1), phân biệt rõ nút "emergency" xe nền (chỉ tắt máy+thủy lực, điện vẫn còn) khác với tắt Ignition/ngắt mass (cắt điện hoàn toàn, cũng là cách ngắt điện Housing — không có công tắc riêng cho Housing). Còn mở: PPE (mục 7), liên hệ khi có tai nạn (mục 9), khoảng cách an toàn khi bắt đầu quét, yêu cầu cụ thể cho không gian kín (mục 8), quy trình xả áp cánh tay thủy lực (người dùng tự cập nhật sau). Các mục outline bên dưới giữ nguyên làm tài liệu tham khảo lịch sử — xem file `docs/manual/02_canh_bao_an_toan.md` để biết nội dung thật đang có.

## ⚠️ Cảnh báo cho agent viết chương này

Đây là chương rủi ro cao nhất nếu viết sai/thiếu: sai thông tin an toàn có thể dẫn đến chấn thương thật. **Agent KHÔNG được tự suy đoán hoặc "viết chung chung cho có"** các mục dưới đây khi không có thông tin xác thực. Với mỗi mục chưa có nguồn xác thực, viết đúng khối:

```
> ⚠️ **CẦN XÁC NHẬN TỪ NGƯỜI VẬN HÀNH/KỸ THUẬT** — <câu hỏi cụ thể còn thiếu>
```

Chương này **bắt buộc phải được người có chuyên môn/vận hành thực tế review trước khi phát hành**, không phát hành bản chỉ do AI viết mà chưa ai kiểm tra.

## Nguồn thông tin / code cần đọc (chỉ cho manh mối kỹ thuật, không đủ để kết luận an toàn)

- `intelijet_v2_ws/src/pps/src/pps/generic_scan_controller.py` — có housing chuyển động cơ khí (mở/đóng theo góc), có hàm dừng khẩn cấp cấp phần mềm (`_open_housing_sequence`, comment "Stop housing in case of emergency or end"). **Đây chỉ là dừng lệnh phần mềm, không phải nút dừng khẩn cấp vật lý** — cần xác nhận có/không có nút E-Stop vật lý riêng trên máy.
- `intelijet_v2_ws/src/sick_scan/README.md` — driver hỗ trợ cả dòng lidar "safety scanner" (TiM7xxS) và dòng thường (TiM7xx, không phải safety scanner); cấu hình thực tế đang trỏ tới `sick_lms_511.launch` (dòng LMS511) theo comment trong `config/devices.yaml`. **Chưa xác định được class laser/an toàn mắt chính xác của model đang lắp thật** — không được đoán class laser.
- `intelijet_v2_ws/src/config/devices.yaml` — hệ thống có PLC + CAN bus điều khiển chuyển động (housing, encoder) — gợi ý có nguồn điện động cơ/servo, nhưng thông số điện (điện áp, dòng) không có trong repo.
- `docker-compose.yml` — máy tính chạy Docker/PyQt5, không phải bản thân phần an toàn cơ khí.

## Nội dung cần có (outline) — khung chương, điền theo thông tin xác thực

1. Đối tượng phải đọc chương này trước khi dùng máy lần đầu.
2. Ký hiệu/biểu tượng cảnh báo dùng trong toàn manual (nếu công ty có chuẩn riêng — cần xác nhận, nếu không dùng ký hiệu chuẩn ISO thông thường: ⚠️ nguy hiểm chung, ⚡ điện, 🔦 laser, ⚙️ cơ khí).
3. **Nguy cơ cơ khí — Housing** (ĐÃ CÓ THÔNG TIN CƠ BẢN, dùng nguyên văn ví dụ mẫu ở trên): Housing đóng/mở bằng động cơ điện, nguy cơ kẹp tay khi đang chuyển động; xử lý = ngắt nguồn điện Housing rồi di chuyển bằng tay để lấy ra.
   > ⚠️ CẦN XÁC NHẬN THÊM (không bắt buộc để viết bản đầu tiên, nhưng nên hỏi khi review) — vị trí công tắc/cầu dao ngắt nguồn Housing cụ thể ở đâu trên máy; có cần đứng cách xa Housing khi bấm "bắt đầu quét" không.
3b. **Nguy cơ cơ khí — Cánh tay thủy lực (hydraulic arm)** — MỤC MỚI, ĐÃ CÓ THÔNG TIN QUAN TRỌNG (xác nhận 2026-08-18, do người dùng cung cấp trực tiếp, không có trong code):

   - Cánh tay thủy lực (2 khớp + 1 xy-lanh) đưa Housing tới gần vị trí cần quét, **là bộ phận chuyển động ĐỘC LẬP với việc Housing đóng/mở** (mục 3 ở trên) — 2 vùng nguy hiểm khác nhau, không phải cùng một chuyển động. Viết thành 2 mục cảnh báo riêng biệt, không gộp chung.
   - Nguồn thủy lực **tắt khi máy được tắt** (qua nút "emergency" trên chính chiếc xe/máy nền (off-highway) mà bộ PPS được lắp lên — xem thêm điểm mâu thuẫn cần làm rõ bên dưới).
   - ⚠️ **KHÁC BIỆT AN TOÀN QUAN TRỌNG SO VỚI HOUSING — PHẢI VIẾT ĐÚNG, KHÔNG ĐƯỢC SUY DIỄN:** cánh tay thủy lực **KHÔNG tự xả áp suất** khi tắt nguồn/tắt máy. Dù nguồn thủy lực đã tắt, xy-lanh/cánh tay **vẫn có thể giữ áp suất và không an toàn để đưa tay vào hay tự kéo/đẩy bằng tay ngay** — khác hẳn Housing (Housing an toàn di chuyển bằng tay ngay sau khi ngắt điện). **Cần một bước xả áp riêng** trước khi được coi là an toàn để can thiệp bằng tay.
   > ⚠️ CẦN XÁC NHẬN — **quy trình xả áp cụ thể** (các bước làm, van/nút xả áp ở đâu). Người dùng đã xác nhận sẽ tự cập nhật phần này sau — **agent TUYỆT ĐỐI không được tự viết ra các bước xả áp thay cho người dùng, kể cả đoán hợp lý** — để nguyên khối CẦN XÁC NHẬN cho tới khi nhận được quy trình thật. Trước khi có quy trình thật, chỉ viết: "không tự ý can thiệp vào cánh tay đang dừng do sự cố — gọi kỹ thuật viên/người có thẩm quyền xả áp trước khi chạm vào".
   > ⚠️ CẦN LÀM RÕ THÊM — điểm có vẻ chưa khớp với câu trả lời trước đó ("máy không có E-Stop vật lý riêng, chỉ có nút Exit phần mềm"): câu trả lời mới nhắc tới một nút **"emergency" trên chính xe/máy nền (off-highway) mang theo bộ PPS**. Cần hỏi lại người dùng để chốt: nút "emergency" này có phải là E-Stop vật lý của xe nền (khác với PPS, nhưng vẫn là công cụ dừng khẩn cấp thật operator có thể dùng) hay không? Nếu đúng, chương an toàn nên hướng dẫn operator dùng nút "emergency" trên xe nền làm hành động dừng khẩn cấp chính, thay vì chỉ nói "không có E-Stop".
4. **Nguy cơ laser/quang học** — đầu quét thật là **SICK LMS511** (đã xác nhận, không phải dòng TiM).
   > ⚠️ CẦN XÁC NHẬN THÊM — class laser cụ thể của LMS511 (tra datasheet SICK chính thức), có cần tránh nhìn trực tiếp / PPE mắt không.
5. **Nguy cơ điện** — **đã xác nhận: bắt buộc ngắt nguồn điện TỔNG của cả máy** (không chỉ Housing) trước khi bảo trì/tháo lắp bất kỳ bộ phận điện nào.
   > ⚠️ CẦN XÁC NHẬN THÊM — vị trí công tắc/cầu dao ngắt nguồn tổng cụ thể ở đâu trên máy; có cần chờ đèn/tín hiệu xác nhận đã ngắt điện an toàn không.
6. **Dừng khẩn cấp — CẦN LÀM RÕ THÊM 1 ĐIỂM TRƯỚC KHI VIẾT MỤC NÀY** (xem chi tiết mâu thuẫn ở mục 3b): bản thân bộ PPS/phần mềm **không có nút E-Stop vật lý riêng** — chỉ có nút "Exit" trên phần mềm, và nút Exit **không đảm bảo dừng ngay chuyển động cơ khí**. Tuy nhiên người dùng cũng nhắc tới một nút **"emergency" trên xe/máy nền (off-highway) mang theo bộ PPS**, và nút này cắt nguồn thủy lực. **Cần xác nhận lại**: nút "emergency" trên xe nền có phải là công cụ dừng khẩn cấp chính operator nên dùng không (viết rõ vị trí/cách dùng), hay đó là điều khiển riêng của xe nền không thuộc phạm vi vận hành PPS? Sau khi rõ, mục này phải nêu bật: cách dừng khẩn cấp thật sự = dùng nút "emergency" trên xe nền (nếu đúng là vậy) và/hoặc ngắt nguồn điện tổng (mục 5) — không phải bấm Exit rồi chờ. Đây là thông tin an toàn quan trọng nhất của cả chương — không được viết mờ nhạt hay lẫn vào giữa bài.
7. Trang bị bảo hộ cá nhân (PPE) yêu cầu khi vận hành/bảo trì, nếu có.
   > ⚠️ CẦN XÁC NHẬN.
8. Môi trường vận hành — **đã xác nhận: có không gian kín/hạn chế** (tunnel/đường ống). Cần cảnh báo về thông gió/không gian hạn chế.
   > ⚠️ CẦN XÁC NHẬN THÊM — có yêu cầu cụ thể nào khác đi kèm không gian kín không (ví dụ đo khí trước khi vào, tối thiểu 2 người, thiết bị chiếu sáng...).
9. Việc cần làm khi có sự cố/tai nạn (quy trình báo cáo, số điện thoại khẩn cấp nội bộ).
   > ⚠️ CẦN XÁC NHẬN.

## Ghi chú

- Không viết mục nào ở trên nếu không có câu trả lời — để nguyên khối "CẦN XÁC NHẬN", **không xóa và không tự điền phỏng đoán** dù chỉ để "cho đủ mục".
- Sau khi có câu trả lời thật, cập nhật lại chương này là bước bắt buộc trước khi phát hành manual, không phải việc "làm sau nếu có thời gian".
