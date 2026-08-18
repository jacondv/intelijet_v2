# Kế hoạch viết Manual vận hành Intelijet v3

> Thư mục này (`docs/manual_plan/`) chứa **kế hoạch/brief** cho từng chương của manual — dùng để giao cho agent AI viết nội dung thật.
> Nội dung manual thật (sản phẩm cuối) sẽ được viết vào `docs/manual/` — xem khung sườn các file đã tạo sẵn ở đó.
>
> Ngày tạo: 2026-08-18

---

## 0. Mục tiêu & đối tượng đọc

Manual này dành cho **người vận hành hiện trường** (operator dùng máy quét mỗi ngày) và **kỹ thuật viên bảo trì/hỗ trợ** (cài đặt, xử lý sự cố, cập nhật). Không phải tài liệu cho lập trình viên — không giả định người đọc biết ROS, Python, hay Docker. Tài liệu kiến trúc/refactor cho dev đã có riêng ở `docs/optimization_plan_overview.md` và `docs/plan/`; **không lặp lại nội dung đó ở đây**.

**Trọng tâm của manual = an toàn khi vận hành + xử lý sự cố khi hệ thống hoạt động không đúng.** Đây là 2 lý do chính khiến operator cần mở manual ra đọc trong thực tế (không phải để tra "nút này tên gì"). Mọi chương khác đều phục vụ 2 mục tiêu này — viết vừa đủ, không viết dài dòng chỉ để "cho đủ mục".

**Không đi sâu kỹ thuật/phần mềm.** Operator không cần biết ROS, container, thuật toán, hay tên hàm/file code. Manual chỉ nói: *hiện tượng gì* → *cần làm gì*. Ví dụ mức độ mong muốn (do người dùng cung cấp, dùng làm mẫu cho toàn bộ manual):

> Housing (nắp che đầu quét) được đóng/mở bằng động cơ điện (DC motor). Khi đang đóng/mở, **có thể kẹp tay** nếu đưa tay vào vùng chuyển động. Nếu bị kẹt: **ngắt nguồn điện của Housing ngay**, sau đó có thể **di chuyển Housing bằng tay** để lấy vật/tay ra.

Đây là văn phong chuẩn cần theo: ngắn, cụ thể, không giải thích cơ chế bên trong (không cần nói động cơ loại gì, điều khiển qua đâu), chỉ cần **nguy cơ + việc phải làm ngay**.

**Bổ sung phần cứng quan trọng (2026-08-18, không có trong code, do người dùng cung cấp trực tiếp):** hệ thống còn có một **cánh tay thủy lực (hydraulic arm)** — 2 khớp + 1 xy-lanh (cylinder) — dùng để đưa Housing (chứa đầu quét) tới gần vị trí cần quét, giúp chất lượng dữ liệu tốt hơn. Đây là bộ phận cơ khí **không hề xuất hiện trong code/tài liệu đã khảo sát trước đó** — trong code chỉ thấy lệnh "Extend/Retract speed" đi kèm lệnh mở/đóng Housing (`ui_can_interface/src/ui_can_interface/command_handler.py`), rất có thể đây chính là cơ cấu điều khiển cánh tay này (mở Housing ~ tay duỗi ra, đóng Housing ~ tay thu về) — **cần xác nhận lại với người dùng** quan hệ chính xác giữa "Housing đóng/mở" (ví dụ DC motor, mục an toàn đã có) và "cánh tay thủy lực duỗi/thu" có phải cùng một chuyển động hay là 2 cơ cấu tách biệt, vì ảnh hưởng trực tiếp tới cách viết chương 02 (an toàn) và chương 06 (SOP). Đã thêm câu hỏi này vào brief chương 02 và 06 — hỏi lại người dùng trước khi viết 2 chương đó.

**Ngôn ngữ: nội dung manual cuối cùng (`docs/manual/*.md`) viết 100% bằng tiếng Anh** (quyết định 2026-08-18, không phải tiếng Việt như giả định ban đầu). Tài liệu kế hoạch trong `docs/manual_plan/` (thư mục này) vẫn giữ tiếng Việt — đây là tài liệu nội bộ giữa người chuẩn bị kế hoạch và agent viết, không phải sản phẩm cuối. **Agent viết nội dung phải luôn nhớ: đọc brief tiếng Việt để hiểu yêu cầu, nhưng viết file trong `docs/manual/` bằng tiếng Anh — không lẫn tiếng Việt vào bản viết ra**, kể cả tên bộ phận/nút bấm (dịch sang tiếng Anh phù hợp, hoặc giữ nguyên tên nhãn UI nếu UI hiện đang là tiếng Anh — cần kiểm tra khi viết chương 05).

---

## 1. Nguyên tắc bắt buộc khi viết (đọc trước khi giao agent)

1. **Không bịa thông tin an toàn vật lý hoặc thông số phần cứng không có trong code/tài liệu sẵn có.** Agent AI chỉ đọc được source code và tài liệu text — nó **không biết** điện áp thực tế, class laser chính xác của đầu SICK đang lắp, điểm kẹp cơ khí của housing khi vận hành thật, quy trình khóa/gắn thẻ (LOTO) tại hiện trường, v.v. Mọi chỗ như vậy phải được đánh dấu rõ bằng khối:

   ```
   > ⚠️ **CẦN XÁC NHẬN TỪ NGƯỜI VẬN HÀNH/KỸ THUẬT** — <câu hỏi cụ thể>
   ```

   thay vì đoán hoặc viết chung chung kiểu "hãy cẩn thận". Xem mục 4 bên dưới — danh sách các câu hỏi loại này đã được liệt kê sẵn theo từng chương, agent chỉ cần chèn đúng chỗ, **không tự trả lời thay**.
2. **Ưu tiên mô tả hành vi phần mềm thực tế** (dựa trên code hiện tại), không mô tả hành vi "lẽ ra nên có" hoặc tính năng đang trong kế hoạch tối ưu (`docs/plan/`) nhưng chưa merge. Nếu không chắc một hành vi đã được triển khai hay còn đang dở dang, kiểm tra code trước khi viết, không suy đoán.
3. **Văn phong**: câu ngắn, bước đánh số rõ ràng (1, 2, 3…), dùng đúng tên nút/nhãn xuất hiện trên UI thật (lấy từ các file `*_ui.py`/`.ui`, không tự đặt tên khác). Mỗi quy trình thao tác nên có chỗ chèn ảnh chụp màn hình thật (đánh dấu placeholder `[ẢNH: mô tả]`, chưa có ảnh thật ở đợt chuẩn bị này).
4. **Nội dung cuối cùng KHÔNG được chứa jargon kỹ thuật/phần mềm**: không tên file code, tên hàm, tên topic ROS, tên class, thuật ngữ Docker/container/thread... Các thứ này chỉ dùng để **agent tự tra cứu và hiểu đúng hành vi**, không phải thứ operator cần đọc. Viết theo công thức đơn giản: **hiện tượng operator thấy → nguyên nhân bằng ngôn ngữ đời thường (nếu cần) → việc cần làm**. Ví dụ SAI: "nếu topic `/pcan_received_messanges` không nhận được frame trong 3s, `PCANMonitor` sẽ báo DISCONNECTED". Ví dụ ĐÚNG: "nếu đèn trạng thái CAN chuyển đỏ, kiểm tra dây CAN có bị lỏng/đứt không".
5. **Mỗi chương ghi rõ nguồn** (đường dẫn file code/cấu hình đã tham khảo) ở cuối dưới dạng mục "Nguồn tham khảo (nội bộ, không in trong bản operator)" — để người review đối chiếu, nhưng nên tách riêng khỏi phần nội dung chính, không lẫn vào giữa bài.
6. Nếu một tính năng có 2 cách triển khai song song trong code (ví dụ pipeline convert point cloud ở cả `helper.py` và `data_converter.py`, xem `docs/optimization_plan_overview.md` mục 2) — **mô tả theo cái đang thực sự được gọi trong luồng chạy chính** (`ui/scripts/app.py`), không mô tả cả hai như thể tương đương.

---

## 2. Cấu trúc thư mục

```
docs/
  manual_plan/          <- kế hoạch (thư mục này) — brief cho từng chương
    00_INDEX.md          <- file này
    01_brief_gioi_thieu.md
    02_brief_an_toan.md
    ...
  manual/                <- sản phẩm cuối, agent viết nội dung thật vào đây
    00_muc_luc.md         <- mục lục, đã tạo sẵn
    01_gioi_thieu.md       <- khung sườn, agent điền nội dung
    02_canh_bao_an_toan.md
    ...
    assets/                <- ảnh chụp màn hình, sơ đồ (chưa có ảnh thật)
```

Quy ước đặt tên: file brief trong `manual_plan/` và file nội dung tương ứng trong `manual/` dùng cùng số thứ tự (`02_brief_an_toan.md` ↔ `02_canh_bao_an_toan.md`) để dễ đối chiếu.

---

## 3. Danh sách chương & mức ưu tiên

| # | Chương | Ưu tiên | Ghi chú nguồn thông tin |
|---|--------|---------|--------------------------|
| 01 | Giới thiệu hệ thống | Thấp — nên rất ngắn (nửa trang) | Đầy đủ trong code + `docs/optimization_plan_overview.md` |
| 02 | Cảnh báo an toàn | **Cao nhất** | ⚠️ Phần lớn **cần xác nhận từ con người**, code không có |
| 03 | Cài đặt lần đầu | Trung bình | Có sẵn gần như đầy đủ (`docs/INSTALL.md`, `install.sh`) |
| 04 | Khởi động & tắt hệ thống hằng ngày | Cao | Có sẵn (`run_docker.sh`, nút Exit trong UI) |
| 05 | Tổng quan giao diện người dùng | Trung bình — chỉ mô tả tối thiểu cần để nhận biết lỗi | Cần đọc code UI để hiểu đúng, nhưng viết ra rất gọn |
| 06 | Quy trình vận hành hằng ngày (SOP quét) | **Cao nhất** | Cốt lõi của manual — cần đọc kỹ `pps/`, `ui/scripts/app.py` |
| 07 | Giám sát thiết bị & hệ thống cảnh báo (đèn/màu trạng thái) | **Cao nhất** | Có sẵn khá đầy đủ (`devices.yaml`, `notification_center.py`) — nền tảng cho chương 08 |
| 08 | Xử lý sự cố | **Cao nhất** | Trọng tâm số 2 của manual theo yêu cầu người dùng — suy ra từ mã cảnh báo + vấn đề đã biết |
| 09 | Bảo trì & cập nhật phần mềm | Trung bình | Có sẵn (`docs/INSTALL.md` phần cập nhật) |
| 10 | Đồng bộ dữ liệu 2 tablet (Syncthing) | Thấp | Có sẵn gần đầy đủ trong `intallation_gui.md` |
| 11 | USB Data Copier | Thấp | Có sẵn gần đầy đủ trong `intallation_gui.md` |
| 12 | Phụ lục (thông số kỹ thuật, sơ đồ, liên hệ hỗ trợ) | Thấp | ⚠️ Phần lớn **cần xác nhận từ con người** |

**Trọng tâm thật sự của manual: chương 02 (an toàn), 06 (quy trình quét), 07+08 (đèn trạng thái → xử lý sự cố).** Các chương còn lại chỉ cần đủ dùng, viết ngắn gọn, không mở rộng thêm.

Xem brief chi tiết từng chương trong file `NN_brief_*.md` tương ứng.

---

## 4. Câu hỏi đã chốt (trả lời 2026-08-18)

1. **Định dạng xuất bản**: Markdown làm gốc + xuất PDF (để in dán tại hiện trường, ví dụ trang an toàn/xử lý sự cố).
2. **Nguồn thông tin an toàn vật lý**: người dùng (chủ dự án) trả lời trực tiếp qua chat khi được hỏi — không cần chờ tài liệu nhà sản xuất hay khảo sát hiện trường riêng.
3. **Ảnh/sơ đồ minh họa**: dùng placeholder `[ẢNH: mô tả]` trước, ảnh thật sẽ được gửi sau để chèn vào.
4. **BLK360G2**: **bỏ hẳn, không nhắc gì** trong toàn bộ manual (kể cả phụ lục) — khác với dự tính ban đầu là "nhắc 1 câu ở phụ lục". `docs/manual_plan/12_brief_phu_luc.md` đã được sửa lại theo quyết định này.
5. **Model laser/lidar thật đang lắp**: **SICK LMS511** (khớp với cấu hình trong code) — không phải dòng TiM. Dùng datasheet/class laser chính thức của LMS511 khi viết cảnh báo laser ở chương 02 (agent vẫn cần tra thêm class laser cụ thể của model này — người dùng xác nhận model, chưa xác nhận class laser chính xác).
6. **Dừng khẩn cấp — ĐÃ CHỐT (2 công tắc khác nhau, không được nhầm)**: bộ PPS/phần mềm không có E-Stop riêng (nút Exit chỉ dừng phần mềm). Nút **"emergency"** trên xe nền chỉ **tắt động cơ + cắt nguồn thủy lực — điện vẫn còn**, không phải cắt điện hoàn toàn. Muốn cắt điện hoàn toàn (an toàn để chạm Housing / bắt buộc trước khi bảo trì) phải **tắt Ignition hoặc ngắt mass (battery master switch) của xe nền** — đây cũng chính là công tắc ngắt điện cho Housing (không có công tắc riêng cho Housing).
7. **Ngắt điện khi bảo trì**: bắt buộc ngắt **nguồn điện tổng của cả máy** trước khi bảo trì/tháo lắp Housing hay bộ phận điện — không chỉ ngắt riêng Housing.
8. **Môi trường vận hành**: có yếu tố không gian kín/hạn chế (tunnel/đường ống) — cần lưu ý thông gió/không gian hạn chế trong chương 02.
9. **Cánh tay thủy lực (2 khớp + 1 xy-lanh, đưa Housing tới gần vị trí quét)** — bổ sung 2026-08-18, KHÔNG có trong code:
   - Là bộ phận chuyển động **độc lập** với Housing đóng/mở (2 vùng nguy hiểm khác nhau).
   - Nguồn thủy lực tắt khi máy tắt (qua nút "emergency" trên xe nền off-highway).
   - **KHÁC QUAN TRỌNG so với Housing: cánh tay KHÔNG tự xả áp khi tắt nguồn** — vẫn có thể giữ áp suất, không an toàn để can thiệp bằng tay ngay như Housing. Cần một bước xả áp riêng.
   - ⚠️ **Còn thiếu, người dùng sẽ tự cập nhật sau**: quy trình xả áp cụ thể — **không được tự bịa ra các bước xả áp**, giữ nguyên khối CẦN XÁC NHẬN cho tới khi có quy trình thật.

10. **Laser SICK LMS511 — Class 1** (an toàn cho mắt, xác nhận trực tiếp từ người dùng) — không cần PPE mắt riêng cho đầu quét này.

## 5. Câu hỏi còn mở — chưa hỏi hoặc chưa trả lời

Còn thiếu (chương 02 trừ những mục đã chốt ở trên): PPE yêu cầu, thông tin liên hệ khi có tai nạn, khoảng cách an toàn khi bắt đầu quét, yêu cầu cụ thể khi làm việc trong không gian kín. Các chương khác: thông tin liên hệ hỗ trợ kỹ thuật, ý nghĩa nghiệp vụ pre-scan/post-scan, danh sách lỗi thực tế thường gặp, lịch bảo trì phần cứng, chính sách giữ dữ liệu.

---

## 6. Quy trình giao việc cho agent viết manual

Gợi ý: giao theo từng chương (hoặc nhóm 2-3 chương liên quan) trong các phiên riêng, không giao "viết hết một lần" — mỗi chương cần đọc code khác nhau, gộp chung dễ bỏ sót hoặc lẫn lộn.

1. Chốt các câu hỏi ở mục 4 trước (ít nhất câu 1, 2, 5, 6 — ảnh hưởng nội dung, không chỉ hình thức).
2. Với mỗi chương: đưa agent brief tương ứng (`manual_plan/NN_brief_*.md`) + nhắc agent đọc nguyên tắc ở mục 1 của file này.
3. Agent viết nội dung vào `manual/NN_*.md` (đã có khung sườn sẵn), giữ nguyên các khối `⚠️ CẦN XÁC NHẬN` chưa có câu trả lời — không tự xóa hay tự trả lời thay.
4. **Bắt buộc review chương 02 (an toàn) và phần thông số kỹ thuật ở chương 12 với người có chuyên môn/vận hành thực tế trước khi phát hành** — đây là phần rủi ro cao nhất nếu sai (an toàn con người), không phát hành chỉ dựa vào bản do AI viết.
5. Sau khi tất cả chương có nội dung, review chéo một lượt cho nhất quán thuật ngữ (ví dụ luôn gọi housing/scanner/job bằng đúng 1 tên xuyên suốt, khớp với tên nút thật trên UI).

---

## 7. Không thuộc phạm vi manual này

- Tài liệu kiến trúc code, kế hoạch refactor, task breakdown cho dev — đã có ở `docs/plan/` và `docs/optimization_plan_overview.md`.
- Driver/tích hợp đầu scan BLK360G2 (chưa dùng thật — xem câu hỏi 6 ở mục 4).
- Tài liệu API nội bộ ROS (topic/service/action nội bộ) — chỉ nhắc tên khi cần giải thích cho troubleshooting, không viết thành tài liệu tham chiếu API đầy đủ.

---

## 8. Nguồn đã khảo sát khi lập kế hoạch này

`docs/optimization_plan_overview.md`, `docs/INSTALL.md`, `intallation_gui.md`, `install.sh`, `run_docker.sh`, `run_intelijet.sh`, `docker-compose.yml`, `intelijet_v2_ws/src/config/{devices.yaml,commond.yaml}`, `intelijet_v2_ws/src/ui/src/ui/notification_center.py`, `intelijet_v2_ws/src/pps/` (cấu trúc + `generic_scan_controller.py`), `intelijet_v2_ws/src/sick_scan/README.md`, cấu trúc `intelijet_v2_ws/src/*` (danh sách package), `intelijet_usbcopier/`, `data/` (cấu trúc runtime).
