# Brief 03 — Cài đặt lần đầu

**File output:** `docs/manual/03_cai_dat.md` — **viết bằng tiếng Anh 100%** (xem `00_INDEX.md` mục 0)
**Đối tượng đọc:** Kỹ thuật viên cài đặt máy mới
**Mức ưu tiên:** Trung bình — nội dung đã có sẵn gần như đầy đủ, chủ yếu là biên soạn lại cho dễ đọc hơn, giữ đúng lệnh/bước nhưng bỏ phần giải thích kỹ thuật không cần thiết cho kỹ thuật viên chạy lệnh (xem văn phong ở `00_INDEX.md` mục 1.4).

## Nguồn thông tin / code cần đọc

- `docs/INSTALL.md` — quy trình cài đặt chính thức hiện tại (yêu cầu máy, `./install.sh`, các bước nó làm, lưu ý đăng xuất/đăng nhập lại sau khi thêm user vào group `docker`, cách cập nhật code sau này qua `docker compose build`).
- `install.sh` — đọc để xác nhận chính xác các bước (cài Docker, tạo icon `.desktop`, build image 1 lần).
- `intallation_gui.md` phần đầu — có một số bước cài đặt môi trường phát triển (venv, catkin) **không dành cho máy vận hành thật** (dành cho máy dev) — không đưa nhầm vào manual vận hành, chỉ ghi chú "dành cho môi trường phát triển, không áp dụng cho máy hiện trường" nếu cần nhắc tới.

## Nội dung cần có (outline)

1. Yêu cầu trước khi cài: máy Linux Ubuntu/Debian có màn hình cảm ứng + X server, đã `git clone` repo.
2. Các bước cài đặt: chạy `./install.sh`, giải thích từng việc script làm (cài Docker, thêm user vào group docker, build image 1 lần, tạo icon Desktop).
3. Lưu ý bắt buộc: đăng xuất/đăng nhập lại (hoặc restart máy) sau khi được thêm vào group `docker` lần đầu, nếu không icon sẽ báo lỗi quyền.
4. Cách xác nhận cài đặt thành công (icon "Intelijet" xuất hiện, bấm chạy được).
5. Cập nhật code sau này: `git pull` rồi `docker compose build` — nhấn mạnh **install.sh không tự động rebuild**, đây là bước phải làm thủ công riêng.

## ⚠️ Cần xác nhận từ người vận hành/kỹ thuật

- Có quy trình cài đặt phần cứng (lắp housing, đấu dây CAN, cấp nguồn) đi kèm không, hay máy luôn được giao đã lắp sẵn phần cứng và chỉ cần cài phần mềm? Nếu có quy trình lắp đặt phần cứng, cần bổ sung thành chương riêng (không có trong code để tham khảo).
