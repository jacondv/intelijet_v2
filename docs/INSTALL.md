# Cài đặt trên máy Linux mới

## Yêu cầu

- Máy Linux (Ubuntu/Debian-based) có màn hình cảm ứng + X server.
- Đã `git clone` repo này về máy (installer không tự clone).

## Các bước

```bash
cd intelijet_v3      # thư mục vừa clone
./install.sh
```

`install.sh` sẽ:
1. Cài Docker + Docker Compose nếu máy chưa có (cần `sudo`, dùng script cài chính thức từ `get.docker.com`).
2. Thêm user hiện tại vào group `docker` nếu chưa có.
3. Build image app (build 1 lần duy nhất; các lần chạy sau qua icon dùng lại image này, KHÔNG rebuild).
4. Sinh icon "Intelijet" (trong menu ứng dụng + Desktop nếu có) trỏ đúng đường dẫn repo trên máy này.

Nếu bước 2 vừa thêm user vào group `docker` lần đầu, cần **đăng xuất/đăng nhập lại** (hoặc restart máy) để quyền group có hiệu lực trước khi bấm icon.

## Sau khi cài

Bấm icon **Intelijet** để chạy app. Mỗi lần bấm: container hiện tại (nếu có) bị dừng và một container mới được khởi động sạch từ image đã build — không rebuild, chỉ mất vài giây.

## Cập nhật code sau này

`install.sh` không tự cập nhật code hay rebuild. Sau khi `git pull`, muốn dùng code mới:

```bash
docker compose build   # rebuild image với code mới
```

Lần bấm icon tiếp theo sẽ dùng image vừa rebuild.
