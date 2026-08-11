# Phase 8 — Docker Compose, icon desktop, nút Exit

> Đọc `docs/plan/00_INDEX.md` trước. Phụ thuộc: không (độc lập, làm lúc nào cũng được). Rủi ro: thấp (không đụng code nghiệp vụ, trừ nút Exit).
> Phạm vi file: `Dockerfile`, mới `docker-compose.yml`, `run_docker.sh`, `intelijet.desktop`, `run_intelijet.sh` (đọc để hiểu), `ui/scripts/app.py` (chỉ `on_shutdown`/`closeEvent`).

## Quyết định đã chốt (INDEX): nút Exit tắt TOÀN BỘ — app → ROS nodes → container. Lần mở sau khởi động sạch.

## Hiện trạng

- `Dockerfile` (gốc repo): image `osrf/ros:noetic-desktop-full-focal` + apt (PyQt5, VTK7, PCL, can-msgs...) + pip (`numpy==1.23.5`, `open3d==0.13.0`, `opencv-contrib-python`, `rosnumpy`). **KHÔNG được đổi version bất kỳ gói nào.**
- `run_docker.sh`: thêm user vào group docker, `xhost +local:docker`, `xrandr --output DSI-1 --rotate right` (xoay màn hình tablet — GIỮ), rồi 3 nhánh if: container đang chạy → exec; tồn tại nhưng dừng → start; chưa có → `docker run` với: mount `$HOME/intelijet_v2:/root/intelijet_v2`, `/etc/localtime`+`/etc/timezone` ro, `DISPLAY`, `QT_X11_NO_MITSHM=1`, `/tmp/.X11-unix`, `/dev/dri`, `--network host`, `--cap-add=SYS_TIME`, image `jacondv/jacon-pps-noetic:v2.1`, lệnh chạy `run_intelijet.sh` trong container.
- `intelijet.desktop`: Exec/TryExec trỏ đường dẫn cứng `/home/nuc/intelijet_v2/run_docker.sh`.
- Nút Exit: `btnShutdown` → `on_shutdown` (hỏi xác nhận) → `closeEvent` → `subprocess.call(["rosnode","kill","-a"])` → app thoát. Container do script bên ngoài quản, không được dọn chủ động.

## Việc 1 — `docker-compose.yml` (đặt ở gốc repo)

Một service `intelijet`, ánh xạ đúng 1:1 các tham số `docker run` hiện tại:
```yaml
services:
  intelijet:
    image: jacondv/jacon-pps-noetic:v2.1
    build: .                    # cho phép build lại từ Dockerfile khi cần
    container_name: intelijet
    network_mode: host
    cap_add: [SYS_TIME]
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - ${INTELIJET_HOME:-~/intelijet_v2}:/root/intelijet_v2
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /etc/localtime:/etc/localtime:ro
      - /etc/timezone:/etc/timezone:ro
    devices:
      - /dev/dri:/dev/dri
    command: bash -lc "/root/intelijet_v2/run_intelijet.sh"
    restart: "no"               # vòng đời do launcher + nút Exit quản, không tự restart
```
(Sonnet đối chiếu lại từng dòng với `run_docker.sh` thực tế — bảng trên là khung, thực tế là chuẩn. Biến `INTELIJET_HOME` cho phép máy khác cài ở đường dẫn khác; tạo file `.env.example` ghi mẫu.)

## Việc 2 — Viết lại `run_docker.sh` thành launcher mỏng

Giữ phần đầu (docker group check, `xhost +local:docker`, `xrandr` xoay màn hình), thay 3 nhánh if bằng:
```
docker compose up -d   # (chạy tại thư mục repo; tự tạo/khởi động container)
```
- Không dùng `sudo docker` nếu user đã trong group docker (hiện script trộn cả hai — chuẩn hoá về không-sudo, chỉ fallback sudo nếu cần).
- Thoát script khi app trong container tự kết thúc? Không cần chờ — `-d` rồi thoát; vòng đời container gắn với `run_intelijet.sh` (khi lệnh command kết thúc, container dừng — xác nhận điều này đúng với nội dung `run_intelijet.sh` thực tế; nếu `run_intelijet.sh` không phải tiến trình tiền cảnh, điều chỉnh cho đúng).
- Kiểm tra `docker compose version` đầu script; nếu máy chỉ có `docker-compose` bản cũ, dùng lệnh đó thay thế (biến `COMPOSE_CMD`).

## Việc 3 — `intelijet.desktop`

- Sửa Exec/TryExec dùng đường dẫn qua `$HOME` không được hỗ trợ trong .desktop → giải pháp: cài đặt (script setup) sinh file .desktop với đường dẫn thật, HOẶC đơn giản giữ đường dẫn tuyệt đối nhưng ghi rõ trong tài liệu cài đặt. Chọn phương án ít phức tạp nhất; ghi hướng dẫn cài vào `docs/INSTALL.md` (file mới, ngắn: các bước từ máy trắng → chạy được bằng icon: cài docker + compose, clone repo, pull/build image, copy .desktop).

## Việc 4 — Nút Exit tắt toàn bộ

Trình tự đích khi bấm Exit (sau xác nhận):
1. App dừng worker thread (P5, nếu đã làm: `worker.wait(5000)`).
2. `rosnode kill -a` như hiện tại (dừng node ngoài app).
3. App thoát bình thường (`close()`).
4. `run_intelijet.sh` kết thúc → container tự dừng (vì command của compose là script này). **Không cần app gọi `docker stop` từ trong container** — kiểm tra `run_intelijet.sh`: nếu nó là script blocking chạy roslaunch/app thì cơ chế này tự đúng; nếu nó daemonize thứ gì đó, sửa để tiến trình chính chờ app (ví dụ `wait`).
5. Trường hợp roscore/tiến trình con không chết theo: thêm `trap` trong `run_intelijet.sh` kill process group khi thoát.

Kết quả cần đạt: sau khi bấm Exit, `docker ps` không còn container `intelijet` (hoặc ở trạng thái Exited), không còn tiến trình ROS nào chạy; bấm icon desktop lần nữa → hệ thống lên lại sạch.

## Kiểm chứng

1. `docker compose config` hợp lệ (parse được).
2. Trên máy có Docker: `run_docker.sh` → app lên; bấm Exit → container dừng; bấm icon → lên lại. Lặp 3 lần không lỗi.
3. Xoá container + image tag rồi chạy lại từ đầu bằng compose (mô phỏng máy mới) — nếu điều kiện cho phép.
4. Không có môi trường Docker trong phiên làm việc → soát tay kỹ + ghi rõ mục nào chưa kiểm chứng thật.

## Tiêu chí nghiệm thu

- [ ] `docker-compose.yml` tương đương 100% tham số `docker run` cũ; `.env.example` cho đường dẫn cài.
- [ ] `run_docker.sh` mỏng, không còn 3 nhánh if thủ công.
- [ ] Exit → không còn container/tiến trình mồ côi; mở lại sạch.
- [ ] `docs/INSTALL.md` hướng dẫn cài từ máy trắng.
- [ ] Commit `[P8] ...`; cập nhật `00_INDEX.md`.

## Báo cáo hoàn thành

Việc 1-3 xong:
- `docker-compose.yml`: hợp nhất thành bản chính thức duy nhất (trước đó có 1 bản "quick dev" tách riêng, giờ dùng chung cho cả dev lẫn icon). `docker compose config` parse hợp lệ (đã chạy thật trong phiên này).
- `run_docker.sh`: viết lại thành launcher mỏng - group check, `xhost`, `xrandr` rotate (giữ), phát hiện `docker compose` vs `docker-compose` cũ, rồi `down` + `up -d`.
- `install.sh` (mới, không có trong plan gốc - phát sinh theo yêu cầu người dùng "cài đặt đơn giản"): tự cài Docker (script `get.docker.com`) + compose plugin nếu thiếu, thêm user vào group `docker`, `docker compose build` một lần, sinh `intelijet.desktop` với đường dẫn tuyệt đối đúng máy đang cài (vào `~/.local/share/applications` + `~/Desktop` nếu có) - giải quyết đúng vấn đề Việc 3 nêu (không dùng `$HOME` được trong .desktop).
- `docs/INSTALL.md` (mới): hướng dẫn cài từ máy trắng - clone → `./install.sh`.
- Xoá `intelijet.desktop` tĩnh ở gốc repo (hardcode `/home/nuc/...`) - đã bị thay thế hoàn toàn bởi bản `install.sh` tự sinh.

Việc 4 (nút Exit tắt toàn bộ container từ trong app) **chưa làm** - người dùng chỉ yêu cầu phần cài đặt/icon lần này, không đụng `app.py`.

## Ghi chú phát sinh

1. **Lệch khỏi Việc 2 gốc theo yêu cầu người dùng**: plan gốc đề xuất icon dùng `docker compose up -d` (tái sử dụng container đang chạy nếu có). Người dùng chốt: mỗi lần bấm icon phải **restart** (down rồi up lại) để luôn khởi động sạch, không rebuild image (giữ nhanh). Đã implement đúng theo hướng này trong `run_docker.sh`.
2. **Chưa kiểm chứng thật trên máy Linux có Docker** (Kiểm chứng mục 2-3 của phase): phiên làm việc này chạy trên WSL, có Docker Desktop nhưng chưa test full luồng `install.sh` → icon → app lên trên máy Linux thật (không có sẵn máy đó trong phiên). Chỉ kiểm chứng được: `docker compose config` hợp lệ, cú pháp `run_docker.sh`/`install.sh` đúng (`bash -n`). Cần người dùng tự chạy `install.sh` trên máy Linux thật và xác nhận icon hoạt động.
3. Image tag đổi từ `jacondv/jacon-pps-noetic:v2.1` (cũ, dùng trong `run_docker.sh` gốc) / `:compose` (bản dev tạm) → `:latest` (thống nhất 1 tag duy nhất cho bản build tại chỗ qua `install.sh`). Nếu sau này cần pull image dựng sẵn từ registry thay vì build tại chỗ, sẽ cần đặt lại tag rõ ràng hơn (vd theo version).
