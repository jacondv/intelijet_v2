# Quy tắc làm việc cho project intelijet_v2

## Kiến trúc
- Ưu tiên config (YAML) hơn hardcode trong Python cho mọi mapping/policy.
- Đặt logic đúng lớp: `shared/` chứa nghiệp vụ dùng chung, UI chỉ là ống dẫn.
- Grep toàn repo trước khi thêm mới, tránh trùng lặp logic.
- Không trừu tượng hóa sớm - đơn giản trước, tổng quát hóa khi thật cần.

## An toàn vận hành
- Không để lỗi trôi qua trong im lặng - luôn báo động rõ ràng cho operator.
- Thiết bị/tiến trình quan trọng phải tự phục hồi khi gặp sự cố.

## Quy trình
- Không tự ý commit/push; không gắn Claude vào commit/PR.
- Thay đổi lớn: thảo luận và được duyệt phương án trước khi code.
- Mỗi tính năng lớn dùng một nhánh git riêng.

## Chất lượng
- Luôn tự kiểm tra (cú pháp, cấu hình) trước khi báo hoàn thành.
- Nói rõ giới hạn kiểm thử khi chưa thể chạy trên phần cứng thật.
