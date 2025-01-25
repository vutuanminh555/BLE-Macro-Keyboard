# BLE-Macro-Keyboard
Một số lưu ý:
- Ngoài các file có chữ hid (hidd) trong tên, các file còn lại đều là do nhóm tự viết và thiết kế.
- Tín hiệu đọc ADC từ biến trở do khối ULP kiểm soát (quét giá trị mỗi 20ms), khối ULP được lập trình bằng ngôn ngữ Assembly và được nhúng vào file main.c khi compile bằng cách sửa đổi file cmake. Từ đó hàm main.c có thể lấy dữ liệu và xử lý các biến của khối ULP (có tiền tố ulp_*).
- Driver LCD nhóm tự viết và thiết kế nhưng tuy nhiên có tham khảo driver LCD của thư viện Arduino, chủ yếu cho việc tạo delay để đồng bộ các tín hiệu giữa esp32 và IC điều khiển module khối LCD là Hitachi HD44780.
