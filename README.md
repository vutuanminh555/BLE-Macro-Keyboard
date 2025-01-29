# ESP32-BLE-Macro-Keyboard
Một số lưu ý:
- Ngoài các file có chữ hid (hidd) trong tên, các file còn lại đều là do nhóm tự viết và thiết kế.
- Tín hiệu đọc ADC từ biến trở do khối ULP kiểm soát (quét giá trị mỗi 20ms), khối ULP được lập trình bằng ngôn ngữ Assembly và được nhúng vào file main.c khi compile bằng cách sửa đổi file cmake. Từ đó hàm main.c có thể lấy dữ liệu và xử lý các biến của khối ULP (có tiền tố ulp_*).
- Driver LCD nhóm tự viết và thiết kế nhưng tuy nhiên có tham khảo driver LCD của thư viện Arduino, chủ yếu cho việc tạo delay để đồng bộ các tín hiệu giữa esp32 và IC điều khiển module khối LCD là Hitachi HD44780.

- Sơ đồ nguyên lý:

  ![image](https://github.com/user-attachments/assets/046993eb-75bb-4525-a03e-e6be01c4c5e6)

- Mô hình 3D:
  
  ![image](https://github.com/user-attachments/assets/1fb5c4f0-3204-40b0-a6d3-0f04c846a506)

- Mạch in thủ công:
  
  ![image](https://github.com/user-attachments/assets/2eb297e3-6549-4b42-894f-4a38871b1934)

  ![image](https://github.com/user-attachments/assets/88d0f1a8-e201-4232-b32e-522b10d3f6ae)

  ![image](https://github.com/user-attachments/assets/c0a8db9b-9886-4d1a-a267-e39a6ad6a0fd)

- Sản phẩm hoàn thiện:

  ![image](https://github.com/user-attachments/assets/e44f484f-8766-4e1c-a802-8371fc110b46)

  ![image](https://github.com/user-attachments/assets/bf0b8fc9-e7b2-4678-9a1d-d063ee295808)

- Video Demo sản phẩm: https://husteduvn-my.sharepoint.com/:v:/g/personal/minh_vt214015_sis_hust_edu_vn/ESyf7wRNJvRGnymgwR_vICYBxZshixKAa6B5-eDRfuJ60w?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=crdaEZ



