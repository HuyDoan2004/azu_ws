# Hướng dẫn Test Cảm biến Hall KY003 với Jetson Orin Nano

## Thông tin kết nối

- **Cảm biến:** Hall sensor KY003
- **Pin GPIO:** Pin 7 (BOARD numbering)
- **Mục đích:** Đo encoder bằng cảm biến Hall effect

## Kết nối phần cứng

```
Hall sensor KY003:
  - VCC → +5V (Jetson Orin Nano pin 2 hoặc 4)
  - GND → GND (Jetson Orin Nano pin 6, 9, hoặc 14)
  - DO  → GPIO pin 7 (Jetson Orin Nano pin 7)
```

## Yêu cầu

Cài đặt thư viện Jetson.GPIO:

```bash
sudo pip3 install Jetson.GPIO
```

## Chạy test

```bash
# Chạy file test
sudo python3 test_hall_ky003_encoder.py

# hoặc với quyền sudo nếu gặp lỗi quyền truy cập
sudo python3 test_hall_ky003_encoder.py
```

## Chức năng của file test

1. **Quick Pulse Count**: Đếm xung trong 5 giây
2. **Continuous Monitoring**: Giám sát liên tục tín hiệu Hall (Ctrl+C để dừng)

## Kết quả dự kiến

```
GPIO pin 7 configured as input for Hall sensor
Pulse detected! Total count: 1
Pulse detected! Total count: 2
...
```

## Ghi chú

- Cảm biến Hall KY003 phát hiện từ trường, phù hợp để đo encoder
- Tín hiệu LOW khi phát hiện từ trường (magnetic field)
- Để tính RPM, cần biết số xung trên một vòng quay của encoder
