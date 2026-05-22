# Camera-LiDAR Extrinsic Calibration - AprilTag Method

## Tổng Quan
Phương pháp này sử dụng **AprilTag marker** để calibrate vị trí tương đối giữa camera và LiDAR.

### Ưu điểm
- ✅ Nhanh (chỉ cần 1 frame)
- ✅ Chính xác cao (có thể đạt 1-2cm)
- ✅ Không phụ thuộc vào intrinsics camera
- ✅ Dễ setup lại (di động)

### Nhược điểm
- ❌ Cần AprilTag marker
- ❌ Marker phải visible từ cả camera và LiDAR

---

## Chuẩn Bị

### 1. In AprilTag Marker
```bash
# Download AprilTag (DICT_6X6_250)
# https://github.com/AprilRobotics/apriltag-imgs

# Hoặc tạo từ code:
# python3 -m pip install pupil-apriltags
# python3 -c "
# import pupil_apriltags as apriltag
# det = apriltag.Detector()
# ... (generate và in)
# "

# Kích thước recommend: 15cm x 15cm (in trên giấy A4)
```

### 2. Chuẩn Bị Thiết Bị
- Đặt AprilTag ngang bằng, có thể nhìn thấy từ camera
- Chắc chắn LiDAR có thể "nhìn" được marker (không bị vật cản)
- Đảm bảo camera & LiDAR chạy bình thường

---

## Bước 1: Capture Dữ Liệu

### Từ Camera
```bash
# Mở terminal 1
ros2 launch my_azu full_mapping.launch.py

# Mở terminal 2 - Capture image
ros2 run image_saver image_saver_node image:=/camera/color/image_raw

# Hoặc dùng ros2 bag
ros2 bag record /camera/color/image_raw /scan --duration 5
```

### Từ LiDAR
```bash
# LiDAR data sẽ được publish trong terminal trên
# Nếu cần save riêng:
ros2 run tf2_tools view_frames  # Để verify TF tree có scan không
```

---

## Bước 2: Chạy Calibration Script

### Setup Environment
```bash
cd ~/azu_ws
source install/setup.bash

# Install dependencies nếu chưa có
pip install scipy opencv-python numpy
```

### Test Mode (với synthetic data)
```bash
python3 src/my_azu/scripts/calibrate_camera_lidar.py
```

**Output:**
```
CAMERA-LIDAR EXTRINSIC CALIBRATION RESULT
============================================================

Translation (camera → lidar) [m]:
  X: 0.150000
  Y: 0.000000
  Z: -0.030000

Rotation Matrix:
  [[1. 0. 0.]
   [0. 1. 0.]
   [0. 0. 1.]]

Euler Angles (Roll, Pitch, Yaw) [degrees]:
  Roll (X):  0.000000
  Pitch (Y): 0.000000
  Yaw (Z):   0.000000

FOR URDF XACRO UPDATE:
  <origin xyz="0.1500 0.0000 -0.0300"
          rpy="0.000000 0.000000 0.000000" />
```

### Real Data (cần sửa script)

**Sửa file `calibrate_camera_lidar.py`:**

Thay phần `main()` từ line 250:
```python
def main():
    calib = CameraLidarCalibration()
    
    # Load real image
    import cv2
    image = cv2.imread('camera_apriltag.jpg')  # Thay tên file thực tế
    
    # Load real point cloud (nếu có)
    # Ví dụ: từ ros2 bag
    # point_cloud = load_point_cloud_from_bag('scan_apriltag.pcd')
    
    # Detect từ camera
    camera_pose = calib.detect_apriltag_camera(image, tag_size=0.15)
    if camera_pose is None:
        print("ERROR: Could not detect AprilTag in camera image")
        return
    
    print("[Camera] Detected AprilTag:")
    print(f"  Position: {camera_pose['tvec'].T}")
    print(f"  ID: {camera_pose['tag_id']}")
    
    # Detect từ LiDAR (simplified)
    # Cần implement full detection hoặc manual input
    lidar_pose = {
        'centroid': np.array([0.15, 0.0, 0.5]),  # Nhập thủ công từ visualization
        'normal': np.array([0.0, 0.0, 1.0])
    }
    
    # Compute transformation
    rotation, translation = calib.compute_transformation(camera_pose, lidar_pose)
    
    # Print results
    result = calib.print_calibration_result(rotation, translation)
```

Chạy:
```bash
python3 src/my_azu/scripts/calibrate_camera_lidar.py
```

---

## Bước 3: Xác Minh Kết Quả

### Visualization trong RViz
```bash
# Terminal 1
ros2 launch my_azu full_mapping.launch.py

# Terminal 2 - Xem TF tree
ros2 run tf2_tools view_frames.py
```

### Check Alignment
```bash
# Xem nếu camera depth points align với LiDAR points
ros2 run rviz2 rviz2 -d ~/azu_ws/src/my_azu/configs/mapping.rviz

# Add displays:
# - PointCloud2 (/camera/depth/image_rect_raw converted to cloud)
# - PointCloud2 (/scan)
# - RobotModel
```

---

## Bước 4: Update URDF

Từ output calibration, lấy giá trị `FOR URDF XACRO UPDATE`.

**Edit file:** [azu.urdf.xacro](../../urdf/azu.urdf.xacro)

Tìm `Head_to_camera` joint:
```xml
<!-- Camera joint (fixed trên Head, offset: x=+0.04m, z=+0.02m) -->
<joint name="Head_to_camera" type="fixed">
  <origin xyz="0.04 0.0 0.02" rpy="0 0 0" />  <!-- ← Update xyz rpy từ đây -->
  <parent link="Head" />
  <child link="camera_link" />
</joint>
```

**Ví dụ cập nhật:**
```xml
<joint name="Head_to_camera" type="fixed">
  <origin xyz="0.1500 0.0000 -0.0300" rpy="0.000000 0.000000 0.000000" />
  <parent link="Head" />
  <child link="camera_link" />
</joint>
```

### Rebuild & Test
```bash
colcon build --packages-select my_azu
source install/setup.bash
ros2 launch my_azu full_mapping.launch.py
```

---

## Troubleshooting

### AprilTag không detect được trong camera
- **Kiểm tra:** Marker có rõ ràng không? Ánh sáng có đủ không?
- **Fix:** In marker lớn hơn, tăng lighting, chụp ảnh rõ hơn

### Script không tìm thấy point cloud
- **Kiểm tra:** `/scan` topic có publish không?
  ```bash
  ros2 topic hz /scan
  ```
- **Fix:** Đảm bảo LiDAR driver chạy bình thường

### Kết quả calibration không hợp lý
- **Rerun:** Nhiều lần với marker ở vị trí khác nhau
- **Verify:** Xem hình ảnh RViz, nếu misalign → recalibrate

---

## Tham Khảo

- AprilTag docs: https://april.eecs.umich.edu/software/apriltag/
- OpenCV solvePnP: https://docs.opencv.org/master/d9/d0c/group__calib3d.html
- ROS2 TF2: https://docs.ros.org/en/humble/Concepts/Advanced/Tf2/tf2.html

