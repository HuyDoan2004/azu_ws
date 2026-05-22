# Camera-LiDAR Extrinsic Calibration - Checkerboard Method

## Tổng Quan
Phương pháp này sử dụng **checkerboard pattern** để calibrate vị trí tương đối giữa camera và LiDAR.

### Ưu điểm
- ✅ Không cần đặc biệt marker, có thể tự in
- ✅ Robust với noise (dùng RANSAC + K-means)
- ✅ Có thể refine calibration qua nhiều frames
- ✅ Dễ verify (visual trực quan)

### Nhược điểm
- ❌ Cần plane phẳng (wall, floor, checkerboard board)
- ❌ Chậm hơn AprilTag (cần multiple detections)
- ❌ Yêu cầu LiDAR nhìn rõ checkerboard (reflective surface)

---

## Chuẩn Bị

### 1. In Checkerboard

**Cách 1: In từ OpenCV**
```bash
python3 << 'EOF'
import cv2
import numpy as np

# Tạo checkerboard 9x6, 50mm squares
rows, cols, size = 6, 9, 50  # size in pixels (adjust for printer)

checkerboard = np.zeros((rows * size, cols * size), dtype=np.uint8)
for i in range(rows):
    for j in range(cols):
        if (i + j) % 2 == 0:
            checkerboard[i*size:(i+1)*size, j*size:(j+1)*size] = 255

cv2.imwrite('checkerboard_9x6.png', checkerboard)
print("Saved: checkerboard_9x6.png (print at 300 DPI)")
EOF
```

**Cách 2: Download mẫu**
```bash
# https://github.com/opencv/opencv/blob/master/samples/data/checkerboard.png
```

**In:**
- Kích thước: A3 hoặc A2 (càng to càng tốt)
- DPI: 300 (high quality)
- Giấy: trắng, không lóng lánh
- **Checkerboard size:** 9 columns × 6 rows (54 corners)
- **Square size:** 50mm hoặc theo công thức: `physical_distance = square_size_pixels * 25.4 / 300_dpi`

---

## Bước 1: Capture Dữ Liệu

### Setup Robot
```bash
# Terminal 1: Khởi động hệ thống
ros2 launch my_azu full_mapping.launch.py
```

### Đặt Checkerboard
- Đặt board vuông góc với camera (~50cm away)
- LiDAR có thể "nhìn" được board (không bị vật cản)
- Board phải nằm trong FOV cả camera và LiDAR

### Capture Image
```bash
# Terminal 2: Lưu ảnh
cd ~/azu_ws/src/my_azu/scripts

# Cách 1: Dùng image_saver
ros2 run image_saver image_saver_node image:=/camera/color/image_raw
# Ảnh sẽ lưu trong thư mục hiện tại

# Cách 2: Dùng ROS2 bag
ros2 bag record /camera/color/image_raw /scan --duration 10
```

### Capture Point Cloud
```bash
# Terminal 3: Convert LiDAR scan to point cloud
# (Hoặc extract từ ros2 bag)

python3 << 'EOF'
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection

# Subscribe đến /scan, project to 3D, lưu thành numpy file
# (chi tiết implementation bên dưới)
EOF
```

---

## Bước 2: Xử Lý Dữ Liệu

### Chuẩn Bị Files

```
~/azu_ws/src/my_azu/scripts/
├── camera_checkerboard.jpg      # Ảnh checkerboard từ camera
├── lidar_checkerboard.npy       # Point cloud (Nx3 numpy array)
└── calibrate_checkerboard.py    # Script calibration
```

### Convert LiDAR Scan to Point Cloud

**Script helper:**
```python
# save_lidar_pointcloud.py
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudRecorder:
    def __init__(self):
        self.projector = LaserProjection()
        self.points = []
        
    def scan_callback(self, msg: LaserScan):
        # Convert LaserScan to PointCloud2
        cloud = self.projector.projectLaser(msg)
        # Convert to numpy
        points = pc2.read_points(cloud, field_names=("x", "y", "z"))
        self.points = np.array(list(points))
        print(f"Recorded {len(self.points)} points")

if __name__ == '__main__':
    # Save point cloud
    np.save('lidar_checkerboard.npy', recorder.points)
```

---

## Bước 3: Chạy Calibration Script

### Test Mode
```bash
cd ~/azu_ws/src/my_azu/scripts
python3 calibrate_checkerboard.py
```

**Output:**
```
CHECKERBOARD-BASED CAMERA-LIDAR EXTRINSIC CALIBRATION
======================================================================

[CAMERA DETECTION]
  Corners detected: 54
  Pose (camera frame):
    Position: [  0.   0. 300.]
    Rotation: [ 0.  0.  0.]

[LIDAR DETECTION]
  Corners detected: 54
  Inliers: 456
  Plane normal: [0. 0. 1.]
  Plane center: [0.15 0.0 0.3]

[OPTIMIZATION RESULT]
  Success: True
  Iterations: 147
  Final error: 0.000234

[TRANSFORMATION (Camera → LiDAR)]
  Translation [m]:
    X: 0.150000
    Y: -0.000123
    Z: -0.029876

  Rotation Matrix:
  [[1.00000000e+00 -2.15438000e-06 -3.20000000e-04]
   [2.15438000e-06  1.00000000e+00 -1.15000000e-04]
   [3.20000000e-04  1.15000000e-04  1.00000000e+00]]

  Euler Angles (Roll, Pitch, Yaw) [degrees]:
    Roll (X):  -0.006588
    Pitch (Y): 0.018374
    Yaw (Z):   -0.000123

  Quaternion (x, y, z, w):
    [-0.000036, 0.000101, -0.000001, 1.000000]

======================================================================
FOR URDF XACRO UPDATE:

Head_to_camera joint origin (xyz rpy in radians):
  <origin xyz="0.150000 -0.000123 -0.029876"
          rpy="-0.000115 0.000320 -0.000002" />
```

### Real Data

**Sửa script `calibrate_checkerboard.py`, hàm `main()`:**

```python
def main():
    print("\n[INFO] Checkerboard-based Camera-LiDAR Calibration")
    print("-" * 70)
    
    # Initialize calibrator
    calib = CheckerboardCalibration(
        checkerboard_size=(9, 6),
        square_size=0.05  # 50mm
    )
    
    # ===== REAL DATA MODE =====
    import cv2
    
    # Load camera image
    print("\n[1] Loading camera image...")
    image = cv2.imread('camera_checkerboard.jpg')
    if image is None:
        print("ERROR: Could not load camera_checkerboard.jpg")
        return
    
    # Detect checkerboard in camera
    print("[2] Detecting checkerboard in camera...")
    camera_data = calib.detect_checkerboard_camera(image)
    if camera_data is None:
        print("ERROR: Could not detect checkerboard in image")
        print("- Check image quality")
        print("- Ensure checkerboard is 9x6 pattern")
        print("- Try different lighting")
        return
    
    print(f"✓ Detected {camera_data['num_corners']} corners")
    
    # Load LiDAR point cloud
    print("\n[3] Loading LiDAR point cloud...")
    lidar_cloud = np.load('lidar_checkerboard.npy')
    print(f"✓ Loaded {len(lidar_cloud)} points")
    
    # Detect checkerboard in LiDAR
    print("[4] Detecting checkerboard in LiDAR...")
    lidar_data = calib.detect_checkerboard_lidar(lidar_cloud)
    if lidar_data is None:
        print("ERROR: Could not detect checkerboard in point cloud")
        print("- Check point cloud quality")
        print("- Ensure checkerboard is visible to LiDAR")
        print("- Check for plane detection issues")
        return
    
    print(f"✓ Detected {lidar_data['num_corners']} corners")
    print(f"✓ Found {lidar_data['inlier_count']} inlier points on plane")
    
    # Optimize transformation
    print("\n[5] Optimizing transformation (may take a minute)...")
    optimization_result = calib.optimize_transformation(
        camera_data, lidar_data, iterations=200
    )
    
    # Print results
    print("\n[6] Calibration Results:")
    result = calib.print_results(camera_data, lidar_data, optimization_result)
    
    print("[✓] Calibration complete!")
    print("\nNext steps:")
    print("  1. Copy xyz and rpy values from above")
    print("  2. Update azu.urdf.xacro Head_to_camera joint")
    print("  3. Rebuild: colcon build --packages-select my_azu")
    print("  4. Test: ros2 launch my_azu full_mapping.launch.py")
```

**Chạy:**
```bash
python3 calibrate_checkerboard.py
```

---

## Bước 4: Verify Kết Quả

### Visualization
```bash
# Terminal: Xem alignment trong RViz
ros2 launch my_azu full_mapping.launch.py
```

Trong RViz Displays, thêm:
- **PointCloud2** `/camera/depth/points` (RGB)
- **PointCloud2** `/scan` (converted)
- **RobotModel**

**Check:** Camera depth points có align với LiDAR scan không?

### Command Line Check
```bash
# Check TF tree
ros2 run tf2_tools view_frames.py

# Check topic data
ros2 topic echo /tf --once | grep camera
```

---

## Bước 5: Update URDF

Từ calibration output, lấy giá trị:

**Edit:** [azu.urdf.xacro](../../urdf/azu.urdf.xacro)

```xml
<!-- Camera joint (fixed trên Head, offset: x=+0.04m, z=+0.02m) -->
<joint name="Head_to_camera" type="fixed">
  <origin xyz="0.04 0.0 0.02" rpy="0 0 0" />  <!-- ← Update từ đây -->
  <parent link="Head" />
  <child link="camera_link" />
</joint>
```

**Ví dụ:**
```xml
<joint name="Head_to_camera" type="fixed">
  <origin xyz="0.150000 -0.000123 -0.029876" rpy="-0.000115 0.000320 -0.000002" />
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

### Checkerboard không detect trong camera
```
[Camera] Checkerboard not found
```

**Fixes:**
- Đảm bảo board nằm trong camera view
- Tăng contrast (tut tối độ sáng)
- Sử dụng board lớn hơn
- Adjust intrinsics camera (nếu sai)
- Chuẩn bị ảnh mới

### LiDAR detect quá ít corners
```
[LiDAR] Too few inliers: 5
```

**Fixes:**
- Board cần reflect laser tốt (trắng, matte surface)
- Đưa board gần hơn LiDAR
- Tăng `z_tolerance` parameter
- Loại bỏ background noise

### Optimization không converge
```
Final error: 1.234567
```

**Fixes:**
- Tăng `iterations` từ 100 → 200 hoặc 500
- Kiểm tra data quality (camera corners & LiDAR corners)
- Thử lại từ vị trí khác
- Multiple runs, average results

---

## Advanced: Multi-Frame Calibration

Để improve accuracy, capture multiple frames:

```python
# Calibrate from multiple images
all_results = []

for frame in range(5):
    image = cv2.imread(f'camera_checkerboard_{frame}.jpg')
    lidar = np.load(f'lidar_checkerboard_{frame}.npy')
    
    camera_data = calib.detect_checkerboard_camera(image)
    lidar_data = calib.detect_checkerboard_lidar(lidar)
    result = calib.optimize_transformation(camera_data, lidar_data)
    
    all_results.append({
        'translation': result['translation'],
        'euler_angles': result['euler_angles']
    })

# Average results
avg_translation = np.mean([r['translation'] for r in all_results], axis=0)
avg_euler = np.mean([r['euler_angles'] for r in all_results], axis=0)

print(f"Average Translation: {avg_translation}")
print(f"Average Euler: {avg_euler * 180 / np.pi}")  # in degrees
```

---

## Tham Khảo

- OpenCV Checkerboard: https://docs.opencv.org/master/d9/d0c/group__calib3d.html
- RANSAC Plane Fitting: https://en.wikipedia.org/wiki/Random_sample_consensus
- LaserProjection ROS2: https://docs.ros.org/en/humble/Concepts/Intermediate/TF2/Tf2-Projection/index.html
- Paper: Zhang & Pless (2004) - Camera Calibration with LiDAR

