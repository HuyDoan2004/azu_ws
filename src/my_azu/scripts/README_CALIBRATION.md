# Camera-LiDAR Extrinsic Calibration - Quick Start Guide

## Giới Thiệu
Calibration xác định **vị trí tương đối** giữa camera và LiDAR mounted trên Head.

**Đầu ra:** Transformation matrix (xyz, rpy) → Update vào URDF → SLAM chính xác hơn

---

## So Sánh 2 Phương Pháp

| Tiêu Chí | **AprilTag** | **Checkerboard** |
|----------|-------------|-----------------|
| **Setup Time** | 5 phút | 10 phút |
| **Accuracy** | 1-2cm | 2-5cm |
| **Speed** | Cực nhanh (1 frame) | Chậm (nhiều frames) |
| **Cost** | In marker | In pattern |
| **Robustness** | Cao (có camera blur) | Trung bình |
| **Difficulty** | Dễ | Trung bình |
| **Multi-frame** | Không cần | Nên làm |
| **Requires** | AprilTag marker | Plane surface |

---

## Chọn Phương Pháp Nào?

### ✅ Dùng **AprilTag** nếu:
- Muốn **nhanh chóng** (5 phút)
- Cần **accuracy cao** (1-2cm)
- Có **AprilTag marker**
- Chỉ calibrate **1 lần**
- Không có plane surface sẵn

### ✅ Dùng **Checkerboard** nếu:
- Muốn **verify calibration** với AprilTag
- Có **plane surface** (wall, floor)
- Muốn **multi-frame** → higher accuracy
- **AprilTag không available**
- Có **thời gian** (15-20 phút)

---

## Quick Start

### Phương Pháp 1: AprilTag (Nhanh)

```bash
# 1. In AprilTag (DICT_6X6_250, ~15cm)
#    Download: https://github.com/AprilRobotics/apriltag-imgs

# 2. Đặt marker trước camera & LiDAR

# 3. Chạy
cd ~/azu_ws/src/my_azu/scripts
python3 calibrate_camera_lidar.py

# 4. Lấy output → Update URDF
# 5. Build & test
colcon build --packages-select my_azu
ros2 launch my_azu full_mapping.launch.py
```

**📖 Chi tiết:** [README_APRILTAG_CALIBRATION.md](README_APRILTAG_CALIBRATION.md)

---

### Phương Pháp 2: Checkerboard (Verify)

```bash
# 1. In checkerboard 9x6 (50mm squares)
#    Run: python3 << 'EOF'
#    import cv2, numpy as np
#    checkerboard = np.zeros((300, 450), dtype=np.uint8)
#    for i in range(6):
#        for j in range(9):
#            if (i+j)%2==0: checkerboard[i*50:(i+1)*50, j*50:(j+1)*50]=255
#    cv2.imwrite('checkerboard.png', checkerboard)
#    EOF

# 2. Đặt checkerboard trước camera & LiDAR

# 3. Capture data
#    Terminal 1: ros2 launch my_azu full_mapping.launch.py
#    Terminal 2: ros2 run image_saver image_saver_node image:=/camera/color/image_raw
#    Terminal 3: (convert lidar scan to point cloud, save as .npy)

# 4. Chạy calibration
cd ~/azu_ws/src/my_azu/scripts
python3 calibrate_checkerboard.py

# 5. Lấy output → Update URDF
# 6. Build & test
colcon build --packages-select my_azu
ros2 launch my_azu full_mapping.launch.py
```

**📖 Chi tiết:** [README_CHECKERBOARD_CALIBRATION.md](README_CHECKERBOARD_CALIBRATION.md)

---

## Workflow Recommend

### Option A: Nhanh & Simple
```
1. AprilTag calibration (5 phút)
   ↓
2. Update URDF, rebuild
   ↓
3. Test trong RViz
   ↓
Done! ✓
```

### Option B: Chính Xác & Verify
```
1. AprilTag calibration (5 phút)
   ↓
2. Checkerboard calibration (15 phút)
   ↓
3. Compare results
   ↓
4. Average / take best result
   ↓
5. Update URDF, rebuild, test
   ↓
Done! ✓
```

### Option C: Full Calibration (Best)
```
1. AprilTag method 1 (5 phút)
   ↓
2. Checkerboard method multi-frame 3x (20 phút)
   ↓
3. Average all 4 results
   ↓
4. Statistical analysis (standard deviation)
   ↓
5. Take final best value
   ↓
6. Update URDF, rebuild, test
   ↓
Done! ✓ (Very high accuracy)
```

---

## Install Dependencies

```bash
# AprilTag
pip install opencv-python numpy scipy

# Checkerboard
pip install opencv-python numpy scipy scikit-learn

# Optional: Point cloud processing
pip install pyproj laspy open3d
```

---

## Verify Calibration Result

Sau khi update URDF:

```bash
# 1. Build
colcon build --packages-select my_azu

# 2. Source
source install/setup.bash

# 3. Launch
ros2 launch my_azu full_mapping.launch.py

# 4. Xem RViz
# - Add PointCloud2: /camera/depth_points (RGB)
# - Add PointCloud2: /scan (merged)
# - Check alignment: Camera depth có match với LiDAR não?

# 5. Terminal check
ros2 topic hz /scan
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /icp_odometry/odom
```

**Indicators:**
- ✅ Depth points & LiDAR scans overlap
- ✅ Odometry không drift cực tốc
- ✅ Map trong RViz không bị "flipped" hay "distorted"

---

## Files

```
scripts/
├── calibrate_camera_lidar.py          # AprilTag method
├── calibrate_checkerboard.py          # Checkerboard method
├── README_APRILTAG_CALIBRATION.md     # AprilTag guide
├── README_CHECKERBOARD_CALIBRATION.md # Checkerboard guide
└── README_CALIBRATION.md              # This file
```

---

## Troubleshooting

### Common Issues

| Issue | Fix |
|-------|-----|
| AprilTag not detected | Increase marker size, better lighting |
| Checkerboard not detected | Better image quality, print higher DPI |
| LiDAR not detecting | Reflective surface needed, closer distance |
| Results inconsistent | Multi-frame averaging, repeat process |
| SLAM still drifting | Recalibrate, check camera/LiDAR alignment |

---

## Next Steps After Calibration

1. **Verify in SLAM:**
   ```bash
   ros2 launch my_azu full_mapping.launch.py
   # Check map quality, odometry stability
   ```

2. **Fine-tune if needed:**
   - If map drifts: recalibrate with multiple frames
   - If misalignment: adjust offset by ±5mm and retest
   - If still issues: check sensor health

3. **Document results:**
   - Save calibration images/point clouds
   - Save final URDF version
   - Note date, conditions, accuracy metrics

4. **Periodic recalibration:**
   - After hardware changes
   - Every 6-12 months (sensor drift)
   - If map quality degrades

---

## References

- **AprilTag:** https://april.eecs.umich.edu/software/apriltag/
- **OpenCV:** https://docs.opencv.org/master/
- **ROS2 TF2:** https://docs.ros.org/en/humble/Concepts/Advanced/Tf2/tf2.html
- **Paper (Zhang & Pless 2004):** Camera Calibration with Laser Range Finder

---

## Contact & Support

- **Questions?** Check specific README files
- **Errors?** Review troubleshooting sections
- **Feedback?** Update documentation

---

**Happy Calibrating! 🎉**

