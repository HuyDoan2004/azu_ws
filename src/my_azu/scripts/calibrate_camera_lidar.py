#!/usr/bin/env python3
"""
Camera-LiDAR Extrinsic Calibration Script
Detects AprilTag in both camera and LiDAR to compute transformation
"""

import numpy as np
import cv2
from cv2 import aruco
import sys
from typing import Tuple, Optional
from scipy.spatial.transform import Rotation as R

class CameraLidarCalibration:
    def __init__(self):
        # AprilTag detection setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Camera intrinsics (D435i typical values)
        self.camera_matrix = np.array([
            [614.7, 0, 319.5],
            [0, 614.7, 239.5],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        
        self.calibration_data = []
    
    def detect_apriltag_camera(self, image: np.ndarray, tag_size: float = 0.1) -> Optional[dict]:
        """
        Detect AprilTag in camera image
        tag_size: physical size of AprilTag in meters
        Returns: dict with pose or None
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is None or len(ids) == 0:
            print("[Camera] No AprilTag detected")
            return None
        
        # Get first detected tag
        tag_id = ids[0][0]
        corner = corners[0]
        
        # Estimate pose using solvePnP
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            [corner], tag_size, self.camera_matrix, self.dist_coeffs
        )
        
        rvec = rvec[0][0]
        tvec = tvec[0][0]
        
        # Convert to rotation matrix
        rot_mat, _ = cv2.Rodrigues(rvec)
        
        result = {
            'tag_id': tag_id,
            'tvec': tvec,  # Position [x, y, z]
            'rvec': rvec,  # Rotation vector
            'rot_mat': rot_mat,
            'corners': corner
        }
        
        return result
    
    def detect_apriltag_lidar(self, point_cloud: np.ndarray, 
                             tag_size: float = 0.1) -> Optional[dict]:
        """
        Detect AprilTag corners in LiDAR point cloud
        Assumes marker is flat (e.g., on ground or wall)
        point_cloud: Nx3 array of 3D points
        """
        if point_cloud.shape[0] == 0:
            print("[LiDAR] Empty point cloud")
            return None
        
        # Find 4 corners of tag (extreme points)
        # This is simplified - for real calibration, you'd use reflective markers
        
        # Find dominant plane (assuming marker is on a plane)
        # Use RANSAC or PCA
        try:
            # Simple approach: find 4 extreme corners
            min_x_idx = np.argmin(point_cloud[:, 0])
            max_x_idx = np.argmax(point_cloud[:, 0])
            min_y_idx = np.argmin(point_cloud[:, 1])
            max_y_idx = np.argmax(point_cloud[:, 1])
            
            corners_3d = np.array([
                point_cloud[min_x_idx],
                point_cloud[max_x_idx],
                point_cloud[min_y_idx],
                point_cloud[max_y_idx]
            ])
            
            # Fit plane to corners
            centroid = corners_3d.mean(axis=0)
            centered = corners_3d - centroid
            
            # SVD to find normal
            U, S, Vt = np.linalg.svd(centered)
            normal = Vt[-1]  # Smallest singular vector
            
            result = {
                'corners_3d': corners_3d,
                'centroid': centroid,
                'normal': normal,
                'num_points': point_cloud.shape[0]
            }
            
            return result
        except Exception as e:
            print(f"[LiDAR] Error detecting marker: {e}")
            return None
    
    def compute_transformation(self, camera_pose: dict, 
                              lidar_pose: dict) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute extrinsic transformation from camera to LiDAR
        Returns: (rotation_matrix, translation_vector)
        """
        # Camera to tag transformation (from camera frame)
        T_cam_to_tag = np.eye(4)
        T_cam_to_tag[:3, :3] = camera_pose['rot_mat']
        T_cam_to_tag[:3, 3] = camera_pose['tvec']
        
        # LiDAR to tag transformation
        # Use plane normal and centroid
        lidar_centroid = lidar_pose['centroid']
        normal = lidar_pose['normal']
        
        # Create rotation matrix that aligns normal to Z
        z_axis = np.array([0, 0, 1])
        if np.abs(np.dot(normal, z_axis)) < 0.99:
            rotation_axis = np.cross(normal, z_axis)
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            angle = np.arccos(np.dot(normal, z_axis))
            rot = R.from_rotvec(angle * rotation_axis).as_matrix()
        else:
            rot = np.eye(3)
        
        T_lidar_to_tag = np.eye(4)
        T_lidar_to_tag[:3, :3] = rot
        T_lidar_to_tag[:3, 3] = lidar_centroid
        
        # Transformation from camera to LiDAR
        # T_cam_to_lidar = T_cam_to_tag * T_tag_to_lidar
        T_tag_to_lidar = np.linalg.inv(T_lidar_to_tag)
        T_cam_to_lidar = T_cam_to_tag @ T_tag_to_lidar
        
        rotation = T_cam_to_lidar[:3, :3]
        translation = T_cam_to_lidar[:3, 3]
        
        return rotation, translation
    
    def print_calibration_result(self, rotation: np.ndarray, 
                                translation: np.ndarray):
        """Print calibration results in multiple formats"""
        
        print("\n" + "="*60)
        print("CAMERA-LIDAR EXTRINSIC CALIBRATION RESULT")
        print("="*60)
        
        # Translation
        print(f"\nTranslation (camera → lidar) [m]:")
        print(f"  X: {translation[0]:.6f}")
        print(f"  Y: {translation[1]:.6f}")
        print(f"  Z: {translation[2]:.6f}")
        
        # Rotation matrix
        print(f"\nRotation Matrix (camera → lidar):")
        print(rotation)
        
        # Euler angles (RPY)
        rot_obj = R.from_matrix(rotation)
        euler = rot_obj.as_euler('xyz', degrees=True)
        print(f"\nEuler Angles (Roll, Pitch, Yaw) [degrees]:")
        print(f"  Roll (X):  {euler[0]:.6f}")
        print(f"  Pitch (Y): {euler[1]:.6f}")
        print(f"  Yaw (Z):   {euler[2]:.6f}")
        
        # Rotation vector
        rotvec = rot_obj.as_rotvec()
        print(f"\nRotation Vector:")
        print(f"  [{rotvec[0]:.6f}, {rotvec[1]:.6f}, {rotvec[2]:.6f}]")
        
        # Quaternion
        quat = rot_obj.as_quat()  # [x, y, z, w]
        print(f"\nQuaternion (x, y, z, w):")
        print(f"  [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
        
        # For URDF xacro
        print(f"\n" + "-"*60)
        print("FOR URDF XACRO UPDATE:")
        print("-"*60)
        print(f"\nHead_to_camera joint origin (xyz rpy):")
        print(f'  <origin xyz="{translation[0]:.4f} {translation[1]:.4f} {translation[2]:.4f}"')
        print(f'           rpy="{euler[0]*np.pi/180:.6f} {euler[1]*np.pi/180:.6f} {euler[2]*np.pi/180:.6f}" />')
        
        print(f"\nOr in degrees for clarity:")
        print(f'  <origin xyz="{translation[0]:.4f} {translation[1]:.4f} {translation[2]:.4f}"')
        print(f'           rpy="{euler[0]:.4f}° {euler[1]:.4f}° {euler[2]:.4f}°" />')
        
        print("="*60 + "\n")
        
        return {
            'translation': translation,
            'rotation_matrix': rotation,
            'euler_angles': euler,
            'quaternion': quat,
            'rotation_vector': rotvec
        }


def main():
    """
    Example usage: calibrate camera-lidar from image and point cloud files
    """
    calib = CameraLidarCalibration()
    
    # Load sample image (placeholder - replace with actual image)
    print("\n[INFO] Camera-LiDAR Extrinsic Calibration Tool")
    print("-" * 60)
    print("Usage:")
    print("  1. Place AprilTag (DICT_6X6_250, ID) in front of both sensors")
    print("  2. Capture camera image: ros2 topic hz /camera/color/image_raw")
    print("  3. Capture LiDAR scan: ros2 topic hz /scan")
    print("  4. Run this script with image and point cloud data")
    print("-" * 60)
    
    # Example: Create synthetic data for testing
    print("\n[TEST] Running with synthetic data...")
    
    # Create test image with AprilTag (would be from camera in real case)
    test_image = np.ones((480, 640, 3), dtype=np.uint8) * 200
    
    # Create test point cloud with marker points
    test_points = np.array([
        [0.1, 0.1, 0.5],   # Corner 1
        [0.2, 0.1, 0.5],   # Corner 2
        [0.1, 0.2, 0.5],   # Corner 3
        [0.2, 0.2, 0.5],   # Corner 4
        [0.15, 0.15, 0.45], # Plane points
        [0.15, 0.15, 0.55],
    ])
    
    # Simulate detection
    camera_pose = {
        'tag_id': 0,
        'tvec': np.array([0.0, 0.0, 0.3]),
        'rvec': np.array([0.0, 0.0, 0.0]),
        'rot_mat': np.eye(3)
    }
    
    lidar_pose = {
        'centroid': np.array([0.15, 0.15, 0.5]),
        'normal': np.array([0.0, 0.0, 1.0]),
        'num_points': 6
    }
    
    print("\n[Camera] Detected AprilTag:")
    print(f"  Position: {camera_pose['tvec']}")
    print(f"  Rotation: {camera_pose['rvec']}")
    
    print("\n[LiDAR] Detected Marker:")
    print(f"  Centroid: {lidar_pose['centroid']}")
    print(f"  Normal: {lidar_pose['normal']}")
    
    # Compute transformation
    rotation, translation = calib.compute_transformation(camera_pose, lidar_pose)
    
    # Print results
    result = calib.print_calibration_result(rotation, translation)
    
    print("[INFO] Update these values in azu.urdf.xacro:")
    print("  - Modify Head_to_camera joint origin")
    print("  - Modify Head_to_lidar joint origin (if needed)")
    print("\nAfter update, rebuild and test:")
    print("  $ colcon build --packages-select my_azu")
    print("  $ ros2 launch my_azu full_mapping.launch.py")


if __name__ == '__main__':
    main()
