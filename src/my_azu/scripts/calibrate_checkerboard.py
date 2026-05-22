#!/usr/bin/env python3
"""
Camera-LiDAR Extrinsic Calibration using Checkerboard Method
Based on: "Extrinsic Calibration of a Camera and Laser Range Finder" 
by Qilong Zhang and Robert Pless

Method:
1. Place checkerboard in front of both camera and LiDAR
2. Detect checkerboard corners in camera image
3. Detect checkerboard edges in LiDAR point cloud
4. Compute transformation via optimization
"""

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import json
from typing import Tuple, Optional, List, Dict
import warnings
warnings.filterwarnings('ignore')


class CheckerboardCalibration:
    def __init__(self, checkerboard_size: Tuple[int, int] = (9, 6), 
                 square_size: float = 0.05):
        """
        Initialize calibration tool
        checkerboard_size: (width, height) number of corners
        square_size: physical size of each square in meters
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        
        # Camera intrinsics (D435i typical)
        self.camera_matrix = np.array([
            [614.7, 0, 319.5],
            [0, 614.7, 239.5],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        
        # Calibration results storage
        self.camera_corners_2d = None
        self.camera_corners_3d = None
        self.lidar_corners_3d = None
        self.calibration_history = []
    
    def detect_checkerboard_camera(self, image: np.ndarray) -> Optional[Dict]:
        """
        Detect checkerboard in camera image
        Returns: dict with 2D corners and 3D pose
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(
            gray, self.checkerboard_size, None
        )
        
        if not ret:
            print("[Camera] Checkerboard not found")
            return None
        
        # Refine corner positions
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        self.camera_corners_2d = corners.reshape(-1, 2)
        
        # Create 3D coordinates of checkerboard (in checkerboard frame)
        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), 
                        dtype=np.float32)
        objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 
                               0:self.checkerboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        
        # Estimate pose
        success, rvec, tvec = cv2.solvePnP(
            objp, self.camera_corners_2d, self.camera_matrix, self.dist_coeffs
        )
        
        if not success:
            print("[Camera] Failed to estimate pose")
            return None
        
        rot_mat, _ = cv2.Rodrigues(rvec)
        
        # Transform checkerboard 3D points to camera frame
        camera_corners_3d = objp @ rot_mat.T + tvec.T
        
        self.camera_corners_3d = camera_corners_3d
        
        result = {
            'corners_2d': self.camera_corners_2d,
            'corners_3d': camera_corners_3d,
            'rvec': rvec,
            'tvec': tvec,
            'rot_mat': rot_mat,
            'num_corners': len(corners)
        }
        
        return result
    
    def detect_checkerboard_lidar(self, point_cloud: np.ndarray,
                                  z_tolerance: float = 0.02) -> Optional[Dict]:
        """
        Detect checkerboard plane in LiDAR point cloud
        Assumes checkerboard is roughly vertical or at known angle
        
        point_cloud: Nx3 array of 3D points from LiDAR
        z_tolerance: vertical tolerance for plane detection (m)
        """
        if point_cloud.shape[0] < 4:
            print("[LiDAR] Insufficient points in cloud")
            return None
        
        try:
            # Find dominant plane using RANSAC
            inliers, plane_model = self._ransac_plane(
                point_cloud, 
                iterations=100, 
                threshold=0.01
            )
            
            if inliers.sum() < 9:  # Need at least 9 corners
                print(f"[LiDAR] Too few inliers: {inliers.sum()}")
                return None
            
            plane_points = point_cloud[inliers]
            
            # Project points to plane
            plane_normal = plane_model[:3]
            plane_d = plane_model[3]
            
            # Get 2D coordinates on plane
            # Choose two orthogonal directions in the plane
            if abs(plane_normal[2]) > 0.9:
                u_vec = np.array([1.0, 0.0, 0.0])
            else:
                u_vec = np.array([0.0, 0.0, 1.0])
            
            u_vec = u_vec - np.dot(u_vec, plane_normal) * plane_normal
            u_vec = u_vec / np.linalg.norm(u_vec)
            
            v_vec = np.cross(plane_normal, u_vec)
            v_vec = v_vec / np.linalg.norm(v_vec)
            
            # Project plane points to 2D
            plane_2d = np.column_stack([
                np.dot(plane_points - plane_points[0], u_vec),
                np.dot(plane_points - plane_points[0], v_vec)
            ])
            
            # Detect corners using K-means clustering
            corners_2d = self._detect_corners_kmeans(plane_2d)
            
            # Convert back to 3D
            plane_center = plane_points.mean(axis=0)
            corners_3d = np.column_stack([
                plane_center + corners_2d[:, 0:1] * u_vec + corners_2d[:, 1:2] * v_vec
            ])
            
            self.lidar_corners_3d = corners_3d
            
            result = {
                'corners_3d': corners_3d,
                'plane_normal': plane_normal,
                'plane_center': plane_center,
                'num_corners': len(corners_3d),
                'inlier_count': inliers.sum(),
                'u_vec': u_vec,
                'v_vec': v_vec
            }
            
            return result
        
        except Exception as e:
            print(f"[LiDAR] Error detecting checkerboard: {e}")
            return None
    
    def _ransac_plane(self, points: np.ndarray, iterations: int = 100,
                     threshold: float = 0.01) -> Tuple[np.ndarray, np.ndarray]:
        """
        RANSAC plane fitting
        Returns: (inlier_mask, plane_model [a, b, c, d])
        """
        best_inliers = np.zeros(len(points), dtype=bool)
        best_model = None
        
        for _ in range(iterations):
            # Random sample 3 points
            sample_idx = np.random.choice(len(points), 3, replace=False)
            p1, p2, p3 = points[sample_idx]
            
            # Compute plane normal
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            
            if np.linalg.norm(normal) < 1e-6:
                continue
            
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, p1)
            
            # Count inliers
            distances = np.abs(np.dot(points, normal) + d)
            inliers = distances < threshold
            
            if inliers.sum() > best_inliers.sum():
                best_inliers = inliers
                best_model = np.array([normal[0], normal[1], normal[2], d])
        
        return best_inliers, best_model
    
    def _detect_corners_kmeans(self, plane_2d: np.ndarray, 
                              n_clusters: int = 9) -> np.ndarray:
        """
        Detect checkerboard corners using K-means clustering
        """
        from sklearn.cluster import KMeans
        
        kmeans = KMeans(n_clusters=n_clusters, n_init=10, random_state=42)
        kmeans.fit(plane_2d)
        
        return kmeans.cluster_centers_
    
    def optimize_transformation(self, camera_data: Dict, 
                               lidar_data: Dict,
                               iterations: int = 100) -> Dict:
        """
        Optimize transformation using iterative closest point (ICP) style optimization
        """
        # Initial guess: identity transformation
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [tx, ty, tz, rx, ry, rz]
        
        # Define optimization function
        def optimization_error(params):
            tx, ty, tz, rx, ry, rz = params
            translation = np.array([tx, ty, tz])
            rotation = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
            
            # Transform camera corners to LiDAR frame
            T = np.eye(4)
            T[:3, :3] = rotation
            T[:3, 3] = translation
            
            camera_corners_transformed = (
                rotation @ camera_data['corners_3d'].T + translation[:, np.newaxis]
            ).T
            
            # Compute distance to nearest LiDAR corner
            error = 0
            for cam_corner in camera_corners_transformed:
                distances = np.linalg.norm(
                    lidar_data['corners_3d'] - cam_corner, axis=1
                )
                error += np.min(distances)
            
            return error / len(camera_data['corners_3d'])
        
        # Optimize
        result = minimize(
            optimization_error,
            x0,
            method='Nelder-Mead',
            options={'maxiter': iterations, 'xatol': 1e-8, 'fatol': 1e-8}
        )
        
        # Extract final transformation
        tx, ty, tz, rx, ry, rz = result.x
        translation = np.array([tx, ty, tz])
        rotation = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
        
        optimization_result = {
            'translation': translation,
            'rotation': rotation,
            'euler_angles': np.array([rx, ry, rz]),
            'final_error': result.fun,
            'success': result.success,
            'iterations': result.nit
        }
        
        return optimization_result
    
    def print_results(self, camera_data: Dict, lidar_data: Dict, 
                     optimization_result: Dict):
        """Print calibration results"""
        
        print("\n" + "="*70)
        print("CHECKERBOARD-BASED CAMERA-LIDAR EXTRINSIC CALIBRATION")
        print("="*70)
        
        print(f"\n[CAMERA DETECTION]")
        print(f"  Corners detected: {camera_data['num_corners']}")
        print(f"  Pose (camera frame):")
        print(f"    Position: {camera_data['tvec'].T}")
        print(f"    Rotation: {camera_data['rvec'].T}")
        
        print(f"\n[LIDAR DETECTION]")
        print(f"  Corners detected: {lidar_data['num_corners']}")
        print(f"  Inliers: {lidar_data['inlier_count']}")
        print(f"  Plane normal: {lidar_data['plane_normal']}")
        print(f"  Plane center: {lidar_data['plane_center']}")
        
        print(f"\n[OPTIMIZATION RESULT]")
        print(f"  Success: {optimization_result['success']}")
        print(f"  Iterations: {optimization_result['iterations']}")
        print(f"  Final error: {optimization_result['final_error']:.6f}")
        
        rotation = optimization_result['rotation']
        translation = optimization_result['translation']
        euler_deg = optimization_result['euler_angles'] * 180 / np.pi
        
        print(f"\n[TRANSFORMATION (Camera → LiDAR)]")
        print(f"  Translation [m]:")
        print(f"    X: {translation[0]:.6f}")
        print(f"    Y: {translation[1]:.6f}")
        print(f"    Z: {translation[2]:.6f}")
        
        print(f"\n  Rotation Matrix:")
        print(rotation)
        
        print(f"\n  Euler Angles (Roll, Pitch, Yaw) [degrees]:")
        print(f"    Roll (X):  {euler_deg[0]:.6f}")
        print(f"    Pitch (Y): {euler_deg[1]:.6f}")
        print(f"    Yaw (Z):   {euler_deg[2]:.6f}")
        
        # Quaternion
        quat = R.from_matrix(rotation).as_quat()  # [x, y, z, w]
        print(f"\n  Quaternion (x, y, z, w):")
        print(f"    [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
        
        print(f"\n" + "-"*70)
        print("FOR URDF XACRO UPDATE:")
        print("-"*70)
        print(f"\nHead_to_camera joint origin (xyz rpy in radians):")
        print(f'  <origin xyz="{translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f}"')
        print(f'           rpy="{optimization_result["euler_angles"][0]:.6f} {optimization_result["euler_angles"][1]:.6f} {optimization_result["euler_angles"][2]:.6f}" />')
        
        print(f"\nOr in degrees for clarity:")
        print(f'  <origin xyz="{translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f}"')
        print(f'           rpy="{euler_deg[0]:.4f}° {euler_deg[1]:.4f}° {euler_deg[2]:.4f}°" />')
        
        print("="*70 + "\n")
        
        return {
            'translation': translation,
            'rotation': rotation,
            'euler_angles_rad': optimization_result['euler_angles'],
            'euler_angles_deg': euler_deg,
            'quaternion': quat
        }


def main():
    """
    Example usage with synthetic data
    For real calibration, load actual image and point cloud
    """
    print("\n[INFO] Checkerboard-based Camera-LiDAR Calibration")
    print("-" * 70)
    print("Usage:")
    print("  1. Print 9x6 checkerboard (square_size=50mm)")
    print("  2. Place in front of both camera and LiDAR")
    print("  3. Capture camera image: 'ros2 topic hz /camera/color/image_raw'")
    print("  4. Capture LiDAR scan: 'ros2 topic hz /scan'")
    print("  5. Convert to point cloud and run this script")
    print("-" * 70)
    
    # Initialize calibrator
    calib = CheckerboardCalibration(
        checkerboard_size=(9, 6),
        square_size=0.05  # 50mm
    )
    
    # TEST MODE: Create synthetic data
    print("\n[TEST MODE] Running with synthetic checkerboard data...\n")
    
    # Create synthetic camera image with checkerboard
    test_image = np.ones((480, 640, 3), dtype=np.uint8) * 200
    
    # Draw synthetic checkerboard pattern
    for i in range(10):
        for j in range(7):
            x, y = 50 + i*60, 50 + j*60
            if (i + j) % 2 == 0:
                cv2.rectangle(test_image, (x, y), (x+60, y+60), (255, 255, 255), -1)
    
    # Detect checkerboard in camera image
    camera_data = calib.detect_checkerboard_camera(test_image)
    
    if camera_data is None:
        print("[ERROR] Could not detect checkerboard in test image")
        print("For real calibration, use actual camera image")
        return
    
    print(f"[Camera] Detected {camera_data['num_corners']} corners")
    
    # Create synthetic LiDAR point cloud
    # Simulate checkerboard points with some noise
    np.random.seed(42)
    synthetic_lidar_points = []
    
    # Add checkerboard corners with noise
    for i in range(9):
        for j in range(6):
            x = (i - 4) * 0.05
            y = (j - 2.5) * 0.05
            z = 0.3  # Distance from sensor
            noise = np.random.randn(3) * 0.005
            synthetic_lidar_points.append([x, y, z] + noise)
    
    # Add background noise
    background_points = np.random.rand(100, 3) * 2 - 1
    background_points[:, 2] += 0.5
    
    lidar_cloud = np.vstack([synthetic_lidar_points, background_points])
    
    # Detect checkerboard in LiDAR
    lidar_data = calib.detect_checkerboard_lidar(lidar_cloud)
    
    if lidar_data is None:
        print("[ERROR] Could not detect checkerboard in LiDAR cloud")
        return
    
    print(f"[LiDAR] Detected {lidar_data['num_corners']} corners")
    
    # Optimize transformation
    print("\n[INFO] Optimizing transformation...")
    optimization_result = calib.optimize_transformation(camera_data, lidar_data)
    
    # Print results
    result = calib.print_results(camera_data, lidar_data, optimization_result)
    
    print("[INFO] Calibration complete!")
    print("Update the values above in azu.urdf.xacro, then rebuild:")
    print("  $ colcon build --packages-select my_azu")
    print("  $ ros2 launch my_azu full_mapping.launch.py")


if __name__ == '__main__':
    main()
