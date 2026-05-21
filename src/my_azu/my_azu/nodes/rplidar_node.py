#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RPLidar A1 → LaserScan (ROS 2)
- Baud mặc định 115200 (chuẩn A1)
- Binning đều góc (mặc định 1.0°)
- Ngoài tầm: +inf ; Dưới tầm: NaN (theo chuẩn sensor_msgs/LaserScan)
- QoS sensor_data (best-effort, volatile)
- Làm sạch buffer trước khi đọc; tự reconnect/reset khi lỗi (Wrong body size)
- Tùy chọn xuất PointCloud2 (XYZ)
"""

import math, threading, time, traceback, inspect
from typing import Optional, List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from rplidar import RPLidar


class RpliDriver(Node):
    def __init__(self):
        super().__init__('rplidar_node')

        # ===== Params =====
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)              # A1: 115200
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/scan_cloud')
        self.declare_parameter('publish_cloud', False)

        self.declare_parameter('range_min', 0.15)               # A1 min≈0.15 m
        self.declare_parameter('range_max', 6.0)                # A1 hiệu dụng ~6m

        self.declare_parameter('angle_min_deg', 0.0)            # 0..360
        self.declare_parameter('angle_max_deg', 360.0)
        self.declare_parameter('angle_increment_deg', 1.0)      # độ / bin

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baudrate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_cloud = bool(self.get_parameter('publish_cloud').value)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cloud_topic = str(self.get_parameter('cloud_topic').value)

        # Publishers (QoS sensor_data)
        self.pub_scan = self.create_publisher(LaserScan, self.scan_topic, qos_profile_sensor_data)
        self.pub_cloud = self.create_publisher(PointCloud2, self.cloud_topic, qos_profile_sensor_data)

        # Lưới góc
        amin = math.radians(float(self.get_parameter('angle_min_deg').value))
        amax = math.radians(float(self.get_parameter('angle_max_deg').value))
        ainc = math.radians(float(self.get_parameter('angle_increment_deg').value))
        amax = max(amin + ainc, amax)  # đảm bảo hợp lệ

        self.angle_min = float(amin)
        self.angle_max = float(amax)
        self.angle_increment = float(ainc)
        self.num_bins = max(1, int(round((amax - amin) / ainc)))

        self.rmin = float(self.get_parameter('range_min').value)
        self.rmax = float(self.get_parameter('range_max').value)

        # HW handle
        self.lidar: Optional[RPLidar] = None
        self._stop_evt = threading.Event()

        # Pre-allocate arrays (giảm GC)
        self._ranges = np.full(self.num_bins, np.inf, dtype=np.float32)
        self._intens = np.zeros(self.num_bins, dtype=np.float32)

        # Worker
        self._worker = threading.Thread(target=self._run_forever, daemon=True)
        self._worker.start()

    # ===== HW open/close =====
    def _open_lidar(self) -> bool:
        try:
            self.lidar = RPLidar(self.port, baudrate=self.baud, timeout=3)
            # Một số phiên bản cần start_motor() trước
            try:
                self.lidar.start_motor()
            except Exception:
                pass

            # Thông tin & health (nếu có)
            try:
                info = self.lidar.get_info()
                health = self.lidar.get_health()
                self.get_logger().info(f'Info:{info}, Health:{health}')
            except Exception:
                self.get_logger().warn('get_info/get_health failed; continue anyway')

            # Xả rác buffer trước khi vào vòng đọc
            self._flush_input()
            time.sleep(0.1)

            self.get_logger().info(f'RPLidar connected on {self.port} @ {self.baud}')
            return True
        except Exception as e:
            self.get_logger().error(f'Cannot open RPLidar: {e}')
            self._close_lidar()
            return False

    def _close_lidar(self):
        try:
            if self.lidar:
                try:
                    self.lidar.stop()
                except Exception:
                    pass
                try:
                    self.lidar.stop_motor()
                except Exception:
                    pass
                try:
                    self.lidar.disconnect()
                except Exception:
                    pass
        finally:
            self.lidar = None

    def _device_reset(self):
        """Reset “mềm”: stop → reset → start_motor → flush."""
        if not self.lidar:
            return
        try:
            self.lidar.stop()
        except Exception:
            pass
        try:
            self.lidar.stop_motor()
        except Exception:
            pass
        try:
            self.lidar.reset()
        except Exception:
            pass
        time.sleep(0.5)
        try:
            self.lidar.start_motor()
        except Exception:
            pass
        self._flush_input()
        time.sleep(0.1)

    def _flush_input(self):
        try:
            if hasattr(self.lidar, 'clean_input'):
                self.lidar.clean_input()
            elif hasattr(self.lidar, 'clear_input'):
                self.lidar.clear_input()
        except Exception:
            pass

    # ===== Main loop =====
    def _run_forever(self):
        if not self._open_lidar():
            # chờ rồi thử lại
            while not self._stop_evt.is_set():
                time.sleep(1.0)
                if self._open_lidar():
                    break

        last_pub_t = time.time()
        bad_frames = 0

        while not self._stop_evt.is_set():
            if not self.lidar:
                if not self._open_lidar():
                    time.sleep(1.0)
                    continue

            try:
                gen = self._iter_scans_fallback(max_buf_meas=512)
                for raw_scan in gen:
                    if self._stop_evt.is_set():
                        break
                    bad_frames = 0

                    ranges, intens = self._accumulate_scan(raw_scan)

                    # ---- LaserScan ----
                    stamp = self.get_clock().now().to_msg()

                    msg = LaserScan()
                    msg.header = Header(stamp=stamp, frame_id=self.frame_id)
                    msg.angle_min = self.angle_min
                    msg.angle_max = self.angle_max
                    msg.angle_increment = self.angle_increment
                    msg.range_min = self.rmin
                    msg.range_max = self.rmax
                    msg.ranges = ranges.tolist()
                    msg.intensities = intens.tolist()

                    now = time.time()
                    msg.scan_time = float(now - last_pub_t)
                    last_pub_t = now
                    msg.time_increment = msg.scan_time / max(1, len(ranges))

                    self.pub_scan.publish(msg)

                    # ---- PointCloud2 (optional) ----
                    if self.publish_cloud and self.pub_cloud.get_subscription_count() > 0:
                        pts = []
                        a = self.angle_min
                        for r in ranges:
                            if math.isfinite(r) and (self.rmin <= r <= self.rmax):
                                x = r * math.cos(a)
                                y = r * math.sin(a)
                                pts.append((x, y, 0.0))
                            a += self.angle_increment
                        if pts:
                            cloud = point_cloud2.create_cloud_xyz32(msg.header, pts)
                            self.pub_cloud.publish(cloud)

            except Exception as e:
                # Ví dụ: rplidar.RPLidarException: Wrong body size
                self.get_logger().warn(f'scan error: {e}; flushing & retry...')
                bad_frames += 1
                # xả buffer + tạm nghỉ
                self._flush_input()
                time.sleep(0.1)

                if bad_frames >= 3:
                    # reset thiết bị và thử lại
                    self.get_logger().warn('Multiple scan errors → resetting device...')
                    try:
                        self._device_reset()
                        bad_frames = 0
                        self.get_logger().info('Reset done, resume scanning')
                    except Exception:
                        self.get_logger().error('Reset failed, will fully reconnect...')
                        self._close_lidar()
                        time.sleep(1.0)

        # on stop
        self._close_lidar()

    # ===== iter_scans fallback (tùy thư viện) =====
    def _iter_scans_fallback(self, max_buf_meas: int):
        """
        Một số bản python-rplidar hỗ trợ scan_type ('normal' / 'standard'), một số thì không.
        Fallback thứ tự:
         1) scan_type='normal'
         2) scan_type='standard'
         3) không truyền scan_type
        """
        if not self.lidar:
            raise RuntimeError('Lidar not opened')

        sig = inspect.signature(self.lidar.iter_scans)
        kwargs = {'max_buf_meas': max_buf_meas}

        if 'scan_type' in sig.parameters:
            try:
                return self.lidar.iter_scans(scan_type='normal', **kwargs)
            except TypeError:
                pass
            try:
                return self.lidar.iter_scans(scan_type='standard', **kwargs)
            except TypeError:
                pass

        return self.lidar.iter_scans(**kwargs)

    # ===== Binning đều góc, chuẩn hóa NaN/+inf =====
    def _accumulate_scan(self, raw_scan: List[Tuple[int, float, float]]) -> Tuple[np.ndarray, np.ndarray]:
        """
        raw_scan: list of (quality, angle_deg, dist_mm)
        Trả về:
          ranges: float32 với NaN / +inf theo chuẩn
          intens: float32 (quality)
        Quy tắc:
          - r < rmin  → NaN
          - r > rmax  → +inf
          - trong tầm → lấy MIN (ưu tiên vật gần)
        """
        ranges = self._ranges
        intens = self._intens
        ranges.fill(np.inf)
        intens.fill(0.0)

        for quality, ang_deg, dist_mm in raw_scan:
            r = float(dist_mm) / 1000.0
            if r <= 0.0:
                continue

            a = math.radians(float(ang_deg))
            # chuẩn hóa về [angle_min, angle_max)
            while a < self.angle_min:
                a += 2.0 * math.pi
            while a >= self.angle_max:
                a -= 2.0 * math.pi

            idx = int((a - self.angle_min) / self.angle_increment)
            if 0 <= idx < self.num_bins:
                if r < self.rmin:
                    v = np.nan
                elif r > self.rmax:
                    v = np.inf
                else:
                    v = r

                cur = ranges[idx]
                if math.isfinite(v):
                    if not math.isfinite(cur) or (v < cur):
                        ranges[idx] = np.float32(v)
                        intens[idx] = np.float32(quality)
                elif np.isnan(v):
                    if np.isinf(cur):  # chỉ ghi NaN nếu hiện tại là +inf
                        ranges[idx] = np.float32(np.nan)
                        intens[idx] = np.float32(quality)
                else:
                    # v == +inf: chỉ ghi nếu chưa có giá trị hữu hạn hay NaN
                    if not math.isfinite(cur):
                        ranges[idx] = np.float32(np.inf)
                        intens[idx] = np.float32(quality)

        return ranges, intens

    # ===== Shutdown =====
    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self._worker.is_alive():
                self._worker.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RpliDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
