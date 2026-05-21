#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time
from typing import Optional
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2

# --- modules (giữ trong package my_robot) ---
from my_robot.perception.yolo_tracker import YoloTracker
from my_robot.perception.distance_estimator import DistanceEstimator
from my_robot.perception.visualizer import Visualizer

try:
    import torch
except Exception:
    torch = None


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')

        # ===== Parameters =====
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('model_weights', 'yolo11n.pt')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('publish_viz', True)
        self.declare_parameter('show_window', False)
        self.declare_parameter('process_stride', 1)
        self.declare_parameter('fps', 15)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        enable_yolo = bool(self.get_parameter('enable_yolo').value)
        self.publish_viz = bool(self.get_parameter('publish_viz').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self._stride = max(1, int(self.get_parameter('process_stride').value))

        # ===== QoS =====
        self.qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ===== Bridge & buffers =====
        self.bridge = CvBridge()
        self.rgb: Optional[np.ndarray] = None
        self.depth: Optional[np.ndarray] = None
        self.caminfo: Optional[CameraInfo] = None
        self.rgb_header: Optional[Header] = None
        self._got_rgb = False

        # ===== Subscriptions =====
        rgb_topic   = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value

        self.create_subscription(Image, rgb_topic, self.on_rgb, self.qos_sensor)
        self.create_subscription(Image, depth_topic, self.on_depth, self.qos_sensor)
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Imu, '/camera/imu', lambda _msg: None, self.qos_sensor)

        # ===== Publishers =====
        self.pub_viz = self.create_publisher(Image, '/camera/yolo/image', 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/camera/yolo/detections', 10)

        # ===== YOLO + TensorRT Auto =====
        model_path = self.get_parameter('model_weights').value
        if model_path.endswith('.engine'):
            self.get_logger().info(f"Detected TensorRT engine: {model_path}")
        else:
            self.get_logger().info(f"Using PyTorch model: {model_path}")

        self.tracker = (YoloTracker(
            model_weights=model_path,
            imgsz=self.imgsz,
            use_gpu=bool(self.get_parameter('use_gpu').value),
            use_fp16=bool(self.get_parameter('use_fp16').value),
        ) if enable_yolo else None)

        if self.tracker is None:
            self.get_logger().warn('enable_yolo=false → sẽ không chạy tracking')

        # ===== Các module phụ =====
        self.dist_est = DistanceEstimator()
        self.viz = Visualizer(show=False)

        # ===== Performance Diagnostics =====   
        os.environ.setdefault("OMP_NUM_THREADS", "2")
        os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")
        self._t_last = time.time()
        self._frames = 0

        if torch is not None:
            cuda_ok = torch.cuda.is_available()
            msg = f"CUDA available={cuda_ok}"
            if cuda_ok:
                try:
                    dev = torch.cuda.current_device()
                    name = torch.cuda.get_device_name(dev)
                    cc = ".".join(map(str, torch.cuda.get_device_capability(dev)))
                    torch.backends.cudnn.benchmark = True
                    msg += f", device={name}, cc={cc}"
                except Exception as e:
                    msg += f" (err={e})"
            self.get_logger().info(f"[YOLO Runtime] {msg}")
            if hasattr(self.tracker, "model"):
                try:
                    device_attr = getattr(self.tracker.model, "device", "unknown")
                    self.get_logger().info(f"[YOLO Model Device] {device_attr}")
                except Exception:
                    pass
        else:
            self.get_logger().warn("PyTorch not available; chỉ TensorRT engine được hỗ trợ.")

        # ===== Timer loop =====
        self._tick = 0
        self.timer = self.create_timer(1.0 / float(self.get_parameter('fps').value), self.loop)

        self.get_logger().info(f'Subscribing: {rgb_topic}, {depth_topic}, {info_topic}')
        self.get_logger().info('Publishing:  /camera/yolo/image, /camera/yolo/detections')

        if self.show_window:
            try:
                cv2.namedWindow('YOLO', cv2.WINDOW_NORMAL)
            except Exception as e:
                self.get_logger().error(f'Cannot create OpenCV window: {e}')
                self.show_window = False

    # ===== Callbacks =====
    def on_rgb(self, msg: Image):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.rgb_header = msg.header
            if not self._got_rgb:
                self._got_rgb = True
                self.get_logger().info(f"Got first RGB frame from {self.get_parameter('rgb_topic').value}")
        except Exception as e:
            self.get_logger().warning(f'cv_bridge rgb error: {e}')

    def on_depth(self, msg: Image):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warning(f'cv_bridge depth error: {e}')

    def on_info(self, msg: CameraInfo):
        self.caminfo = msg

    # ===== Convert YOLO output → vision_msgs =====
    def _to_detection_array(self, header: Header, result, depths_mm=None) -> Detection2DArray:
        arr = Detection2DArray(); arr.header = header
        boxes = getattr(result, 'boxes', None)
        if boxes is None or len(boxes) == 0:
            return arr
        xyxy = boxes.xyxy.cpu().numpy().astype(float)
        conf = boxes.conf.cpu().numpy().astype(float)
        cls  = boxes.cls.cpu().numpy().astype(int)
        ids  = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else [-1] * len(xyxy)
        names = getattr(getattr(self.tracker, 'model', None), 'names', {}) if self.tracker else {}

        for i, (bb, cf, ci, tid) in enumerate(zip(xyxy, conf, cls, ids)):
            x1, y1, x2, y2 = bb
            det = Detection2D(); det.header = header
            cx = (x1 + x2) * 0.5; cy = (y1 + y2) * 0.5
            w  = max(1.0, x2 - x1); h  = max(1.0, y2 - y1)
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = cx; det.bbox.center.position.y = cy
            det.bbox.size_x = w; det.bbox.size_y = h

            hyp = ObjectHypothesisWithPose()
            if isinstance(names, dict) and ci in names:
                hyp.hypothesis.class_id = names[ci]
            else:
                hyp.hypothesis.class_id = str(ci)
            hyp.hypothesis.score = float(cf)

            if depths_mm is not None and i < len(depths_mm) and depths_mm[i] is not None:
                try:
                    hyp.pose.pose.position.z = float(depths_mm[i]) / 1000.0
                except Exception:
                    pass

            det.results.append(hyp); arr.detections.append(det)
        return arr

    # ===== Main loop =====
    def loop(self):
        if self.rgb is None or self.tracker is None:
            return
        self._tick = (self._tick + 1) % self._stride
        if self._tick != 0:
            return

        rgb = self.rgb.copy()
        header = self.rgb_header if self.rgb_header is not None else Header()

        # --- FPS counter ---
        self._frames += 1
        if self._frames % 60 == 0:
            now = time.time()
            fps = 60.0 / max(1e-6, (now - self._t_last))
            self._t_last = now
            self.get_logger().info(f"[YOLO] approx FPS={fps:.1f}")

        # 1) YOLO inference
        try:
            result, dets_list = self.tracker.infer(rgb)
        except Exception as e:
            self.get_logger().warning(f'YOLO track error: {e}')
            return

        # 2) Distance estimate
        depths_mm = [0]*len(dets_list) if self.depth is None else self.dist_est.estimate(self.depth, dets_list)

        # 3) Publish detections
        det_msg = self._to_detection_array(header, result, depths_mm)
        self.pub_det.publish(det_msg)

        # 4) Visualization
        try:
            viz = self.viz.draw(rgb.copy(), dets_list, depths_mm)
        except Exception:
            viz = rgb

        if self.publish_viz:
            try:
                self.pub_viz.publish(self.bridge.cv2_to_imgmsg(viz, encoding='bgr8'))
            except Exception as e:
                self.get_logger().warning(f'viz publish error: {e}')

        if self.show_window:
            cv2.imshow('YOLO', viz)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = RealSenseYoloNode()
    try:
        rclpy.spin(node)
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
