#!/usr/bin/env python3
"""HSV + aligned-depth cube localization with TF2 and temporal stability."""
from __future__ import annotations

import threading
from collections import deque
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class CubeDetector(Node):
    def __init__(self) -> None:
        super().__init__("cube_detector")
        self._declare_parameters()
        self._load_parameters()
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color_msg: Optional[Image] = None
        self.info: Optional[CameraInfo] = None
        self.history = deque(maxlen=self.window)
        self.last_log_ns = 0
        self.depth_encoding_logged = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.point_pub = self.create_publisher(PointStamped, "/x5a_vision/object_point_camera", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/x5a_vision/object_pose", 10)
        self.debug_pub = self.create_publisher(Image, "/x5a_vision/debug_image", 10)
        self.stable_pub = self.create_publisher(Bool, "/x5a_vision/detection_stable", 10)
        self.create_subscription(Image, self.color_topic, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.get_logger().info(
            f"RGB-D detector color={self.color_topic} depth={self.depth_topic} "
            f"info={self.info_topic} {self.optical_frame}->{self.base_frame}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "color_topic": "/camera/camera/color/image_raw",
            "aligned_depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "optical_frame": "camera_color_optical_frame", "base_frame": "base_link",
            "detection.hsv_lower": [0, 80, 40], "detection.hsv_upper": [12, 255, 255],
            "detection.hsv_lower_2": [170, 80, 40], "detection.hsv_upper_2": [179, 255, 255],
            "detection.min_area_px": 500.0, "detection.max_area_px": 5000.0,
            "detection.open_iterations": 1, "detection.close_iterations": 2,
            "detection.inner_erode_px": 5, "depth.scale_16uc1": 0.001,
            "depth.min_m": 0.20, "depth.max_m": 0.80, "depth.near_percentile": 55.0,
            "object.size_z": 0.03, "table.z": -0.006, "table.top_tolerance": 0.03,
            "workspace.x_min": 0.06, "workspace.x_max": 0.44,
            "workspace.y_min": 0.05, "workspace.y_max": 0.58,
            "workspace.z_min": -0.015, "workspace.z_max": 0.08,
            "workspace.expected_x": 0.20, "workspace.expected_y": 0.33,
            "filter.window": 15, "filter.max_std_x": 0.005,
            "filter.max_std_y": 0.005, "filter.max_std_z": 0.008,
            "filter.reset_distance_m": 0.03,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        g = lambda name: self.get_parameter(name).value
        self.color_topic = str(g("color_topic")); self.depth_topic = str(g("aligned_depth_topic"))
        self.info_topic = str(g("camera_info_topic")); self.optical_frame = str(g("optical_frame"))
        self.base_frame = str(g("base_frame"))
        self.hsv1 = (np.array(g("detection.hsv_lower"), np.uint8), np.array(g("detection.hsv_upper"), np.uint8))
        self.hsv2 = (np.array(g("detection.hsv_lower_2"), np.uint8), np.array(g("detection.hsv_upper_2"), np.uint8))
        self.min_area = float(g("detection.min_area_px")); self.max_area = float(g("detection.max_area_px"))
        self.open_iterations = int(g("detection.open_iterations")); self.close_iterations = int(g("detection.close_iterations"))
        self.inner_erode_px = int(g("detection.inner_erode_px")); self.depth_scale_16u = float(g("depth.scale_16uc1"))
        self.depth_min = float(g("depth.min_m")); self.depth_max = float(g("depth.max_m"))
        self.near_percentile = float(g("depth.near_percentile")); self.object_height = float(g("object.size_z"))
        self.table_z = float(g("table.z")); self.table_tol = float(g("table.top_tolerance"))
        self.bounds = {k: float(g(f"workspace.{k}")) for k in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")}
        self.expected_xy = np.array([float(g("workspace.expected_x")), float(g("workspace.expected_y"))])
        self.window = int(g("filter.window"))
        self.max_std = np.array([float(g("filter.max_std_x")), float(g("filter.max_std_y")), float(g("filter.max_std_z"))])
        self.reset_distance = float(g("filter.reset_distance_m"))

    @staticmethod
    def stamp_sec(msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def on_color(self, msg: Image) -> None:
        with self.lock: self.color_msg = msg

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def depth_meters(self, image: np.ndarray, encoding: str) -> np.ndarray:
        enc = encoding.upper()
        if enc in ("16UC1", "MONO16"): scale = self.depth_scale_16u
        elif enc == "32FC1": scale = 1.0
        else: raise ValueError(f"unsupported depth encoding {encoding}")
        if not self.depth_encoding_logged:
            self.get_logger().info(f"depth encoding={encoding} scale_to_m={scale}")
            self.depth_encoding_logged = True
        return np.asarray(image, dtype=np.float32) * scale

    def contour_measurement(self, contour, depth: np.ndarray, camera_info: CameraInfo, stamp) -> Optional[Dict]:
        area = float(cv2.contourArea(contour))
        if area < self.min_area or area > self.max_area: return None
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-9: return None
        u = float(moments["m10"] / moments["m00"]); v = float(moments["m01"] / moments["m00"])
        h, w = depth.shape[:2]
        if not (1 <= u < w - 1 and 1 <= v < h - 1): return None
        mask = np.zeros((h, w), np.uint8); cv2.drawContours(mask, [contour], -1, 255, -1)
        k = max(1, self.inner_erode_px | 1)
        inner = cv2.erode(mask, np.ones((k, k), np.uint8), iterations=1) > 0
        values = depth[inner]
        values = values[np.isfinite(values) & (values >= self.depth_min) & (values <= self.depth_max)]
        if values.size < 20: return None
        values = values[values <= float(np.percentile(values, self.near_percentile))]
        if values.size < 10: return None
        z = float(np.median(values))
        fx, fy, cx, cy = float(camera_info.k[0]), float(camera_info.k[4]), float(camera_info.k[2]), float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0: return None
        point_camera = PointStamped(); point_camera.header.stamp = stamp; point_camera.header.frame_id = self.optical_frame
        point_camera.point.x = (u - cx) * z / fx; point_camera.point.y = (v - cy) * z / fy; point_camera.point.z = z
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.optical_frame, Time.from_msg(stamp), timeout=Duration(seconds=0.2))
            point_base = do_transform_point(point_camera, transform)
        except TransformException as exc:
            self.get_logger().warn(f"TF unavailable: {exc}", throttle_duration_sec=2.0); return None
        p = np.array([point_base.point.x, point_base.point.y, point_base.point.z], dtype=float); b = self.bounds
        if not (b["x_min"] <= p[0] <= b["x_max"] and b["y_min"] <= p[1] <= b["y_max"] and b["z_min"] <= p[2] <= b["z_max"]): return None
        if abs(p[2] - (self.table_z + self.object_height)) > self.table_tol: return None
        x, y, wb, hb = cv2.boundingRect(contour)
        return {"area": area, "pixel": (u, v), "depth": z, "camera": point_camera,
                "base": p, "bbox": (x, y, wb, hb), "contour": contour,
                "score": float(np.linalg.norm(p[:2] - self.expected_xy))}

    def on_depth(self, msg: Image) -> None:
        with self.lock: color_msg = self.color_msg
        info = self.info
        if color_msg is None or info is None or abs(self.stamp_sec(color_msg) - self.stamp_sec(msg)) > 0.10: return
        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth = self.depth_meters(self.bridge.imgmsg_to_cv2(msg, "passthrough"), msg.encoding)
        except Exception as exc:
            self.get_logger().error(f"image conversion failed: {exc}"); return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv1[0], self.hsv1[1]) | cv2.inRange(hsv, self.hsv2[0], self.hsv2[1])
        kernel = np.ones((3, 3), np.uint8)
        if self.open_iterations: mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=self.open_iterations)
        if self.close_iterations: mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.close_iterations)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = [m for m in (self.contour_measurement(c, depth, info, msg.header.stamp) for c in contours) if m is not None]
        debug = bgr.copy()
        if not candidates:
            self.history.clear(); self.stable_pub.publish(Bool(data=False))
            cv2.putText(debug, "NO VALID CUBE", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")); return
        best = min(candidates, key=lambda item: item["score"]); p = best["base"]
        if self.history and float(np.linalg.norm(p - np.median(np.asarray(self.history), axis=0))) > self.reset_distance: self.history.clear()
        self.history.append(p); values = np.asarray(self.history); median = np.median(values, axis=0); std = np.std(values, axis=0)
        stable = len(self.history) >= self.window and bool(np.all(std <= self.max_std))
        self.point_pub.publish(best["camera"])
        pose = PoseStamped(); pose.header.stamp = msg.header.stamp; pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = [float(v) for v in median]; pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose); self.stable_pub.publish(Bool(data=stable))
        cv2.drawContours(debug, [best["contour"]], -1, (0, 255, 0), 2); u, v = best["pixel"]
        cv2.circle(debug, (round(u), round(v)), 5, (255, 0, 0), -1); x, y, wb, hb = best["bbox"]
        cv2.rectangle(debug, (x, y), (x + wb, y + hb), (0, 255, 255), 1)
        cv2.putText(debug, f"pixel=({u:.1f},{v:.1f}) depth={best['depth']:.3f}m", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2)
        cv2.putText(debug, f"base=({median[0]:.3f},{median[1]:.3f},{median[2]:.3f}) stable={stable}", (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 1_000_000_000:
            self.last_log_ns = now_ns
            pc = best["camera"].point
            self.get_logger().info(
                f"VISION DETECTION pixel=({u:.1f},{v:.1f}) depth={best['depth']:.4f} "
                f"camera=({pc.x:.4f},{pc.y:.4f},{pc.z:.4f}) "
                f"base=({median[0]:.4f},{median[1]:.4f},{median[2]:.4f}) "
                f"std_mm={np.round(std*1000,2).tolist()} stable={stable} n={len(self.history)}")


def main(args=None) -> None:
    rclpy.init(args=args); node = CubeDetector()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
