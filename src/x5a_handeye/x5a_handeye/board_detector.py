#!/usr/bin/env python3
"""Detect ChArUco board pose from RealSense RGB and publish PoseStamped + TF + debug image."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from x5a_handeye.transforms import R_to_quat, rvec_tvec_to_T


def load_board(config_path: str):
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    spec = cfg["board"]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    size = (int(spec["squares_x"]), int(spec["squares_y"]))
    square = float(spec["square_length_m"])
    marker = float(spec["marker_length_m"])
    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard(size, square, marker, dictionary)
    else:
        board = cv2.aruco.CharucoBoard_create(size[0], size[1], square, marker, dictionary)
    return dictionary, board, cfg


class BoardDetector(Node):
    def __init__(self) -> None:
        super().__init__("x5a_board_detector")
        self.declare_parameter("board_config", "")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("min_charuco_corners", 6)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("board_frame", "calibration_board")

        board_config = str(self.get_parameter("board_config").value)
        if not board_config:
            from ament_index_python.packages import get_package_share_directory

            board_config = str(
                Path(get_package_share_directory("x5a_handeye")) / "config" / "board.yaml"
            )
        self.dictionary, self.board, self.cfg = load_board(board_config)
        self.min_corners = int(self.get_parameter("min_charuco_corners").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.board_frame = str(self.get_parameter("board_frame").value)
        self.params = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, "DetectorParameters")
            else cv2.aruco.DetectorParameters_create()
        )
        self.bridge = CvBridge()
        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None
        self.camera_frame = ""
        self.last_T: Optional[np.ndarray] = None
        self.last_quality = 0
        self.tf_broadcaster = TransformBroadcaster(self)

        image_topic = str(self.get_parameter("image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, "/calibration_board/pose", 10)
        self.debug_pub = self.create_publisher(Image, "/x5a_handeye/debug_image", 10)
        self.get_logger().info(
            f"board detector ready image={image_topic} info={info_topic} config={board_config}"
        )

    def on_info(self, msg: CameraInfo) -> None:
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.D = np.array(msg.d, dtype=float).reshape(-1)
        self.camera_frame = msg.header.frame_id

    def detect(self, bgr: np.ndarray):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, "CharucoDetector"):
            det = cv2.aruco.CharucoDetector(self.board)
            charuco_corners, charuco_ids, marker_corners, marker_ids = det.detectBoard(gray)
        else:
            if hasattr(cv2.aruco, "ArucoDetector"):
                mdet = cv2.aruco.ArucoDetector(self.dictionary, self.params)
                marker_corners, marker_ids, rejected = mdet.detectMarkers(gray)
            else:
                marker_corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.dictionary, parameters=self.params
                )
            charuco_corners = charuco_ids = None
            if marker_ids is not None and len(marker_ids) > 0:
                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, gray, self.board
                )
        return marker_corners, marker_ids, charuco_corners, charuco_ids

    def estimate_pose(self, charuco_corners, charuco_ids) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        if (
            charuco_corners is None
            or charuco_ids is None
            or len(charuco_ids) < self.min_corners
            or self.K is None
        ):
            return False, None, None
        if hasattr(cv2.aruco, "estimatePoseCharucoBoard"):
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board, self.K, self.D, None, None
            )
            return bool(ok), rvec, tvec
        if hasattr(self.board, "getChessboardCorners"):
            all_obj = self.board.getChessboardCorners()
        else:
            all_obj = self.board.chessboardCorners
        ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
        obj = np.asarray(all_obj, dtype=np.float32)[ids]
        img = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE)
        return bool(ok), rvec, tvec

    def on_image(self, msg: Image) -> None:
        if self.K is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        marker_corners, marker_ids, charuco_corners, charuco_ids = self.detect(frame)
        debug = frame.copy()
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug, marker_corners, marker_ids)
        n_corners = 0 if charuco_ids is None else int(len(charuco_ids))
        ok, rvec, tvec = self.estimate_pose(charuco_corners, charuco_ids)
        if ok:
            if charuco_corners is not None and charuco_ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(debug, charuco_corners, charuco_ids)
            cv2.drawFrameAxes(debug, self.K, self.D, rvec, tvec, 0.05)
            T = rvec_tvec_to_T(rvec, tvec)
            self.last_T = T
            self.last_quality = n_corners
            xyz = T[:3, 3]
            qx, qy, qz, qw = R_to_quat(T[:3, :3])
            pose = PoseStamped()
            pose.header = msg.header
            if not pose.header.frame_id:
                pose.header.frame_id = self.camera_frame
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pose_pub.publish(pose)
            if self.publish_tf:
                tf = TransformStamped()
                tf.header = pose.header
                tf.child_frame_id = self.board_frame
                tf.transform.translation.x = float(xyz[0])
                tf.transform.translation.y = float(xyz[1])
                tf.transform.translation.z = float(xyz[2])
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(tf)
            cv2.putText(
                debug,
                f"corners={n_corners} t=[{xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f}]",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            self.last_T = None
            self.last_quality = n_corners
            cv2.putText(
                debug,
                f"NO POSE corners={n_corners}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = BoardDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
