#!/usr/bin/env python3
"""Publish static base_link -> camera_link TF from handeye_result.yaml."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

from x5a_handeye.transforms import inv_T, pose_to_T, T_to_pose


class HandeyeTF(Node):
    def __init__(self) -> None:
        super().__init__("x5a_handeye_tf")
        self.declare_parameter("result_yaml", "")
        path = str(self.get_parameter("result_yaml").value)
        if not path:
            from ament_index_python.packages import get_package_share_directory

            path = str(Path(get_package_share_directory("x5a_handeye")) / "config" / "handeye_result.yaml")
        self.result = yaml.safe_load(Path(path).read_text())
        self.T_base_optical = np.asarray(self.result["T_base_camera"], float)
        self.static_br = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.try_publish)
        self.published = False
        self.get_logger().info(f"loaded handeye result from {path}")

    def try_publish(self):
        if self.published:
            return
        optical = self.result.get("camera_frame", "camera_color_optical_frame")
        # RealSense frames often: camera_link -> camera_color_frame -> camera_color_optical_frame
        # We calibrated base->optical. Convert to base->camera_link if possible.
        parent = "base_link"
        child = self.result.get("camera_root_frame", "camera_link")
        try:
            tf = self.tf_buffer.lookup_transform(
                child, optical, Time(), timeout=Duration(seconds=0.5)
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            T_link_optical = pose_to_T([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])
            T_base_link = self.T_base_optical @ inv_T(T_link_optical)
            xyz, quat = T_to_pose(T_base_link)
            self.get_logger().info("publishing base_link -> camera_link using optical calibration")
        except Exception as e:
            # The calibrated transform is base->optical, while RealSense owns
            # the optical frame. Never create a second optical-frame parent or
            # a substitute frame: wait until the camera's link->optical chain
            # exists, then publish the one allowed transform base->camera_link.
            self.get_logger().warn(
                f"camera_link TF unavailable ({e}); waiting for RealSense TF",
                throttle_duration_sec=5.0,
            )
            return
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        msg.transform.rotation.x = float(quat[0])
        msg.transform.rotation.y = float(quat[1])
        msg.transform.rotation.z = float(quat[2])
        msg.transform.rotation.w = float(quat[3])
        self.static_br.sendTransform(msg)
        self.published = True
        self.get_logger().info(
            f"static TF {parent}->{child} xyz={[round(v,4) for v in xyz]} quat={[round(v,4) for v in quat]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
