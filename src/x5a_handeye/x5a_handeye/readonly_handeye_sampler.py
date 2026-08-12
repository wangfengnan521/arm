#!/usr/bin/env python3
"""Read-only manual eye-to-hand sample collector for the ARX X5A.

This node deliberately has no RobotCmd import, no action/service client and no
application publisher.  It only subscribes to the official robot feedback and
the ChArUco detector pose, then writes accepted samples to a JSON file.

Robot pose source:
  /arm_status.joint_pos -> URDF FK -> base_link-to-tool0

Keys:
  s  collect one sample while the operator keeps the arm and board still
  q  quit without changing the robot mode
"""
from __future__ import annotations

import json
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from arx5_arm_msg.msg import RobotStatus
from x5a_handeye.transforms import T_to_pose
from x5a_handeye.x5a_fk import fk_base_tool0


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> List[float]:
    """Convert fixed-axis roll/pitch/yaw to quaternion [x, y, z, w]."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = np.asarray(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=float,
    )
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return [float(v) for v in q]


def joint_pose(joints: Sequence[float]) -> Dict[str, List[float]]:
    xyz, quat = T_to_pose(fk_base_tool0(joints))
    return {"xyz": xyz, "quat_xyzw": quat}


def board_pose(msg: PoseStamped) -> Dict[str, List[float]]:
    p = msg.pose.position
    q = msg.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError("invalid board pose")
    quat = np.asarray(values[3:], dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-9:
        raise ValueError("zero board quaternion")
    quat /= norm
    if quat[3] < 0.0:
        quat = -quat
    return {
        "xyz": [float(v) for v in values[:3]],
        "quat_xyzw": [float(v) for v in quat],
    }


def average_pose(poses: Sequence[Dict[str, List[float]]]) -> Dict[str, List[float]]:
    xyz = np.mean(np.asarray([p["xyz"] for p in poses], dtype=float), axis=0)
    quats = np.asarray([p["quat_xyzw"] for p in poses], dtype=float)
    for i in range(1, len(quats)):
        if float(np.dot(quats[0], quats[i])) < 0.0:
            quats[i] *= -1.0
    quat = np.mean(quats, axis=0)
    quat /= np.linalg.norm(quat)
    if quat[3] < 0.0:
        quat = -quat
    return {
        "xyz": [float(v) for v in xyz],
        "quat_xyzw": [float(v) for v in quat],
    }


def translation_std_mm(poses: Sequence[Dict[str, List[float]]]) -> float:
    xyz = np.asarray([p["xyz"] for p in poses], dtype=float)
    return float(np.linalg.norm(np.std(xyz, axis=0)) * 1000.0)


def rotation_std_deg(poses: Sequence[Dict[str, List[float]]]) -> float:
    mean = average_pose(poses)["quat_xyzw"]
    angles = []
    for pose in poses:
        dot = abs(float(np.dot(mean, pose["quat_xyzw"])))
        angles.append(math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot)))))
    return float(np.std(angles))


class ReadOnlyHandeyeSampler(Node):
    def __init__(self) -> None:
        super().__init__("x5a_readonly_handeye_sampler")
        self.declare_parameter("status_topic", "/arm_status")
        self.declare_parameter("board_pose_topic", "/calibration_board/pose")
        self.declare_parameter(
            "sample_json",
            str(
                Path.home()
                / "arx/x5a_ws/src/x5a_handeye/data/manual_samples_readonly.json"
            ),
        )
        self.declare_parameter("capture_s", 0.8)
        self.declare_parameter("min_robot_messages", 10)
        self.declare_parameter("min_board_messages", 5)
        self.declare_parameter("max_robot_translation_std_mm", 2.0)
        self.declare_parameter("max_board_translation_std_mm", 1.5)
        self.declare_parameter("max_robot_rotation_std_deg", 1.0)
        self.declare_parameter("max_board_rotation_std_deg", 1.0)
        self.declare_parameter("append_existing", True)

        self.status_topic = str(self.get_parameter("status_topic").value)
        self.board_topic = str(self.get_parameter("board_pose_topic").value)
        self.sample_json = Path(str(self.get_parameter("sample_json").value))
        self.capture_s = float(self.get_parameter("capture_s").value)
        self.min_robot = int(self.get_parameter("min_robot_messages").value)
        self.min_board = int(self.get_parameter("min_board_messages").value)
        self.max_robot_t = float(
            self.get_parameter("max_robot_translation_std_mm").value
        )
        self.max_board_t = float(
            self.get_parameter("max_board_translation_std_mm").value
        )
        self.max_robot_r = float(self.get_parameter("max_robot_rotation_std_deg").value)
        self.max_board_r = float(self.get_parameter("max_board_rotation_std_deg").value)

        self.latest_q: Optional[List[float]] = None
        self.latest_gripper = 0.0
        self.have_board = False
        self.capturing = False
        self.robot_records: List[Dict] = []
        self.board_records: List[Dict] = []
        self.samples: List[Dict] = []

        if bool(self.get_parameter("append_existing").value) and self.sample_json.exists():
            try:
                payload = json.loads(self.sample_json.read_text())
                existing = payload.get("samples", [])
                if isinstance(existing, list):
                    self.samples = existing
            except Exception as exc:
                self.get_logger().warn(f"could not load existing samples: {exc}")

        # These are the only application subscriptions.  There is intentionally
        # no publisher/client that can reach /arm_cmd, /arx_joy or an action.
        self.create_subscription(
            RobotStatus, self.status_topic, self.on_status, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, self.board_topic, self.on_board, qos_profile_sensor_data
        )

        self.get_logger().info(
            "READ-ONLY sampler: subscriptions=%s,%s; arm command publishers=NONE"
            % (self.status_topic, self.board_topic)
        )
        self.get_logger().info(
            f"output={self.sample_json}; existing samples={len(self.samples)}"
        )

    def on_status(self, msg: RobotStatus) -> None:
        if len(msg.joint_pos) < 6:
            self.get_logger().warn("/arm_status has fewer than six joints")
            return
        self.latest_q = [float(msg.joint_pos[i]) for i in range(6)]
        try:
            pose = joint_pose(self.latest_q)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        if len(msg.joint_pos) >= 7:
            self.latest_gripper = float(msg.joint_pos[6])
        if self.capturing:
            self.robot_records.append(
                {
                    "pose": pose,
                    "joints": list(self.latest_q) if self.latest_q is not None else None,
                    "gripper": self.latest_gripper,
                }
            )

    def on_board(self, msg: PoseStamped) -> None:
        try:
            pose = board_pose(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        self.have_board = True
        if self.capturing:
            self.board_records.append(
                {"pose": pose, "frame_id": str(msg.header.frame_id)}
            )

    def ready(self) -> bool:
        if self.latest_q is None:
            self.get_logger().error(f"no robot feedback on {self.status_topic}")
            return False
        if not self.have_board:
            self.get_logger().error(f"no board pose on {self.board_topic}")
            return False
        return True

    def save_sample(self) -> None:
        if not self.ready():
            return

        self.robot_records = []
        self.board_records = []
        self.capturing = True
        self.get_logger().info(
            f"capturing feedback for {self.capture_s:.2f}s; keep arm and board still"
        )
        deadline = time.monotonic() + self.capture_s
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.03)
        finally:
            self.capturing = False

        nr, nb = len(self.robot_records), len(self.board_records)
        if nr < self.min_robot or nb < self.min_board:
            self.get_logger().error(
                f"rejected: too few fresh messages robot={nr}/{self.min_robot} "
                f"board={nb}/{self.min_board}"
            )
            return

        robot_poses = [record["pose"] for record in self.robot_records]
        board_poses = [record["pose"] for record in self.board_records]
        robot_t = translation_std_mm(robot_poses)
        board_t = translation_std_mm(board_poses)
        robot_r = rotation_std_deg(robot_poses)
        board_r = rotation_std_deg(board_poses)
        if robot_t > self.max_robot_t or robot_r > self.max_robot_r:
            self.get_logger().error(
                f"rejected: robot still moving t_std={robot_t:.2f}mm "
                f"r_std={robot_r:.2f}deg"
            )
            return
        if board_t > self.max_board_t or board_r > self.max_board_r:
            self.get_logger().error(
                f"rejected: board unstable t_std={board_t:.2f}mm "
                f"r_std={board_r:.2f}deg"
            )
            return

        joints = [r["joints"] for r in self.robot_records if r["joints"] is not None]
        q = (
            np.mean(np.asarray(joints, dtype=float), axis=0).tolist()
            if joints
            else None
        )
        frame_ids = [r["frame_id"] for r in self.board_records if r["frame_id"]]
        sample = {
            "index": len(self.samples),
            "stamp": time.time(),
            "joints": q,
            "gripper_fb": float(
                np.mean([r["gripper"] for r in self.robot_records])
            ),
            "T_base_tool": average_pose(robot_poses),
            "T_camera_board": average_pose(board_poses),
            "quality": {
                "robot_translation_std_mm": robot_t,
                "robot_rotation_std_deg": robot_r,
                "board_translation_std_mm": board_t,
                "board_rotation_std_deg": board_r,
                "n_robot": nr,
                "n_board": nb,
            },
            "source": {
                "robot": f"{self.status_topic}.joint_pos -> URDF FK base_link->tool0",
                "board": self.board_topic,
                "board_frame_id": frame_ids[-1] if frame_ids else "",
                "control_commands_sent": False,
            },
        }
        self.samples.append(sample)
        self.sample_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "calibration_type": "eye_to_hand",
            "collection": "manual_gravity_readonly_urdf_fk",
            "base_frame": "base_link",
            "tool_frame": "tool0",
            "robot_pose_frame": "tool0",
            "control_commands_sent": False,
            "samples": self.samples,
        }
        self.sample_json.write_text(json.dumps(payload, indent=2))
        self.get_logger().info(
            f"SAVED #{len(self.samples)} robot_std={robot_t:.2f}mm/{robot_r:.2f}deg "
            f"board_std={board_t:.2f}mm/{board_r:.2f}deg"
        )


def get_key(timeout: float = 0.03) -> Optional[str]:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReadOnlyHandeyeSampler()
    if not sys.stdin.isatty():
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("interactive terminal required")

    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print("\n=== X5A READ-ONLY HAND-EYE SAMPLER ===")
        print("No /arm_cmd, /arx_joy, service or action commands are created.")
        print("Keep gravity mode as-is. Stop moving, then press s. q=quit.\n")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = get_key()
            if key is None:
                continue
            key = key.lower()
            if key == "s":
                node.save_sample()
            elif key == "q":
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)
        node.destroy_node()
        rclpy.shutdown()
        print("read-only sampler exited; robot mode was not changed")


if __name__ == "__main__":
    main()
