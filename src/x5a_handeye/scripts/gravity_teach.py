#!/usr/bin/env python3
"""Put ARX X5A into gravity-compensation mode for manual teach/sampling.

Confirmed from ARX X5Controller source:
  InterfacesThread::state::G_COMPENSATION = 3
  RobotCmd.mode is passed to setArmStatus(mode)

Usage:
  source /opt/ros/humble/setup.bash
  source ~/arx/arm/install/setup.bash
  export LD_LIBRARY_PATH=~/repos/arx/ARX_X5/ROS2/X5_ws/install/arx_x5_controller/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH

  # enter gravity mode (drag arm freely)
  ros2 run x5a_handeye gravity_teach

  # or:
  ros2 run x5a_handeye gravity_teach

Keys while running (terminal focus):
  g : gravity compensation (mode=3)
  h : hold / position-control current pose (mode=5)
  o : open gripper (cmd=5.0)
  c : close gripper (cmd=0.0)
  s : save one sample (T_base_tool + T_camera_board if available)
  q : quit (switch to hold first)

Safety:
  - Keep one hand on e-stop.
  - Support the arm when leaving gravity mode if needed.
  - Do not let the board hit the table/camera.
"""
from __future__ import annotations

import json
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from arx5_arm_msg.msg import RobotCmd, RobotStatus


MODE_GO_HOME = 1
MODE_G_COMPENSATION = 3
MODE_END_CONTROL = 4
MODE_POSITION_CONTROL = 5

ARM = [f"joint{i}" for i in range(1, 7)]


def pose_msg_to_T(msg: PoseStamped) -> List[List[float]]:
    p = msg.pose.position
    q = msg.pose.orientation
    # only store xyz+quat; full matrix filled later if needed
    return {
        "xyz": [float(p.x), float(p.y), float(p.z)],
        "quat_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
    }


class GravityTeach(Node):
    def __init__(self) -> None:
        super().__init__("x5a_gravity_teach")
        self.declare_parameter("cmd_topic", "arm_cmd")
        self.declare_parameter("status_topic", "arm_status")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("gripper_open", 5.0)
        self.declare_parameter("gripper_close", 0.0)
        self.declare_parameter("sample_json", str(Path.home() / "arx/x5a_ws/src/x5a_handeye/data/manual_samples_v2.json"))

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.gripper_open = float(self.get_parameter("gripper_open").value)
        self.gripper_close = float(self.get_parameter("gripper_close").value)
        self.sample_json = Path(str(self.get_parameter("sample_json").value))

        self.mode = MODE_G_COMPENSATION
        self.gripper_cmd = self.gripper_open
        self.q: Optional[List[float]] = None
        self.gripper_fb = 0.0
        self.board_pose: Optional[PoseStamped] = None
        self.samples = []

        self.cmd_pub = self.create_publisher(RobotCmd, self.cmd_topic, 10)
        self.create_subscription(RobotStatus, self.status_topic, self.on_status, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(PoseStamped, "/calibration_board/pose", self.on_board, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            "Gravity teach ready. Keys: [g]ravity  [h]old  [o]pen  [c]lose  [s]ample  [q]uit"
        )
        self.get_logger().info(
            f"Publishing {self.cmd_topic} at {self.rate_hz:.0f} Hz, initial mode=G_COMPENSATION(3)"
        )

    def on_status(self, msg: RobotStatus) -> None:
        self.q = [float(msg.joint_pos[i]) for i in range(6)]
        self.gripper_fb = float(msg.joint_pos[6]) if len(msg.joint_pos) > 6 else 0.0

    def on_js(self, msg: JointState) -> None:
        # fallback if only joint_states available
        if self.q is not None:
            return
        try:
            self.q = [float(msg.position[msg.name.index(n)]) for n in ARM]
        except Exception:
            pass

    def on_board(self, msg: PoseStamped) -> None:
        self.board_pose = msg

    def _publish_cmd(self) -> None:
        if self.q is None:
            return
        msg = RobotCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.end_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Keep latest joints as command context; mode decides controller behavior.
        for i in range(6):
            msg.joint_pos[i] = float(self.q[i])
        msg.gripper = float(self.gripper_cmd)
        msg.mode = int(self.mode)
        self.cmd_pub.publish(msg)

    def on_timer(self) -> None:
        self._publish_cmd()

    def set_gravity(self) -> None:
        self.mode = MODE_G_COMPENSATION
        for _ in range(5):
            self._publish_cmd()
            time.sleep(0.01)
        self.get_logger().warn("MODE -> G_COMPENSATION (3). Arm should be drag-able now.")
        print("\n*** GRAVITY ON (mode=3) — drag the arm ***\n", flush=True)

    def set_hold(self) -> None:
        self.mode = MODE_POSITION_CONTROL
        for _ in range(5):
            self._publish_cmd()
            time.sleep(0.01)
        if self.q is not None:
            self.get_logger().warn(
                f"MODE -> POSITION_CONTROL (5), hold q={[round(v, 3) for v in self.q]}"
            )
        else:
            self.get_logger().warn("MODE -> POSITION_CONTROL (5)")
        print("\n*** HOLD ON (mode=5) ***\n", flush=True)

    def _lookup_tool(self):
        tf = self.tf_buffer.lookup_transform(
            "base_link", "tool0", Time(), timeout=Duration(seconds=0.5)
        )
        t = tf.transform.translation
        r = tf.transform.rotation
        return {
            "xyz": [float(t.x), float(t.y), float(t.z)],
            "quat_xyzw": [float(r.x), float(r.y), float(r.z), float(r.w)],
        }

    def _average_pose_dicts(self, poses):
        xyz = np.mean([p["xyz"] for p in poses], axis=0)
        quats = np.array([p["quat_xyzw"] for p in poses], dtype=float)
        for i in range(1, len(quats)):
            if np.dot(quats[0], quats[i]) < 0:
                quats[i] *= -1
        q = np.mean(quats, axis=0)
        q /= np.linalg.norm(q)
        if q[3] < 0:
            q = -q
        return {
            "xyz": [float(v) for v in xyz],
            "quat_xyzw": [float(v) for v in q],
        }

    def _pose_std_mm(self, poses) -> float:
        xyz = np.array([p["xyz"] for p in poses], dtype=float)
        return float(np.linalg.norm(xyz.std(axis=0)) * 1000.0)

    def save_sample(self) -> None:
        """Stable sample: hold pose, average tool TF + board pose, reject if jittery."""
        if self.q is None:
            self.get_logger().error("no joint state yet")
            return

        was_mode = int(self.mode)
        # Always hold while sampling so arm/board are static.
        self.mode = MODE_POSITION_CONTROL
        self.get_logger().info("sampling: temporary HOLD for stable capture...")

        tool_poses = []
        board_poses = []
        joints = []
        t0 = time.time()
        # Collect ~0.8 s of measurements
        while time.time() - t0 < 0.85:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                tool_poses.append(self._lookup_tool())
            except Exception:
                pass
            if self.board_pose is not None:
                board_poses.append(pose_msg_to_T(self.board_pose))
            if self.q is not None:
                joints.append(list(self.q))
            time.sleep(0.03)

        if len(tool_poses) < 5:
            self.mode = was_mode
            self.get_logger().error(f"TF base_link->tool0 unstable/missing (n={len(tool_poses)})")
            return
        if len(board_poses) < 5:
            self.mode = was_mode
            self.get_logger().error(
                f"board pose missing/unstable (n={len(board_poses)}). "
                "Keep ChArUco fully visible and still, then press s again."
            )
            return

        tool_std = self._pose_std_mm(tool_poses)
        board_std = self._pose_std_mm(board_poses)
        if tool_std > 2.0:
            self.mode = was_mode
            self.get_logger().error(
                f"reject sample: tool jitter {tool_std:.2f} mm > 2 mm. Hold arm still."
            )
            return
        if board_std > 1.5:
            self.mode = was_mode
            self.get_logger().error(
                f"reject sample: board jitter {board_std:.2f} mm > 1.5 mm. "
                "Clamp board rigidly on gripper and keep still."
            )
            return

        T_base_tool = self._average_pose_dicts(tool_poses)
        board = self._average_pose_dicts(board_poses)
        q = list(np.mean(np.asarray(joints, dtype=float), axis=0)) if joints else list(self.q)

        sample = {
            "index": len(self.samples),
            "stamp": time.time(),
            "mode": int(was_mode),
            "sample_mode": MODE_POSITION_CONTROL,
            "joints": q,
            "gripper_fb": float(self.gripper_fb),
            "T_base_tool": T_base_tool,
            "T_camera_board": board,
            "quality": {
                "tool_std_mm": tool_std,
                "board_std_mm": board_std,
                "n_tool": len(tool_poses),
                "n_board": len(board_poses),
            },
        }
        self.samples.append(sample)
        self.sample_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "calibration_type": "eye_to_hand",
            "collection": "manual_gravity_teach",
            "samples": self.samples,
        }
        self.sample_json.write_text(json.dumps(payload, indent=2))
        self.get_logger().info(
            f"saved sample #{len(self.samples)} tool={np_round(T_base_tool['xyz'])} "
            f"board={np_round(board['xyz'])} "
            f"tool_std={tool_std:.2f}mm board_std={board_std:.2f}mm -> {self.sample_json}"
        )
        # Restore previous mode (usually gravity) for next drag.
        self.mode = was_mode
        if was_mode == MODE_G_COMPENSATION:
            self.get_logger().info("restored GRAVITY mode; drag to next pose, then s again")


def np_round(xs):
    return [round(float(v), 4) for v in xs]


def get_key(timeout=0.05):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = GravityTeach()

    # terminal raw mode for keypresses
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        node.set_gravity()
        print("\n=== X5A Gravity Teach ===")
        print("g=gravity  h=hold  o=open  c=close  s=sample  q=quit\n")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(0.01)
            if key is None:
                continue
            key = key.lower()
            if key == "g":
                node.set_gravity()
            elif key == "h":
                node.set_hold()
            elif key == "o":
                node.gripper_cmd = node.gripper_open
                node.get_logger().info(f"gripper open cmd={node.gripper_open}")
            elif key == "c":
                node.gripper_cmd = node.gripper_close
                node.get_logger().info(f"gripper close cmd={node.gripper_close}")
            elif key == "s":
                node.save_sample()
            elif key == "q":
                node.set_hold()
                # publish hold a bit before exit
                t0 = time.time()
                while time.time() - t0 < 0.5:
                    rclpy.spin_once(node, timeout_sec=0.05)
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
        print("exited gravity teach")


if __name__ == "__main__":
    main()
