#!/usr/bin/env python3
"""Collect eye-to-hand samples: T_base_tool and T_camera_board at diverse poses."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration

from x5a_handeye.transforms import pose_to_T, se3_errors


ARM = [f"joint{i}" for i in range(1, 7)]
QMIN = np.array([-3.14, -0.05, -0.1, -1.6, -1.57, -2.0])
QMAX = np.array([2.618, 3.50, 3.20, 1.55, 1.57, 2.0])


def clamp_q(q: Sequence[float]) -> List[float]:
    out = []
    for i, v in enumerate(q):
        lo = QMIN[i] + 0.05
        hi = QMAX[i] - 0.05
        out.append(float(min(hi, max(lo, v))))
    return out


class SampleCollector(Node):
    def __init__(self) -> None:
        super().__init__("x5a_handeye_sampler")
        self.declare_parameter("output_json", str(Path.home() / "arx/x5a_ws/src/x5a_handeye/data/samples.json"))
        self.declare_parameter("num_samples", 18)
        self.declare_parameter("settle_s", 0.9)
        self.declare_parameter("min_corners", 8)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("board_pose_topic", "/calibration_board/pose")
        self.declare_parameter("motion_duration_s", 3.5)

        self.output_json = Path(str(self.get_parameter("output_json").value))
        self.num_samples = int(self.get_parameter("num_samples").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.min_corners = int(self.get_parameter("min_corners").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.motion_duration_s = float(self.get_parameter("motion_duration_s").value)

        self.q: Optional[List[float]] = None
        self.board_pose: Optional[PoseStamped] = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("board_pose_topic").value), self.on_board, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.fjt = ActionClient(self, FollowJointTrajectory, "/x5a_arm_controller/follow_joint_trajectory")
        self.samples: List[Dict] = []
        self.rejected = 0

    def on_js(self, msg: JointState) -> None:
        try:
            self.q = [float(msg.position[msg.name.index(n)]) for n in ARM]
        except Exception:
            pass

    def on_board(self, msg: PoseStamped) -> None:
        self.board_pose = msg

    def wait_ready(self, timeout=20.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.q is not None and self.fjt.wait_for_server(timeout_sec=0.05):
                return True
        return False

    def lookup_base_tool(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, Time(), timeout=Duration(seconds=1.0)
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            return pose_to_T([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])
        except Exception as e:
            self.get_logger().warn(f"TF base->tool failed: {e}")
            return None

    def stable_board_pose(self, n: int = 12, max_t_std=0.004, max_r_std_deg=1.5) -> Optional[Dict]:
        poses = []
        t0 = time.time()
        while len(poses) < n and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.board_pose is None:
                continue
            p = self.board_pose.pose
            poses.append(
                (
                    [p.position.x, p.position.y, p.position.z],
                    [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
                )
            )
            time.sleep(0.03)
        if len(poses) < max(6, n // 2):
            return None
        xyz = np.array([p[0] for p in poses], float)
        quat = np.array([p[1] for p in poses], float)
        # flip quats to same hemisphere
        for i in range(1, len(quat)):
            if np.dot(quat[0], quat[i]) < 0:
                quat[i] = -quat[i]
        t_std = float(np.linalg.norm(np.std(xyz, axis=0)))
        # approximate rotation std via mean quat distance to mean
        q_mean = np.mean(quat, axis=0)
        q_mean /= np.linalg.norm(q_mean)
        ang = []
        for q in quat:
            d = abs(float(np.dot(q_mean, q)))
            d = min(1.0, d)
            ang.append(math.degrees(2 * math.acos(d)))
        r_std = float(np.std(ang))
        if t_std > max_t_std or r_std > max_r_std_deg:
            self.get_logger().warn(f"unstable board pose t_std={t_std:.4f} r_std={r_std:.2f}")
            return None
        xyz_m = np.mean(xyz, axis=0).tolist()
        q_m = (q_mean / np.linalg.norm(q_mean)).tolist()
        if q_m[3] < 0:
            q_m = [-v for v in q_m]
        return {
            "xyz": xyz_m,
            "quat_xyzw": q_m,
            "t_std": t_std,
            "r_std_deg": r_std,
            "n": len(poses),
        }

    def move_joints(self, target: Sequence[float], duration: Optional[float] = None) -> bool:
        if self.q is None:
            return False
        duration = self.motion_duration_s if duration is None else duration
        traj = JointTrajectory()
        traj.joint_names = ARM
        p0 = JointTrajectoryPoint()
        p0.positions = list(self.q)
        p0.time_from_start = MsgDuration(sec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = clamp_q(target)
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        p1.time_from_start = MsgDuration(sec=sec, nanosec=nsec)
        traj.points = [p0, p1]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        gh = fut.result()
        if not gh or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60)
        res = rf.result().result
        return res.error_code == 0

    def calibration_targets(self, q0: Sequence[float]) -> List[List[float]]:
        """Generate diverse joint targets around current pose with rotation emphasis."""
        targets = []
        # deltas carefully within software limits and mild enough for large board
        candidates = [
            [0.15, 0.10, 0.05, -0.10, 0.20, 0.15],
            [-0.15, 0.15, 0.10, -0.15, -0.20, -0.15],
            [0.25, 0.20, 0.15, -0.20, 0.30, 0.00],
            [-0.25, 0.25, 0.20, -0.25, -0.30, 0.20],
            [0.10, 0.35, 0.25, -0.30, 0.15, -0.25],
            [-0.10, 0.40, 0.30, -0.20, -0.15, 0.30],
            [0.30, 0.15, 0.20, -0.35, 0.25, -0.10],
            [-0.30, 0.20, 0.25, -0.15, -0.25, 0.10],
            [0.05, 0.30, 0.35, -0.25, 0.35, 0.20],
            [-0.05, 0.35, 0.15, -0.30, -0.35, -0.20],
            [0.20, 0.45, 0.20, -0.10, 0.10, 0.25],
            [-0.20, 0.50, 0.25, -0.20, -0.10, -0.25],
            [0.15, 0.25, 0.40, -0.35, 0.20, 0.15],
            [-0.15, 0.30, 0.30, -0.40, -0.20, -0.15],
            [0.00, 0.20, 0.15, -0.15, 0.40, 0.30],
            [0.00, 0.25, 0.20, -0.20, -0.40, -0.30],
            [0.18, 0.35, 0.10, -0.25, 0.25, -0.30],
            [-0.18, 0.40, 0.15, -0.30, -0.25, 0.30],
            [0.12, 0.15, 0.25, -0.20, 0.15, 0.35],
            [-0.12, 0.20, 0.30, -0.25, -0.15, -0.35],
        ]
        for d in candidates:
            targets.append(clamp_q([q0[i] + d[i] for i in range(6)]))
        # include near-home-ish open pose and current
        targets.insert(0, clamp_q(q0))
        return targets[: max(self.num_samples + 5, self.num_samples)]

    def run(self) -> int:
        if not self.wait_ready():
            self.get_logger().error("sampler not ready")
            return 2
        q0 = list(self.q)
        self.get_logger().info(f"start q={np.round(q0,3).tolist()}")
        targets = self.calibration_targets(q0)
        for i, tgt in enumerate(targets):
            if len(self.samples) >= self.num_samples:
                break
            self.get_logger().info(f"move sample candidate {i+1}/{len(targets)} valid={len(self.samples)}")
            if not self.move_joints(tgt):
                self.get_logger().warn("motion failed; skip")
                self.rejected += 1
                continue
            time.sleep(self.settle_s)
            # spin for fresh board pose
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
            T_bt = self.lookup_base_tool()
            board = self.stable_board_pose()
            if T_bt is None or board is None:
                self.get_logger().warn("sample rejected: missing TF or board pose")
                self.rejected += 1
                continue
            T_cb = pose_to_T(board["xyz"], board["quat_xyzw"])
            sample = {
                "index": len(self.samples),
                "stamp": time.time(),
                "joints": list(self.q) if self.q else tgt,
                "T_base_tool": T_bt.tolist(),
                "T_camera_board": T_cb.tolist(),
                "board_quality": {
                    "t_std": board["t_std"],
                    "r_std_deg": board["r_std_deg"],
                    "n": board["n"],
                },
            }
            self.samples.append(sample)
            self.get_logger().info(
                f"recorded sample {len(self.samples)} board_t_std={board['t_std']:.4f}"
            )
        # return near start
        self.move_joints(q0, duration=4.0)
        self.output_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "calibration_type": "eye_to_hand",
            "base_frame": self.base_frame,
            "tool_frame": self.tool_frame,
            "samples": self.samples,
            "rejected": self.rejected,
        }
        self.output_json.write_text(json.dumps(payload, indent=2))
        self.get_logger().info(
            f"saved {len(self.samples)} samples rejected={self.rejected} -> {self.output_json}"
        )
        return 0 if len(self.samples) >= 15 else 1


def main(args=None):
    rclpy.init(args=args)
    node = SampleCollector()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
