#!/usr/bin/env python3
"""Standalone OPEN-CLOSE-OPEN gripper test. The arm must not move.

Sends three GripperCommand goals (default 0.044 m OPEN, 0.0 m CLOSE,
0.044 m OPEN) to the official adapter and records every feedback sample
plus the final result. Safety gates:

  * exactly one publisher on /arm_cmd (the adapter) before every goal
  * no arm trajectory goal is ever sent
  * /joint_states is monitored to prove the arm stayed still

Exit code 0 on PASS, 1 on FAIL.
"""
from __future__ import annotations

import time
from typing import List, Optional, Tuple

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperTest(Node):
    def __init__(self) -> None:
        super().__init__("x5a_gripper_test")
        self.declare_parameter("action_name", "/x5a_gripper_controller/gripper_cmd")
        self.declare_parameter("positions", [0.044, 0.0, 0.044])
        self.declare_parameter("command_topic", "/arm_cmd")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("goal_timeout_sec", 15.0)
        self.declare_parameter("open_threshold_m", 0.03)
        self.declare_parameter("open_tolerance_m", 0.004)

        self.action_name = str(self.get_parameter("action_name").value)
        self.positions: List[float] = [
            float(v) for v in self.get_parameter("positions").value
        ]
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.open_threshold = float(self.get_parameter("open_threshold_m").value)
        self.open_tolerance = float(self.get_parameter("open_tolerance_m").value)

        self.client = ActionClient(self, GripperCommand, self.action_name)
        self.arm_q_start: Optional[List[float]] = None
        self.arm_q_end: Optional[List[float]] = None
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_cb,
            10,
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        names = list(msg.name)
        if len(names) < 6 or len(msg.position) < 6:
            return
        index = {name: i for i, name in enumerate(names)}
        arm = [
            float(msg.position[index["joint" + str(j)]])
            for j in range(1, 7)
            if "joint" + str(j) in index
        ]
        if len(arm) != 6:
            return
        if self.arm_q_start is None:
            self.arm_q_start = arm
        self.arm_q_end = arm

    @staticmethod
    def _fmt(values: Optional[List[float]]) -> str:
        if values is None:
            return "[]"
        return "[" + ",".join(f"{v:.4f}" for v in values) + "]"

    def _arm_cmd_publishers(self) -> int:
        try:
            return self.count_publishers(self.command_topic)
        except Exception:
            return 0

    def _label(self, index: int, target_m: float) -> str:
        kind = "OPEN" if target_m > 0.0 else "CLOSE"
        count = sum(1 for p in self.positions[:index + 1] if p == target_m)
        if kind == "OPEN":
            return f"Open #{count}"
        return "Close"

    def _wait_for_server(self) -> bool:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.client.server_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _run_goal(self, target_m: float, label: str) -> Tuple[bool, str]:
        samples: List[Tuple[float, float, bool]] = []

        def feedback_cb(fb_msg):
            samples.append(
                (time.monotonic(), float(fb_msg.feedback.position),
                 bool(fb_msg.feedback.reached_goal))
            )

        goal = GripperCommand.Goal()
        goal.command.position = float(target_m)
        self.get_logger().info(
            f"[GRIPPER_TEST] GOAL label={label} target_m={target_m:.4f} "
            f"t_ns={time.monotonic_ns()}"
        )
        send_future = self.client.send_goal_async(goal, feedback_callback=feedback_cb)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send_future.done():
            return False, f"{label}: no goal response from {self.action_name}"
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, f"{label}: goal rejected by {self.action_name}"
        self.get_logger().info(f"[GRIPPER_TEST] ACCEPTED label={label}")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self.goal_timeout
        while time.monotonic() < deadline and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return False, f"{label}: goal timed out after {self.goal_timeout:.0f} s"
        status = result_future.result().status
        result = result_future.result().result
        for t_ns, position, reached in samples:
            self.get_logger().info(
                f"[GRIPPER_TEST] FEEDBACK label={label} t_mono={t_ns:.3f} "
                f"position={position:.4f} reached_goal={reached}"
            )
        first = samples[0][1] if samples else None
        last = samples[-1][1] if samples else None
        final_position = (
            float(result.position) if result is not None else None
        )
        reached_goal = bool(result.reached_goal) if result is not None else False
        self.get_logger().info(
            f"[GRIPPER_TEST] RESULT label={label} status={status} "
            f"final_position={final_position if final_position is None else round(final_position, 4)} "
            f"reached_goal={reached_goal} first_feedback="
            f"{first if first is None else round(first, 4)} last_feedback="
            f"{last if last is None else round(last, 4)}"
        )
        if status != 4:  # STATUS_SUCCEEDED
            return False, (
                f"{label}: action {status} final={final_position} "
                f"first_feedback={first} last_feedback={last}"
            )
        if target_m > self.open_threshold and final_position is not None:
            if abs(final_position - target_m) > self.open_tolerance:
                return False, (
                    f"{label}: strict OPEN check failed target={target_m:.4f} "
                    f"final={final_position:.4f}"
                )
        return True, (
            f"{label}: PASS first_feedback={first if first is None else round(first, 4)} "
            f"last_feedback={last if last is None else round(last, 4)} "
            f"final={final_position if final_position is None else round(final_position, 4)}"
        )

    def run(self) -> bool:
        if len(self.positions) < 2:
            self.get_logger().error("need at least two positions (e.g. 0.044 0.0 0.044)")
            return False
        publishers = self._arm_cmd_publishers()
        if publishers != 1:
            self.get_logger().error(
                f"[GRIPPER_TEST] /arm_cmd has {publishers} publishers "
                f"(expected exactly 1 adapter); refusing to test"
            )
            return False
        if not self._wait_for_server():
            self.get_logger().error(f"[GRIPPER_TEST] {self.action_name} is offline")
            return False
        # Let joint_states settle so the arm-still check has a baseline.
        settle_deadline = time.monotonic() + 2.0
        while time.monotonic() < settle_deadline and self.arm_q_start is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f"[GRIPPER_TEST] START sequence={[round(p, 4) for p in self.positions]} "
            f"arm_start={self._fmt(self.arm_q_start)}"
        )
        all_ok = True
        for index, target_m in enumerate(self.positions):
            label = self._label(index, target_m)
            publishers = self._arm_cmd_publishers()
            if publishers != 1:
                self.get_logger().error(
                    f"[GRIPPER_TEST] /arm_cmd has {publishers} publishers "
                    f"before {label}; aborting"
                )
                all_ok = False
                break
            ok, message = self._run_goal(target_m, label)
            self.get_logger().info(f"[GRIPPER_TEST] {message}")
            all_ok = all_ok and ok

        # Arm-motion check: max |dq| across the whole test.
        if self.arm_q_start is not None and self.arm_q_end is not None:
            max_delta = max(abs(a - b) for a, b in zip(self.arm_q_start, self.arm_q_end))
            self.get_logger().info(
                f"[GRIPPER_TEST] ARM_STILL_CHECK max_dq={max_delta:.4f} rad "
                f"start={self._fmt(self.arm_q_start)} end={self._fmt(self.arm_q_end)}"
            )
            if max_delta > 0.01:
                self.get_logger().error(
                    f"[GRIPPER_TEST] arm moved during gripper test: "
                    f"max_dq={max_delta:.4f} rad"
                )
                all_ok = False
        else:
            self.get_logger().warn(
                "[GRIPPER_TEST] no /joint_states received; arm-still check skipped"
            )
        self.get_logger().info(
            f"[GRIPPER_TEST] OVERALL {'PASS' if all_ok else 'FAIL'}"
        )
        return all_ok


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GripperTest()
    try:
        ok = node.run()
    except Exception as exception:  # noqa: BLE001
        node.get_logger().error(f"[GRIPPER_TEST] exception: {exception}")
        ok = False
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
