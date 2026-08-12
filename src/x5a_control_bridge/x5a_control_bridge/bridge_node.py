#!/usr/bin/env python3
"""ARX X5A control bridge.

RobotStatus -> /joint_states
FollowJointTrajectory -> arm_cmd (RobotCmd, POSITION_CONTROL)

Does not launch ARX hardware itself. NOT VALIDATED FOR FULL HARDWARE RANGE.
"""
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Sequence

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from arx5_arm_msg.msg import RobotCmd, RobotStatus


def duration_sec(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def lerp(a: Sequence[float], b: Sequence[float], u: float) -> List[float]:
    u = max(0.0, min(1.0, u))
    return [ai + (bi - ai) * u for ai, bi in zip(a, b)]


class X5AControlBridge(Node):
    def __init__(self) -> None:
        super().__init__("x5a_control_bridge")
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("status_topic", "arm_status")
        self.declare_parameter("cmd_topic", "arm_cmd")
        self.declare_parameter("position_mode", 5)
        self.declare_parameter("publish_gripper_as_joint7_joint8", False)
        self.declare_parameter("command_rate_hz", 100.0)
        self.declare_parameter("goal_tolerance_rad", 0.05)
        self.declare_parameter("goal_settle_time_s", 0.5)
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "joint_pos_min", [-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]
        )
        self.declare_parameter(
            "joint_pos_max", [2.618, 3.50, 3.20, 1.55, 1.57, 2.0]
        )
        self.declare_parameter(
            "arm_action_name", "/x5a_arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("hold_on_cancel", True)
        self.declare_parameter("gripper_open_command", 5.0)
        self.declare_parameter("gripper_close_command", 0.0)
        self.declare_parameter("gripper_settle_time_s", 0.8)
        self.declare_parameter("gripper_hold_rate_hz", 50.0)

        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").value
        )
        self.q_min = [float(x) for x in self.get_parameter("joint_pos_min").value]
        self.q_max = [float(x) for x in self.get_parameter("joint_pos_max").value]
        self.position_mode = int(self.get_parameter("position_mode").value)
        self.command_period = 1.0 / float(
            self.get_parameter("command_rate_hz").value
        )
        self.goal_tol = float(self.get_parameter("goal_tolerance_rad").value)
        self.goal_settle = float(self.get_parameter("goal_settle_time_s").value)
        self.hold_on_cancel = bool(self.get_parameter("hold_on_cancel").value)
        self.gripper_open = float(self.get_parameter("gripper_open_command").value)
        self.gripper_close = float(self.get_parameter("gripper_close_command").value)
        self.gripper_settle = float(self.get_parameter("gripper_settle_time_s").value)
        self.gripper_hold_period = 1.0 / float(self.get_parameter("gripper_hold_rate_hz").value)
        self._gripper_cmd = self.gripper_open
        self.publish_gripper = bool(
            self.get_parameter("publish_gripper_as_joint7_joint8").value
        )

        self._lock = threading.Lock()
        self._q_actual = [0.0] * 6
        self._dq_actual = [0.0] * 6
        self._have_status = False
        self._status_stamp = None
        self._gripper = 0.0
        self._active = False
        self._cancel_requested = False

        status_topic = str(self.get_parameter("status_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        action_name = str(self.get_parameter("arm_action_name").value)

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_pub = self.create_publisher(RobotCmd, cmd_topic, 10)
        self.create_subscription(
            RobotStatus,
            status_topic,
            self._on_status,
            qos_profile_sensor_data,
            callback_group=self.cb_group,
        )

        self._action = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            "/x5a_gripper/open",
            self._open_gripper_cb,
            callback_group=self.cb_group,
        )
        self.create_service(
            Trigger,
            "/x5a_gripper/close",
            self._close_gripper_cb,
            callback_group=self.cb_group,
        )
        self.get_logger().info(
            f"X5A bridge ready: {status_topic} -> /joint_states, "
            f"action {action_name} -> {cmd_topic} mode={self.position_mode}; "
            f"gripper open={self.gripper_open} close={self.gripper_close}"
        )

    def _set_gripper(self, command: float, label: str):
        from std_srvs.srv import Trigger as _T
        # keep arm at current joint pose while setting gripper
        with self._lock:
            if self._active:
                return _T.Response(success=False, message="trajectory active; refuse gripper command")
            if not self._have_status:
                return _T.Response(success=False, message="no RobotStatus yet")
            q = list(self._q_actual)
            self._gripper_cmd = float(command)
        t0 = time.time()
        while time.time() - t0 < self.gripper_settle:
            self._send_cmd(q)
            time.sleep(self.gripper_hold_period)
        with self._lock:
            feedback = self._gripper
        msg = f"{label} cmd={command:.3f} feedback={feedback:.3f}"
        self.get_logger().info(msg)
        return _T.Response(success=True, message=msg)

    def _open_gripper_cb(self, request, response):
        res = self._set_gripper(self.gripper_open, "open")
        response.success = res.success
        response.message = res.message
        return response

    def _close_gripper_cb(self, request, response):
        res = self._set_gripper(self.gripper_close, "close")
        response.success = res.success
        response.message = res.message
        return response

    def _on_status(self, msg: RobotStatus) -> None:
        q = [float(msg.joint_pos[i]) for i in range(6)]
        dq = [float(msg.joint_vel[i]) for i in range(6)]
        grip = float(msg.joint_pos[6]) if len(msg.joint_pos) > 6 else 0.0
        with self._lock:
            self._q_actual = q
            self._dq_actual = dq
            self._gripper = grip
            self._have_status = True
            self._status_stamp = msg.header.stamp

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.position = q
        js.velocity = dq
        # Do not invent dual-finger prismatic feedback from single gripper DOF.
        self.js_pub.publish(js)

    def _get_actual(self):
        with self._lock:
            return list(self._q_actual), list(self._dq_actual), self._have_status

    def _within_limits(self, q: Sequence[float]) -> bool:
        for i, v in enumerate(q):
            if not math.isfinite(v):
                return False
            if v < self.q_min[i] - 1e-6 or v > self.q_max[i] + 1e-6:
                return False
        return True

    def _reorder(self, names: Sequence[str], values: Sequence[float]) -> Optional[List[float]]:
        try:
            idx = {n: i for i, n in enumerate(names)}
            return [float(values[idx[n]]) for n in self.joint_names]
        except Exception:
            return None

    def _send_cmd(self, q: Sequence[float]) -> None:
        msg = RobotCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        # end_pos unused in POSITION_CONTROL path but must be finite
        msg.end_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # fixed-size array assignment
        for i in range(6):
            msg.joint_pos[i] = float(q[i])
        msg.gripper = float(self._gripper_cmd)
        msg.mode = int(self.position_mode)
        self.cmd_pub.publish(msg)
        if not hasattr(self, "_cmd_count"):
            self._cmd_count = 0
        self._cmd_count += 1
        if self._cmd_count % 50 == 1:
            self.get_logger().info(
                f"cmd#{self._cmd_count} mode={msg.mode} q={[round(v,4) for v in q]}"
            )

    def _goal_cb(self, goal_request: FollowJointTrajectory.Goal):
        traj = goal_request.trajectory
        if not traj.joint_names or not traj.points:
            self.get_logger().error("reject empty trajectory")
            return GoalResponse.REJECT
        if set(self.joint_names) - set(traj.joint_names):
            self.get_logger().error(
                f"reject missing joints: need {self.joint_names}, got {list(traj.joint_names)}"
            )
            return GoalResponse.REJECT
        last_t = -1.0
        for pt in traj.points:
            q = self._reorder(traj.joint_names, pt.positions)
            if q is None or not self._within_limits(q):
                self.get_logger().error(f"reject invalid point {q}")
                return GoalResponse.REJECT
            t = duration_sec(pt.time_from_start)
            if t + 1e-9 < last_t:
                self.get_logger().error("reject non-monotonic time_from_start")
                return GoalResponse.REJECT
            last_t = t
        with self._lock:
            if self._active:
                self.get_logger().warn("reject: another trajectory is active")
                return GoalResponse.REJECT
            if not self._have_status:
                self.get_logger().error("reject: no RobotStatus yet")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self.get_logger().warn("cancel requested")
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        self._cancel_requested = False
        with self._lock:
            self._active = True
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)

        times: List[float] = []
        positions: List[List[float]] = []
        for pt in traj.points:
            times.append(duration_sec(pt.time_from_start))
            positions.append(self._reorder(names, pt.positions))  # type: ignore

        # If first point time is 0, keep it; ensure trajectory starts near now
        t0 = time.monotonic()
        goal_handle.publish_feedback(feedback)
        last_cmd = positions[0]
        try:
            idx = 0
            while True:
                if self._cancel_requested:
                    if self.hold_on_cancel:
                        q_now, _, _ = self._get_actual()
                        self._send_cmd(q_now)
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    # control_msgs uses int8; canceled path still returns result
                    result.error_string = "canceled; holding current position"
                    with self._lock:
                        self._active = False
                    return result

                elapsed = time.monotonic() - t0
                # advance segment
                while idx + 1 < len(times) and elapsed >= times[idx + 1]:
                    idx += 1
                if elapsed >= times[-1]:
                    last_cmd = positions[-1]
                    self._send_cmd(last_cmd)
                    break
                if idx + 1 < len(times):
                    t_a, t_b = times[idx], times[idx + 1]
                    if t_b <= t_a:
                        u = 1.0
                    else:
                        u = (elapsed - t_a) / (t_b - t_a)
                    cmd = lerp(positions[idx], positions[idx + 1], u)
                else:
                    cmd = positions[-1]
                last_cmd = cmd
                self._send_cmd(cmd)

                q_act, dq_act, _ = self._get_actual()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = list(self.joint_names)
                feedback.desired.positions = list(cmd)
                feedback.actual.positions = list(q_act)
                feedback.actual.velocities = list(dq_act)
                feedback.error.positions = [a - d for a, d in zip(q_act, cmd)]
                goal_handle.publish_feedback(feedback)
                time.sleep(self.command_period)

            # settle
            settle_end = time.monotonic() + self.goal_settle
            while time.monotonic() < settle_end:
                if self._cancel_requested:
                    break
                self._send_cmd(last_cmd)
                time.sleep(self.command_period)

            q_act, _, _ = self._get_actual()
            errs = [abs(a - g) for a, g in zip(q_act, positions[-1])]
            max_err = max(errs) if errs else 0.0
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = list(self.joint_names)
            feedback.desired.positions = list(positions[-1])
            feedback.actual.positions = list(q_act)
            feedback.error.positions = [a - d for a, d in zip(q_act, positions[-1])]
            goal_handle.publish_feedback(feedback)

            if max_err <= self.goal_tol:
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = f"success max_err={max_err:.4f}"
            else:
                goal_handle.abort()
                result.error_code = (
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                )
                result.error_string = (
                    f"goal tolerance violated max_err={max_err:.4f} tol={self.goal_tol}"
                )
            return result
        finally:
            with self._lock:
                self._active = False


def main(args=None):
    rclpy.init(args=args)
    node = X5AControlBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
