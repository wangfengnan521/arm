#!/usr/bin/env python3
"""Expose standard MoveIt interfaces over the official ARX X5 ROS 2 API.

Hardware-facing topics are strictly the vendor interfaces:
  /arm_status (arx5_arm_msg/RobotStatus)
  /arm_cmd    (arx5_arm_msg/RobotCmd)
"""
from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from arx5_arm_msg.msg import RobotCmd, RobotStatus


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
STATE_JOINTS = ARM_JOINTS + ["joint7", "joint8"]


def point_time(point: JointTrajectoryPoint) -> float:
    return float(point.time_from_start.sec) + 1e-9 * float(point.time_from_start.nanosec)


class OfficialTrajectoryAdapter(Node):
    def __init__(self) -> None:
        super().__init__("x5a_official_trajectory_adapter")
        self.cb = ReentrantCallbackGroup()
        self._declare_parameters()
        self._load_parameters()

        self.lock = threading.Lock()
        self.q: Optional[List[float]] = None
        self.dq: List[float] = [0.0] * 6
        self.effort: List[float] = [0.0] * 6
        self.end_pos: List[float] = [0.0] * 6
        self.gripper = 0.0
        self.last_status_time = 0.0
        self.state_started_time = 0.0
        self.last_state_jump_time = time.monotonic()
        self.state_fault = ""
        self.goal_active = False

        latest_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.js_pub = self.create_publisher(
            JointState, self.joint_state_topic, latest_reliable
        )
        self.cmd_pub = self.create_publisher(
            RobotCmd, self.command_topic, latest_reliable
        )
        self.create_subscription(
            RobotStatus,
            self.status_topic,
            self._status_cb,
            latest_best_effort,
            callback_group=self.cb,
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self.cb,
        )
        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            self.gripper_action_name,
            execute_callback=self._execute_gripper,
            goal_callback=self._gripper_goal,
            cancel_callback=self._cancel,
            callback_group=self.cb,
        )
        self.get_logger().info(
            f"MoveIt adapter ready: {self.action_name} -> official {self.command_topic}; "
            f"gripper={self.gripper_action_name}; official {self.status_topic} "
            f"-> {self.joint_state_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("official_status_topic", "/arm_status")
        self.declare_parameter("official_command_topic", "/arm_cmd")
        self.declare_parameter("action_name", "/x5a_arm_controller/follow_joint_trajectory")
        self.declare_parameter(
            "gripper_action_name", "/x5a_gripper_controller/gripper_cmd"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("position_mode", 5)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("joint_state_rate_hz", 30.0)
        self.declare_parameter("status_timeout", 0.25)
        self.declare_parameter("state_settle_sec", 2.0)
        self.declare_parameter("max_status_jump_rad", 0.08)
        self.declare_parameter("start_tolerance_rad", 0.15)
        self.declare_parameter("goal_tolerance_rad", 0.01)
        self.declare_parameter("goal_time_tolerance_sec", 2.0)
        self.declare_parameter("tracking_grace_sec", 0.75)
        self.declare_parameter("tracking_tolerance_rad", 0.15)
        self.declare_parameter("response_command_delta_rad", 0.03)
        self.declare_parameter("response_actual_delta_rad", 0.004)
        self.declare_parameter("response_timeout_sec", 0.75)
        self.declare_parameter("max_joint_speed_rad_s", 1.0)
        self.declare_parameter("gripper_closed_readout", 0.0)
        self.declare_parameter("gripper_open_readout", 5.0)
        self.declare_parameter("gripper_finger_max_m", 0.044)
        self.declare_parameter("gripper_command_duration_sec", 0.8)
        self.declare_parameter("gripper_goal_tolerance_m", 0.004)

    def _load_parameters(self) -> None:
        g = lambda name: self.get_parameter(name).value
        self.status_topic = str(g("official_status_topic"))
        self.command_topic = str(g("official_command_topic"))
        self.action_name = str(g("action_name"))
        self.gripper_action_name = str(g("gripper_action_name"))
        self.joint_state_topic = str(g("joint_state_topic"))
        self.position_mode = int(g("position_mode"))
        self.rate_hz = float(g("command_rate_hz"))
        self.state_rate_hz = float(g("joint_state_rate_hz"))
        self.state_period = 1.0 / max(1.0, self.state_rate_hz)
        self.status_timeout = float(g("status_timeout"))
        self.state_settle = float(g("state_settle_sec"))
        self.max_status_jump = float(g("max_status_jump_rad"))
        self.start_tolerance = float(g("start_tolerance_rad"))
        self.goal_tolerance = float(g("goal_tolerance_rad"))
        self.goal_time_tolerance = float(g("goal_time_tolerance_sec"))
        self.tracking_grace = float(g("tracking_grace_sec"))
        self.tracking_tolerance = float(g("tracking_tolerance_rad"))
        self.response_command_delta = float(g("response_command_delta_rad"))
        self.response_actual_delta = float(g("response_actual_delta_rad"))
        self.response_timeout = float(g("response_timeout_sec"))
        self.max_speed = float(g("max_joint_speed_rad_s"))
        self.gripper_closed = float(g("gripper_closed_readout"))
        self.gripper_open = float(g("gripper_open_readout"))
        self.finger_max = float(g("gripper_finger_max_m"))
        self.gripper_duration = float(g("gripper_command_duration_sec"))
        self.gripper_tolerance = float(g("gripper_goal_tolerance_m"))

    def finger_position(self, readout: float) -> float:
        span = self.gripper_open - self.gripper_closed
        if abs(span) < 1e-9:
            return 0.0
        fraction = (readout - self.gripper_closed) / span
        return self.finger_max * max(0.0, min(1.0, fraction))

    def gripper_readout(self, finger_position: float) -> float:
        if self.finger_max <= 1e-9:
            return self.gripper_closed
        fraction = max(0.0, min(1.0, finger_position / self.finger_max))
        return self.gripper_closed + fraction * (
            self.gripper_open - self.gripper_closed
        )

    def _status_cb(self, msg: RobotStatus) -> None:
        if len(msg.joint_pos) < 6:
            return
        received_at = time.monotonic()
        with self.lock:
            # The vendor publishes from a 1 ms timer.  MoveIt only needs a
            # fresh latest sample; processing every vendor message saturated
            # a CPU core and could starve the driver's CAN receive thread.
            if (
                self.last_status_time > 0.0
                and received_at - self.last_status_time < self.state_period
            ):
                return
            new_q = [float(msg.joint_pos[i]) for i in range(6)]
            if self.q is None:
                self.state_started_time = received_at
                self.last_state_jump_time = received_at
            else:
                jump = max(abs(a - b) for a, b in zip(new_q, self.q))
                if jump > self.max_status_jump:
                    self.last_state_jump_time = received_at
                    self.state_fault = f"official state jumped by {jump:.3f} rad"
                elif (
                    self.state_fault
                    and received_at - self.last_state_jump_time >= self.state_settle
                ):
                    self.state_fault = ""
            self.q = new_q
            self.dq = [float(msg.joint_vel[i]) for i in range(6)] if len(msg.joint_vel) >= 6 else [0.0] * 6
            self.effort = [float(msg.joint_cur[i]) for i in range(6)] if len(msg.joint_cur) >= 6 else [0.0] * 6
            self.end_pos = [float(msg.end_pos[i]) for i in range(6)]
            self.gripper = float(msg.joint_pos[6]) if len(msg.joint_pos) > 6 else self.gripper
            self.last_status_time = received_at
            q = list(self.q); dq = list(self.dq); effort = list(self.effort)
            finger = self.finger_position(self.gripper)
        js = JointState()
        js.header = msg.header
        js.name = list(STATE_JOINTS)
        js.position = q + [finger, finger]
        js.velocity = dq + [0.0, 0.0]
        js.effort = effort + [0.0, 0.0]
        self.js_pub.publish(js)

    def snapshot(self) -> Tuple[Optional[List[float]], List[float], float, float]:
        with self.lock:
            q = None if self.q is None else list(self.q)
            return q, list(self.end_pos), float(self.gripper), float(self.last_status_time)

    def readiness_fault(self) -> str:
        now = time.monotonic()
        with self.lock:
            if self.q is None:
                return "no official RobotStatus"
            if now - self.last_status_time > self.status_timeout:
                return "official RobotStatus is stale"
            if now - self.state_started_time < self.state_settle:
                return "official state has not settled after startup"
            if self.state_fault or now - self.last_state_jump_time < self.state_settle:
                return self.state_fault or "official state has not settled after a jump"
        return ""

    def _goal(self, request: FollowJointTrajectory.Goal) -> GoalResponse:
        traj = request.trajectory
        if self.goal_active:
            self.get_logger().error("rejecting trajectory: another goal is active")
            return GoalResponse.REJECT
        if list(traj.joint_names) != ARM_JOINTS or not traj.points:
            self.get_logger().error("rejecting trajectory: expected joint1..joint6 in canonical order")
            return GoalResponse.REJECT
        previous = -1.0
        for p in traj.points:
            t = point_time(p)
            if len(p.positions) != 6 or t < previous or not all(math.isfinite(v) for v in p.positions):
                self.get_logger().error("rejecting malformed trajectory")
                return GoalResponse.REJECT
            previous = t
        fault = self.readiness_fault()
        if fault:
            self.get_logger().error(f"rejecting trajectory: {fault}")
            return GoalResponse.REJECT
        # This adapter must be the only writer to the official hardware command topic.
        if self.count_publishers(self.command_topic) != 1:
            self.get_logger().error(
                f"rejecting trajectory: {self.command_topic} has "
                f"{self.count_publishers(self.command_topic)} publishers (expected adapter only)"
            )
            return GoalResponse.REJECT
        self.goal_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _gripper_goal(self, request: GripperCommand.Goal) -> GoalResponse:
        position = float(request.command.position)
        if self.goal_active:
            self.get_logger().error("rejecting gripper command: another command is active")
            return GoalResponse.REJECT
        if not math.isfinite(position) or position < -1e-6 or position > self.finger_max + 1e-6:
            self.get_logger().error(
                f"rejecting gripper position {position:.4f} m; expected 0..{self.finger_max:.4f} m"
            )
            return GoalResponse.REJECT
        fault = self.readiness_fault()
        if fault:
            self.get_logger().error(f"rejecting gripper command: {fault}")
            return GoalResponse.REJECT
        if self.count_publishers(self.command_topic) != 1:
            self.get_logger().error(
                f"rejecting gripper command: {self.command_topic} has "
                f"{self.count_publishers(self.command_topic)} publishers"
            )
            return GoalResponse.REJECT
        self.goal_active = True
        return GoalResponse.ACCEPT

    def _publish_command(self, joints: Sequence[float], end_pos: Sequence[float], gripper: float) -> None:
        msg = RobotCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(6):
            msg.joint_pos[i] = float(joints[i])
            msg.end_pos[i] = float(end_pos[i])
        msg.gripper = float(gripper)
        msg.mode = self.position_mode
        self.cmd_pub.publish(msg)

    def _execute_gripper(self, goal_handle):
        result = GripperCommand.Result()
        target_m = max(
            0.0, min(self.finger_max, float(goal_handle.request.command.position))
        )
        target_readout = self.gripper_readout(target_m)
        try:
            deadline = time.monotonic() + self.gripper_duration
            period = 1.0 / max(5.0, self.rate_hz)
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                q, end_pos, actual_readout, stamp = self.snapshot()
                if q is None or time.monotonic() - stamp > self.status_timeout:
                    goal_handle.abort()
                    return result
                if self.count_publishers(self.command_topic) != 1:
                    goal_handle.abort()
                    return result
                self._publish_command(q, end_pos, target_readout)
                actual_m = self.finger_position(actual_readout)
                feedback = GripperCommand.Feedback()
                feedback.position = actual_m
                feedback.effort = 0.0
                feedback.stalled = False
                feedback.reached_goal = abs(actual_m - target_m) <= self.gripper_tolerance
                goal_handle.publish_feedback(feedback)
                time.sleep(period)

            _, _, actual_readout, _ = self.snapshot()
            result.position = self.finger_position(actual_readout)
            result.effort = 0.0
            result.stalled = False
            # The vendor API exposes a position target but no independent
            # grasp-complete flag. A complete, continuously published command
            # is therefore the action success criterion; the measured position
            # remains in the result for diagnostics.
            result.reached_goal = True
            goal_handle.succeed()
            self.get_logger().info(
                f"gripper command complete: target={target_m:.4f} m "
                f"feedback={result.position:.4f} m"
            )
            return result
        finally:
            self.goal_active = False

    @staticmethod
    def _desired_at(points: List[Tuple[float, List[float]]], elapsed: float) -> List[float]:
        if elapsed <= points[0][0]:
            return list(points[0][1])
        for index in range(len(points) - 1):
            t0, q0 = points[index]; t1, q1 = points[index + 1]
            if elapsed <= t1:
                alpha = 1.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
                return [a + (b - a) * alpha for a, b in zip(q0, q1)]
        return list(points[-1][1])

    def _feedback(self, goal_handle, desired: Sequence[float], actual: Sequence[float]) -> None:
        fb = FollowJointTrajectory.Feedback()
        fb.header.stamp = self.get_clock().now().to_msg()
        fb.joint_names = list(ARM_JOINTS)
        fb.desired.positions = [float(v) for v in desired]
        fb.actual.positions = [float(v) for v in actual]
        fb.error.positions = [float(a - d) for a, d in zip(actual, desired)]
        goal_handle.publish_feedback(fb)

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            q0, end_pos, gripper, _ = self.snapshot()
            if q0 is None:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "no official RobotStatus"
                goal_handle.abort(); return result
            raw = [(point_time(p), [float(v) for v in p.positions]) for p in goal_handle.request.trajectory.points]
            if raw[0][0] > 1e-6:
                points = [(0.0, list(q0))] + raw
            else:
                points = raw
            start_error = max(abs(a - b) for a, b in zip(q0, points[0][1]))
            if start_error > self.start_tolerance:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f"start error {start_error:.3f} rad"
                goal_handle.abort(); return result
            for (ta, qa), (tb, qb) in zip(points, points[1:]):
                if tb > ta:
                    speed = max(abs(b - a) / (tb - ta) for a, b in zip(qa, qb))
                    if speed > self.max_speed:
                        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                        result.error_string = f"trajectory speed {speed:.3f} exceeds {self.max_speed:.3f} rad/s"
                        goal_handle.abort(); return result

            duration = points[-1][0]
            period = 1.0 / max(5.0, self.rate_hz)
            started = time.monotonic()
            command_count = 0
            response_started: List[Optional[float]] = [None] * 6
            while True:
                if goal_handle.is_cancel_requested:
                    q, end_pos, gripper, _ = self.snapshot()
                    if q is not None: self._publish_command(q, end_pos, gripper)
                    goal_handle.canceled(); return result
                elapsed = min(time.monotonic() - started, duration)
                desired = self._desired_at(points, elapsed)
                actual, latest_end, latest_gripper, stamp = self.snapshot()
                if actual is None or time.monotonic() - stamp > self.status_timeout:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "official RobotStatus lost during execution"
                    goal_handle.abort(); return result
                if self.count_publishers(self.command_topic) != 1:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "another /arm_cmd publisher appeared"
                    goal_handle.abort(); return result
                fault = self.readiness_fault()
                if fault:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = fault
                    self.get_logger().error(f"aborting trajectory: {fault}")
                    goal_handle.abort(); return result
                if elapsed >= self.tracking_grace:
                    tracking_errors = [abs(a - d) for a, d in zip(actual, desired)]
                    if max(tracking_errors) > self.tracking_tolerance:
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        result.error_string = (
                            f"tracking error {max(tracking_errors):.3f} rad"
                        )
                        self.get_logger().error(
                            f"aborting trajectory: {result.error_string}; "
                            f"per_joint={[round(v, 4) for v in tracking_errors]}"
                        )
                        self._publish_command(actual, latest_end, gripper)
                        goal_handle.abort(); return result
                    response_now = time.monotonic()
                    for index, (commanded, moved) in enumerate(
                        zip(
                            (abs(d - s) for d, s in zip(desired, q0)),
                            (abs(a - s) for a, s in zip(actual, q0)),
                        )
                    ):
                        if commanded < self.response_command_delta:
                            response_started[index] = None
                            continue
                        if response_started[index] is None:
                            response_started[index] = response_now
                            continue
                        if (
                            response_now - response_started[index]
                            >= self.response_timeout
                            and moved < self.response_actual_delta
                        ):
                            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                            result.error_string = (
                                f"joint{index + 1} did not respond to position commands"
                            )
                            self.get_logger().error(
                                f"aborting trajectory: {result.error_string}"
                            )
                            self._publish_command(actual, latest_end, gripper)
                            goal_handle.abort(); return result
                self._publish_command(desired, latest_end, gripper)
                command_count += 1
                self._feedback(goal_handle, desired, actual)
                if elapsed >= duration:
                    break
                time.sleep(period)

            deadline = time.monotonic() + self.goal_time_tolerance
            final = points[-1][1]
            error = float("inf")
            while time.monotonic() < deadline:
                actual, latest_end, _, stamp = self.snapshot()
                if actual is None or time.monotonic() - stamp > self.status_timeout:
                    break
                self._publish_command(final, latest_end, gripper)
                command_count += 1
                error = max(abs(a - b) for a, b in zip(actual, final))
                self._feedback(goal_handle, final, actual)
                if error <= self.goal_tolerance:
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    goal_handle.succeed()
                    self.get_logger().info(
                        f"trajectory complete: max error={error:.4f} rad, "
                        f"commands={command_count} at {self.rate_hz:.1f} Hz"
                    )
                    return result
                time.sleep(period)
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = f"goal error {error:.3f} rad"
            if actual is not None:
                per_joint = [round(a - b, 4) for a, b in zip(actual, final)]
                self.get_logger().error(
                    f"trajectory goal tolerance violated: actual={[round(v, 4) for v in actual]} "
                    f"target={[round(v, 4) for v in final]} error={per_joint}"
                )
            goal_handle.abort()
            return result
        finally:
            self.goal_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfficialTrajectoryAdapter()
    # Two threads are sufficient: one action execution thread and one latest
    # status callback thread. More threads only increase contention in Python.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
