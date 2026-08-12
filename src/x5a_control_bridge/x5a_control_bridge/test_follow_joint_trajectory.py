#!/usr/bin/env python3
"""Small safe FollowJointTrajectory test near current pose."""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
QMIN = [-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]
QMAX = [2.618, 3.50, 3.20, 1.55, 1.57, 2.0]


class TestNode(Node):
    def __init__(self):
        super().__init__("x5a_fjt_test")
        self.q = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.ac = ActionClient(
            self, FollowJointTrajectory, "/x5a_arm_controller/follow_joint_trajectory"
        )

    def _cb(self, msg: JointState):
        if not msg.name:
            return
        try:
            self.q = [float(msg.position[msg.name.index(n)]) for n in ARM]
        except Exception:
            pass


def pick_safe_delta(q):
    # choose joint with largest distance to nearest software limit
    best_i = None
    best_margin = -1.0
    for i, v in enumerate(q):
        margin = min(v - QMIN[i], QMAX[i] - v)
        if margin > best_margin:
            best_margin = margin
            best_i = i
    if best_i is None or best_margin < 0.15:
        raise RuntimeError(f"no joint with safe margin, q={q}, best_margin={best_margin}")
    # 0.05..0.10 rad toward center
    center = 0.5 * (QMIN[best_i] + QMAX[best_i])
    direction = 1.0 if center >= q[best_i] else -1.0
    delta = direction * min(0.08, best_margin * 0.4)
    if abs(delta) < 0.05:
        delta = direction * 0.05
    return best_i, delta


def main():
    rclpy.init()
    node = TestNode()
    # wait joint states
    t0 = time.time()
    while node.q is None and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.q is None:
        print("FAIL: no /joint_states")
        sys.exit(2)
    start = list(node.q)
    ji, delta = pick_safe_delta(start)
    goal = list(start)
    goal[ji] = start[ji] + delta
    print("start", start)
    print("move joint", ARM[ji], "delta", delta)
    print("goal", goal)
    if not node.ac.wait_for_server(timeout_sec=10.0):
        print("FAIL: action server missing")
        sys.exit(2)
    traj = JointTrajectory()
    traj.joint_names = ARM
    p0 = JointTrajectoryPoint()
    p0.positions = start
    p0.time_from_start = Duration(sec=0, nanosec=0)
    p1 = JointTrajectoryPoint()
    p1.positions = goal
    p1.time_from_start = Duration(sec=4, nanosec=0)
    traj.points = [p0, p1]
    g = FollowJointTrajectory.Goal()
    g.trajectory = traj
    send = node.ac.send_goal_async(g)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        print("FAIL: goal rejected")
        sys.exit(1)
    res_f = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_f)
    result = res_f.result().result
    # final state
    t1 = time.time()
    while time.time() - t1 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    final = list(node.q)
    err = [abs(a - b) for a, b in zip(final, goal)]
    print("result error_code", result.error_code, result.error_string)
    print("final", final)
    print("abs_error", err, "max", max(err))
    ok = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL and max(err) < 0.12
    print("ACTION_TEST", "PASS" if ok else "FAIL")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
