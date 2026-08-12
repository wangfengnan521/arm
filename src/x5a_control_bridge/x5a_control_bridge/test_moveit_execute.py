#!/usr/bin/env python3
"""Plan+Execute a nearby joint goal via MoveIt move_action (real robot)."""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState,
)
from sensor_msgs.msg import JointState

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
QMIN = [-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]
QMAX = [2.618, 3.50, 3.20, 1.55, 1.57, 2.0]


class N(Node):
    def __init__(self):
        super().__init__("x5a_moveit_exec_test")
        self.q = None
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.ac = ActionClient(self, MoveGroup, "move_action")

    def cb(self, msg):
        try:
            self.q = [float(msg.position[msg.name.index(n)]) for n in ARM]
        except Exception:
            pass


def pick_goal(q):
    best_i, best_m = 0, -1.0
    for i, v in enumerate(q):
        m = min(v - QMIN[i], QMAX[i] - v)
        if m > best_m:
            best_m, best_i = m, i
    center = 0.5 * (QMIN[best_i] + QMAX[best_i])
    direction = 1.0 if center >= q[best_i] else -1.0
    delta = direction * min(0.12, max(0.06, best_m * 0.25))
    g = list(q)
    g[best_i] = q[best_i] + delta
    return g, best_i, delta


def main():
    rclpy.init()
    node = N()
    t0 = time.time()
    while node.q is None and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.q is None:
        print("FAIL no joint_states")
        sys.exit(2)
    start = list(node.q)
    goal, ji, delta = pick_goal(start)
    print("start", start)
    print("goal", goal, "joint", ARM[ji], "delta", delta)
    assert node.ac.wait_for_server(30.0)
    g = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = "arm"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.1
    req.max_acceleration_scaling_factor = 0.1
    rs = RobotState()
    rs.joint_state.name = ARM
    rs.joint_state.position = start
    req.start_state = rs
    cons = Constraints()
    for n, v in zip(ARM, goal):
        jc = JointConstraint()
        jc.joint_name = n
        jc.position = float(v)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        cons.joint_constraints.append(jc)
    req.goal_constraints.append(cons)
    g.request = req
    g.planning_options = PlanningOptions()
    g.planning_options.plan_only = False  # EXECUTE
    g.planning_options.replan = False
    send = node.ac.send_goal_async(g)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15)
    gh = send.result()
    if not gh or not gh.accepted:
        print("FAIL goal not accepted")
        sys.exit(1)
    resf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, resf, timeout_sec=120)
    result = resf.result().result
    # sample final
    time.sleep(0.5)
    t1 = time.time()
    while time.time() - t1 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    final = list(node.q)
    err = [abs(a - b) for a, b in zip(final, goal)]
    print("moveit error_code", result.error_code.val)
    print("final", final)
    print("abs_error", err, "max", max(err))
    ok = result.error_code.val == 1 and max(err) < 0.1
    print("MOVEIT_EXECUTE", "PASS" if ok else "FAIL")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
