#!/usr/bin/env python3
"""Plan joint-space goals with MoveIt. Planning only; no hardware."""
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

GOALS = {
    "goalA": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "goalB": [0.0, 0.8, 0.8, -0.5, 0.0, 0.0],
    "goalC": [0.6, 0.5, 0.9, -0.4, 0.3, 0.2],
}


class Planner(Node):
    def __init__(self):
        super().__init__("x5a_plan_joint_path")
        self.client = ActionClient(self, MoveGroup, "move_action")
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("move_action not available")

    def plan_to(self, name, joints, start=None):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.planner_id = "RRTConnect"
        if start is not None:
            rs = RobotState()
            rs.joint_state.name = ARM
            rs.joint_state.position = start
            req.start_state = rs
        constraints = Constraints()
        for jn, val in zip(ARM, joints):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True  # never execute
        goal.planning_options.replan = False
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if not gh.accepted:
            return name, False, None
        res_f = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_f)
        result = res_f.result().result
        ok = result.error_code.val == 1
        traj = result.planned_trajectory.joint_trajectory
        info = {
            "error_code": result.error_code.val,
            "points": len(traj.points),
            "planning_time": result.planning_time,
            "start": list(start) if start is not None else "current",
            "goal": joints,
            "planner": req.planner_id,
        }
        return name, ok, info


def main():
    rclpy.init()
    node = Planner()
    # start at zeros for first, then chain
    start = [0.0] * 6
    order = ["goalB", "goalC", "goalA"]
    all_ok = True
    for name in order:
        goal = GOALS[name]
        n, ok, info = node.plan_to(name, goal, start=start)
        print("=" * 60)
        print(n, "OK" if ok else "FAIL", info)
        if not ok:
            all_ok = False
        else:
            start = goal
    node.destroy_node()
    rclpy.shutdown()
    print("OVERALL", "PASS" if all_ok else "FAIL")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
