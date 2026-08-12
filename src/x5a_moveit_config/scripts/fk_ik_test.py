#!/usr/bin/env python3
"""FK->IK validation for X5A arm group. Pure planning, no hardware."""
import sys
import time
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
TESTS = {
    "PoseA_zeros": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "PoseB_ready": [0.0, 0.8, 0.8, -0.5, 0.0, 0.0],
    "PoseC_side": [0.6, 0.5, 0.9, -0.4, 0.3, 0.2],
}


class FkIk(Node):
    def __init__(self):
        super().__init__("x5a_fk_ik_test")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        for c in (self.fk, self.ik):
            if not c.wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f"service not available: {c.srv_name}")

    def run(self):
        results = []
        for name, joints in TESTS.items():
            js = JointState()
            js.name = ARM
            js.position = joints
            rs = RobotState()
            rs.joint_state = js
            # FK
            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = "base_link"
            fk_req.fk_link_names = ["tool0"]
            fk_req.robot_state = rs
            fk_res = self.fk.call(fk_req)
            if fk_res.error_code.val != 1:
                results.append((name, False, f"FK failed code={fk_res.error_code.val}", None, None))
                continue
            pose = fk_res.pose_stamped[0]
            # IK from zero seed
            ik_req = GetPositionIK.Request()
            ik_req.ik_request = PositionIKRequest()
            ik_req.ik_request.group_name = "arm"
            ik_req.ik_request.robot_state = RobotState()
            ik_req.ik_request.robot_state.joint_state.name = ARM
            ik_req.ik_request.robot_state.joint_state.position = [0.0] * 6
            ik_req.ik_request.pose_stamped = pose
            ik_req.ik_request.timeout.sec = 1
            ik_req.ik_request.avoid_collisions = False
            ik_res = self.ik.call(ik_req)
            ok = ik_res.error_code.val == 1
            sol = None
            if ok:
                names = list(ik_res.solution.joint_state.name)
                pos = list(ik_res.solution.joint_state.position)
                sol = {n: pos[names.index(n)] for n in ARM if n in names}
            results.append((name, ok, pose, joints, sol))
        return results


def main():
    rclpy.init()
    node = FkIk()
    results = node.run()
    all_ok = True
    for name, ok, pose, joints, sol in results:
        print("=" * 60)
        print(name)
        print("input_joints", joints)
        if not ok and joints is None:
            print("FAIL", pose)
            all_ok = False
            continue
        if hasattr(pose, "pose"):
            p = pose.pose.position
            o = pose.pose.orientation
            print(f"FK_tool0 xyz=({p.x:.4f},{p.y:.4f},{p.z:.4f}) quat=({o.x:.4f},{o.y:.4f},{o.z:.4f},{o.w:.4f})")
        print("IK_success", ok)
        print("IK_solution", sol)
        if not ok:
            all_ok = False
            print("error_detail", pose if not hasattr(pose, "pose") else "see error_code")
    node.destroy_node()
    rclpy.shutdown()
    print("OVERALL", "PASS" if all_ok else "FAIL")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
