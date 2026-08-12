#!/usr/bin/env python3
"""Pose-driven pick and place for ARX X5A using MoveIt 2 planning.

Uses:
- MoveGroup action for planning only
- compute_cartesian_path service for vertical approach/lift/descend/retreat
- standard MoveIt ExecuteTrajectory and GripperCommand actions
- /joint_states from the single official ARX hardware adapter
- PlanningScene topics for table/object attach/detach
"""
from __future__ import annotations

import math
import sys
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from control_msgs.action import GripperCommand
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PlanningScene,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from shape_msgs.msg import SolidPrimitive
from x5a_handeye.x5a_fk import fk_base_tool0


ARM_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def pose_xyz_quat(
    x: float, y: float, z: float, quat: Quaternion
) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation = quat
    return p


def pose_to_xyz(pose: Pose) -> Tuple[float, float, float]:
    return (pose.position.x, pose.position.y, pose.position.z)


class PickPlaceNode(Node):
    def __init__(self) -> None:
        super().__init__("x5a_pick_place")
        self.cb = ReentrantCallbackGroup()
        self._declare_params()
        self._load_params()

        self.q: Optional[List[float]] = None
        self.dq: Optional[List[float]] = None
        self.gripper_position = 0.0
        self.create_subscription(
            JointState, self.joint_state_topic, self._joint_state_cb, 10,
            callback_group=self.cb
        )
        self.create_subscription(
            PoseStamped, self.vision_pose_topic, self._vision_pose_cb, 10, callback_group=self.cb
        )
        self.create_subscription(
            Bool, self.vision_stable_topic, self._vision_stable_cb, 10, callback_group=self.cb
        )
        self.vision_pose: Optional[PoseStamped] = None
        self.vision_pose_arrival = 0.0
        self.vision_stable = False

        self.move_ac = ActionClient(self, MoveGroup, "move_action", callback_group=self.cb)
        self.execute_ac = ActionClient(
            self, ExecuteTrajectory, self.execute_action, callback_group=self.cb
        )
        self.gripper_ac = ActionClient(
            self, GripperCommand, self.gripper_action, callback_group=self.cb
        )
        self.cart_cli = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self.cb
        )
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb)

        scene_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", scene_qos)
        self.grasp_quat = rpy_to_quat(*self.grasp_rpy)
        self.transit_quat = rpy_to_quat(*self.transit_rpy)
        self.last_joints = None

        self.get_logger().info(
            f"pick_place ready plan_only={self.plan_only} vision={self.vision_enabled} "
            f"execution={self.execute_action} gripper={self.gripper_action} "
            f"state={self.joint_state_topic}"
        )

    def _declare_params(self) -> None:
        self.declare_parameter("plan_only", False)
        self.declare_parameter("planning.group", "arm")
        self.declare_parameter("planning.base_frame", "base_link")
        self.declare_parameter("planning.tcp_frame", "tool0")
        self.declare_parameter("planning.transit_velocity_scaling", 0.45)
        self.declare_parameter("planning.transit_acceleration_scaling", 0.18)
        self.declare_parameter("planning.precision_velocity_scaling", 0.15)
        self.declare_parameter("planning.precision_acceleration_scaling", 0.08)
        self.declare_parameter("planning.lift_retreat_velocity_scaling", 0.20)
        self.declare_parameter("planning.lift_retreat_acceleration_scaling", 0.10)
        self.declare_parameter("planning.planning_time", 5.0)
        self.declare_parameter("planning.max_attempts", 8)
        self.declare_parameter("execution.action_name", "/execute_trajectory")
        self.declare_parameter("execution.timeout", 120.0)
        self.declare_parameter("state.joint_state_topic", "/joint_states")
        self.declare_parameter("vision.enabled", False)
        self.declare_parameter("vision.pose_topic", "/x5a_vision/object_pose")
        self.declare_parameter("vision.stable_topic", "/x5a_vision/detection_stable")
        self.declare_parameter("vision.max_pose_age", 0.5)
        self.declare_parameter("vision.wait_timeout", 10.0)
        self.declare_parameter("grasp_orientation_rpy", [3.14159265, 0.0, 0.0])
        self.declare_parameter("transit_orientation_rpy", [0.0, 1.15, 0.0])
        for ns, keys in {
            "object": ["x", "y", "z", "size_x", "size_y", "size_z"],
            "place": ["x", "y", "z"],
            "table": ["x", "y", "z", "size_x", "size_y", "size_z"],
            "motion": [
                "pre_grasp_height",
                "lift_height",
                "pre_place_height",
                "retreat_height",
                "cartesian_step",
                "jump_threshold",
                "min_cartesian_fraction",
            ],
            "grasp": ["x_offset", "y_offset", "z_offset"],
        }.items():
            for k in keys:
                default = 0.0
                if k.endswith("height"):
                    default = 0.10
                if k == "cartesian_step":
                    default = 0.01
                if k == "min_cartesian_fraction":
                    default = 0.95
                if k.startswith("size"):
                    default = 0.03 if ns == "object" else 0.5
                self.declare_parameter(f"{ns}.{k}", default)
        self.declare_parameter(
            "gripper.action_name", "/x5a_gripper_controller/gripper_cmd"
        )
        self.declare_parameter("gripper.open_position_m", 0.044)
        self.declare_parameter("gripper.close_position_m", 0.0)
        self.declare_parameter("gripper.max_effort", 0.0)
        self.declare_parameter("gripper.timeout", 3.0)
        self.declare_parameter(
            "ready_joints", [-0.2, 0.3, 0.6, -0.3, 0.0, 0.0]
        )

    def _load_params(self) -> None:
        g = self.get_parameter
        self.plan_only = bool(g("plan_only").value)
        self.group = str(g("planning.group").value)
        self.base_frame = str(g("planning.base_frame").value)
        self.tcp_frame = str(g("planning.tcp_frame").value)
        self.transit_vel = float(g("planning.transit_velocity_scaling").value)
        self.transit_acc = float(g("planning.transit_acceleration_scaling").value)
        self.precision_vel = float(g("planning.precision_velocity_scaling").value)
        self.precision_acc = float(g("planning.precision_acceleration_scaling").value)
        self.lift_retreat_vel = float(
            g("planning.lift_retreat_velocity_scaling").value
        )
        self.lift_retreat_acc = float(
            g("planning.lift_retreat_acceleration_scaling").value
        )
        self.planning_time = float(g("planning.planning_time").value)
        self.max_attempts = int(g("planning.max_attempts").value)
        self.execute_action = str(g("execution.action_name").value)
        self.execution_timeout = float(g("execution.timeout").value)
        self.joint_state_topic = str(g("state.joint_state_topic").value)
        self.vision_enabled = bool(g("vision.enabled").value)
        self.vision_pose_topic = str(g("vision.pose_topic").value)
        self.vision_stable_topic = str(g("vision.stable_topic").value)
        self.vision_max_age = float(g("vision.max_pose_age").value)
        self.vision_wait_timeout = float(g("vision.wait_timeout").value)
        self.grasp_rpy = [float(x) for x in g("grasp_orientation_rpy").value]
        self.transit_rpy = [float(x) for x in g("transit_orientation_rpy").value]
        self.obj_x = float(g("object.x").value)
        self.obj_y = float(g("object.y").value)
        self.obj_z = float(g("object.z").value)
        self.obj_sx = float(g("object.size_x").value)
        self.obj_sy = float(g("object.size_y").value)
        self.obj_sz = float(g("object.size_z").value)
        self.place_x = float(g("place.x").value)
        self.place_y = float(g("place.y").value)
        self.place_z = float(g("place.z").value)
        self.table_x = float(g("table.x").value)
        self.table_y = float(g("table.y").value)
        self.table_z = float(g("table.z").value)
        self.table_sx = float(g("table.size_x").value)
        self.table_sy = float(g("table.size_y").value)
        self.table_sz = float(g("table.size_z").value)
        self.pre_grasp_h = float(g("motion.pre_grasp_height").value)
        self.lift_h = float(g("motion.lift_height").value)
        self.pre_place_h = float(g("motion.pre_place_height").value)
        self.retreat_h = float(g("motion.retreat_height").value)
        self.cart_step = float(g("motion.cartesian_step").value)
        self.jump_threshold = float(g("motion.jump_threshold").value)
        self.min_frac = float(g("motion.min_cartesian_fraction").value)
        self.gx = float(g("grasp.x_offset").value)
        self.gy = float(g("grasp.y_offset").value)
        self.gz = float(g("grasp.z_offset").value)
        self.gripper_action = str(g("gripper.action_name").value)
        self.gripper_open_position = float(g("gripper.open_position_m").value)
        self.gripper_close_position = float(g("gripper.close_position_m").value)
        self.gripper_max_effort = float(g("gripper.max_effort").value)
        self.gripper_timeout = float(g("gripper.timeout").value)
        self.ready_joints = [float(x) for x in g("ready_joints").value]

    def _joint_state_cb(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if any(name not in index for name in ARM_JOINTS):
            return
        self.q = [float(msg.position[index[name]]) for name in ARM_JOINTS]
        self.dq = [
            float(msg.velocity[index[name]])
            if index[name] < len(msg.velocity) else 0.0
            for name in ARM_JOINTS
        ]
        if "joint7" in index and index["joint7"] < len(msg.position):
            self.gripper_position = float(msg.position[index["joint7"]])

    def _vision_pose_cb(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != self.base_frame:
            return
        self.vision_pose = msg
        self.vision_pose_arrival = time.monotonic()

    def _vision_stable_cb(self, msg: Bool) -> None:
        self.vision_stable = bool(msg.data)

    def wait_ready(self, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if (
                self.q is not None
                and self.move_ac.wait_for_server(timeout_sec=0.1)
                and self.cart_cli.wait_for_service(timeout_sec=0.1)
                and (
                    self.plan_only
                    or (
                        self.execute_ac.wait_for_server(timeout_sec=0.1)
                        and self.gripper_ac.wait_for_server(timeout_sec=0.1)
                    )
                )
            ):
                return True
            time.sleep(0.05)
        return False

    @staticmethod
    def wait_future(future, timeout: float) -> bool:
        """Wait while the node's dedicated executor thread services callbacks."""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def current_tcp(self) -> Optional[Tuple[float, float, float]]:
        if self.q is None:
            return None
        t = fk_base_tool0(self.q)[:3, 3]
        return (float(t[0]), float(t[1]), float(t[2]))

    def log_stage(self, stage: str, **kwargs) -> None:
        parts = [f"[{stage}]"]
        for k, v in kwargs.items():
            if isinstance(v, float):
                parts.append(f"{k}={v:.4f}")
            else:
                parts.append(f"{k}={v}")
        self.get_logger().info(" ".join(parts))

    def execute_moveit_trajectory(self, trajectory: RobotTrajectory, stage: str) -> bool:
        """Execute a planned path through MoveIt's standard execution pipeline."""
        if not trajectory.joint_trajectory.points:
            self.log_stage(stage, execution="FAIL", reason="empty trajectory")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.execute_ac.send_goal_async(goal)
        if not self.wait_future(future, 10.0):
            self.log_stage(stage, execution="FAIL", reason="goal timeout")
            return False
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.log_stage(stage, execution="FAIL", reason="goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, self.execution_timeout):
            self.log_stage(stage, execution="FAIL", reason="result timeout")
            return False
        result = result_future.result().result
        ok = result.error_code.val == 1
        if ok:
            self.last_joints = list(self.q) if self.q is not None else self.last_joints
        self.log_stage(
            stage,
            execution="PASS" if ok else f"FAIL({result.error_code.val})",
            interface="MoveIt ExecuteTrajectory",
        )
        return ok

    def call_gripper(self, open_gripper: bool) -> bool:
        stage = "OPEN_GRIPPER" if open_gripper else "CLOSE_GRIPPER"
        if self.plan_only:
            self.log_stage(stage, result="SKIP_PLAN_ONLY")
            return True
        if not self.gripper_ac.wait_for_server(timeout_sec=2.0):
            self.log_stage(stage, result="FAIL", reason="GripperCommand unavailable")
            return False
        target = self.gripper_open_position if open_gripper else self.gripper_close_position
        goal = GripperCommand.Goal()
        goal.command.position = float(target)
        goal.command.max_effort = float(self.gripper_max_effort)
        future = self.gripper_ac.send_goal_async(goal)
        if not self.wait_future(future, 2.0):
            self.log_stage(stage, result="FAIL", reason="goal timeout")
            return False
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.log_stage(stage, result="FAIL", reason="goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, self.gripper_timeout):
            self.log_stage(stage, result="FAIL", reason="result timeout")
            return False
        result = result_future.result().result
        ok = bool(result.reached_goal)
        self.log_stage(
            stage,
            result="PASS" if ok else "FAIL",
            target_m=target,
            feedback_m=float(result.position),
            interface="control_msgs/GripperCommand",
        )
        return ok

    def publish_box(
        self,
        name: str,
        xyz: Sequence[float],
        size: Sequence[float],
        frame: str = "base_link",
        operation=CollisionObject.ADD,
    ) -> None:
        co = CollisionObject()
        co.id = name
        co.header.frame_id = frame
        co.header.stamp = self.get_clock().now().to_msg()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = operation
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self.scene_pub.publish(scene)

    def setup_scene(self) -> None:
        self.publish_box(
            "table",
            [self.table_x, self.table_y, self.table_z],
            [self.table_sx, self.table_sy, self.table_sz],
        )
        self.publish_box(
            "object",
            [self.scene_obj_x, self.scene_obj_y, self.scene_obj_z],
            [self.obj_sx, self.obj_sy, self.obj_sz],
        )
        time.sleep(0.3)
        self.log_stage("SCENE", table="PASS", object="PASS")

    def attach_object(self) -> None:
        aco = AttachedCollisionObject()
        aco.link_name = self.tcp_frame
        aco.object.id = "object"
        aco.object.operation = CollisionObject.ADD
        # object pose relative to tool0 approx identity; use current world pose as attached
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [self.obj_sx, self.obj_sy, self.obj_sz]
        pose = Pose()
        pose.orientation.w = 1.0
        aco.object.primitives = [prim]
        aco.object.primitive_poses = [pose]
        aco.touch_links = ["link6", "link7", "link8", "tool0"]
        # remove world object then attach
        self.publish_box(
            "object",
            [self.scene_obj_x, self.scene_obj_y, self.scene_obj_z],
            [self.obj_sx, self.obj_sy, self.obj_sz],
            operation=CollisionObject.REMOVE,
        )
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        self.scene_pub.publish(scene)
        time.sleep(0.2)
        self.log_stage("ATTACH_OBJECT", result="PASS")

    def detach_object(self, place_xyz: Sequence[float]) -> None:
        # detach and place as world object
        aco = AttachedCollisionObject()
        aco.object.id = "object"
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        self.scene_pub.publish(scene)
        time.sleep(0.1)
        self.publish_box(
            "object",
            place_xyz,
            [self.obj_sx, self.obj_sy, self.obj_sz],
            operation=CollisionObject.ADD,
        )
        time.sleep(0.2)
        self.log_stage("DETACH_OBJECT", result="PASS", place=list(place_xyz))

    def move_joints(
        self,
        joints: Sequence[float],
        stage: str,
        velocity_scaling: float,
        acceleration_scaling: float,
    ) -> bool:
        assert self.q is not None
        g = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = self.max_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = velocity_scaling
        req.max_acceleration_scaling_factor = acceleration_scaling
        rs = RobotState()
        rs.joint_state.name = list(ARM_JOINTS)
        rs.joint_state.position = self.planning_start_joints()
        req.start_state = rs
        cons = Constraints()
        for n, v in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        req.goal_constraints.append(cons)
        g.request = req
        g.planning_options = PlanningOptions()
        # Ask MoveGroup for a plan; execution is sent separately through
        # MoveIt's ExecuteTrajectory action so the controller manager remains
        # the single owner of hardware execution.
        g.planning_options.plan_only = True
        start_tcp = self.current_tcp()
        fut = self.move_ac.send_goal_async(g)
        if not self.wait_future(fut, 20.0):
            self.log_stage(stage, planning="FAIL", reason="goal timeout")
            return False
        gh = fut.result()
        if not gh or not gh.accepted:
            self.log_stage(stage, planning="FAIL", reason="goal not accepted")
            return False
        rf = gh.get_result_async()
        if not self.wait_future(rf, 120.0):
            self.log_stage(stage, planning="FAIL", reason="result timeout")
            return False
        res = rf.result().result
        planning_ok = res.error_code.val == 1
        ok = planning_ok
        if planning_ok:
            self._update_last_joints_from_result(res)
            if self.plan_only:
                self.last_joints = [float(v) for v in joints]
            else:
                ok = self.execute_moveit_trajectory(res.planned_trajectory, stage + "_EXEC")
        actual = self.current_tcp()
        self.log_stage(
            stage,
            planning="PASS" if planning_ok else f"FAIL({res.error_code.val})",
            execution="SKIP" if self.plan_only else ("PASS" if ok else "FAIL"),
            start_tcp=start_tcp,
            actual_tcp=actual,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
        )
        return ok

    def move_pose(
        self,
        pose: Pose,
        stage: str,
        velocity_scaling: float,
        acceleration_scaling: float,
    ) -> bool:
        assert self.q is not None
        # IK pre-solve for free-space pose goals, then joint-space plan (more reliable on this arm).
        joints = self.ik_joints_for_pose(pose)
        if joints is not None:
            return self.move_joints(
                joints, stage, velocity_scaling, acceleration_scaling
            )
        g = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = self.max_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = velocity_scaling
        req.max_acceleration_scaling_factor = acceleration_scaling
        rs = RobotState()
        rs.joint_state.name = list(ARM_JOINTS)
        rs.joint_state.position = self.planning_start_joints()
        req.start_state = rs
        # position constraint around target with small box
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.tcp_frame
        pc.weight = 1.0
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.01]
        pc.constraint_region.primitives = [prim]
        pc.constraint_region.primitive_poses = [pose]
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.tcp_frame
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0
        cons = Constraints()
        cons.position_constraints.append(pc)
        cons.orientation_constraints.append(oc)
        req.goal_constraints.append(cons)
        g.request = req
        g.planning_options = PlanningOptions()
        g.planning_options.plan_only = True
        target = pose_to_xyz(pose)
        start_tcp = self.current_tcp()
        fut = self.move_ac.send_goal_async(g)
        if not self.wait_future(fut, 20.0):
            self.log_stage(stage, planning="FAIL", reason="goal timeout", target_tcp=target)
            return False
        gh = fut.result()
        if not gh or not gh.accepted:
            self.log_stage(stage, planning="FAIL", target_tcp=target, reason="not accepted")
            return False
        rf = gh.get_result_async()
        if not self.wait_future(rf, 120.0):
            self.log_stage(stage, planning="FAIL", reason="result timeout", target_tcp=target)
            return False
        res = rf.result().result
        planning_ok = res.error_code.val == 1
        ok = planning_ok
        if planning_ok:
            self._update_last_joints_from_result(res)
            if not self.plan_only:
                ok = self.execute_moveit_trajectory(res.planned_trajectory, stage + "_EXEC")
        actual = self.current_tcp()
        self.log_stage(
            stage,
            planning="PASS" if planning_ok else f"FAIL({res.error_code.val})",
            execution="SKIP" if self.plan_only else ("PASS" if ok else "FAIL"),
            target_tcp=target,
            start_tcp=start_tcp,
            actual_tcp=actual,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
        )
        return ok

    def cartesian_to(self, pose: Pose, stage: str) -> Tuple[bool, float]:
        assert self.q is not None
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.group
        req.link_name = self.tcp_frame
        req.max_step = self.cart_step
        req.jump_threshold = self.jump_threshold
        req.avoid_collisions = True
        start_q = self.planning_start_joints()
        req.start_state.joint_state.name = list(ARM_JOINTS)
        req.start_state.joint_state.position = start_q
        wp = Pose()
        wp.position = pose.position
        wp.orientation = pose.orientation
        req.waypoints = [wp]
        start_tcp = self.current_tcp()
        target = pose_to_xyz(pose)
        fut = self.cart_cli.call_async(req)
        if not self.wait_future(fut, 30.0):
            self.log_stage(stage, planning="FAIL", fraction=0.0, reason="service timeout")
            return False, 0.0
        res = fut.result()
        if res is None:
            self.log_stage(stage, planning="FAIL", fraction=0.0, reason="no response")
            return False, 0.0
        frac = float(res.fraction)
        if frac < self.min_frac:
            self.log_stage(
                stage,
                planning="FAIL",
                fraction=frac,
                target_tcp=target,
                start_tcp=start_tcp,
            )
            return False, frac
        self._update_last_joints_from_traj(res.solution)
        if self.plan_only:
            self.log_stage(
                stage,
                planning="PASS",
                execution="SKIP",
                fraction=frac,
                target_tcp=target,
                start_tcp=start_tcp,
            )
            return True, frac
        ok = self.execute_moveit_trajectory(res.solution, stage + "_EXEC")
        actual = self.current_tcp()
        self.log_stage(
            stage,
            planning="PASS",
            execution="PASS" if ok else "FAIL",
            fraction=frac,
            target_tcp=target,
            start_tcp=start_tcp,
            actual_tcp=actual,
        )
        return ok, frac


    def planning_start_joints(self) -> List[float]:
        if self.last_joints is not None:
            return list(self.last_joints)
        if self.q is not None:
            return list(self.q)
        raise RuntimeError("no joint state available")

    def _update_last_joints_from_result(self, res) -> None:
        try:
            traj = res.planned_trajectory.joint_trajectory
            if traj.points:
                names = list(traj.joint_names)
                pos = list(traj.points[-1].positions)
                self.last_joints = [float(pos[names.index(n)]) for n in ARM_JOINTS]
                return
        except Exception:
            pass
        if self.q is not None:
            self.last_joints = list(self.q)

    def _update_last_joints_from_traj(self, robot_traj: RobotTrajectory) -> None:
        try:
            traj = robot_traj.joint_trajectory
            if traj.points:
                names = list(traj.joint_names)
                pos = list(traj.points[-1].positions)
                self.last_joints = [float(pos[names.index(n)]) for n in ARM_JOINTS]
        except Exception:
            if self.q is not None:
                self.last_joints = list(self.q)


    def ik_joints_for_pose(self, pose: Pose) -> Optional[List[float]]:
        if not self.ik_cli.wait_for_service(timeout_sec=2.0):
            return None
        seed = list(self.last_joints) if self.last_joints is not None else list(self.q or [0.0]*6)
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.robot_state.joint_state.name = list(ARM_JOINTS)
        req.ik_request.robot_state.joint_state.position = seed
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 1
        # Prefer collision-aware IK; if it fails (table/object padding), retry without collisions.
        for avoid in (True, False):
            req.ik_request.avoid_collisions = avoid
            fut = self.ik_cli.call_async(req)
            if not self.wait_future(fut, 5.0):
                continue
            res = fut.result()
            if res and res.error_code.val == 1:
                break
        else:
            return None
        names = list(res.solution.joint_state.name)
        pos = list(res.solution.joint_state.position)
        try:
            return [float(pos[names.index(n)]) for n in ARM_JOINTS]
        except Exception:
            return None

    def vertical_move(
        self,
        pose: Pose,
        stage: str,
        fallback_velocity_scaling: float,
        fallback_acceleration_scaling: float,
    ) -> Tuple[bool, float]:
        """Prefer cartesian; fallback to IK+joint planning for vertical stages."""
        ok, frac = self.cartesian_to(pose, stage + "_CART")
        if ok:
            return True, frac
        self.get_logger().warn(f"{stage}: cartesian fraction={frac:.3f}, fallback to IK joint plan")
        joints = self.ik_joints_for_pose(pose)
        if joints is None:
            self.log_stage(stage, planning="FAIL", reason="IK failed", target_tcp=pose_to_xyz(pose))
            return False, frac
        ok2 = self.move_joints(
            joints,
            stage + "_IK",
            fallback_velocity_scaling,
            fallback_acceleration_scaling,
        )
        if ok2:
            return True, 1.0
        return False, frac

    def make_pose(self, x: float, y: float, z: float) -> Pose:
        return pose_xyz_quat(x, y, z, self.grasp_quat)

    def make_transit_pose(self, x: float, y: float, z: float) -> Pose:
        return pose_xyz_quat(x, y, z, self.transit_quat)

    def wait_for_frozen_vision(self) -> Optional[Tuple[float, float, float]]:
        """Wait for a fresh stable base_link pose, then freeze it for this cycle."""
        deadline = time.monotonic() + self.vision_wait_timeout
        self.log_stage("WAIT_FOR_VISION", timeout=self.vision_wait_timeout)
        while time.monotonic() < deadline:
            pose = self.vision_pose
            age = time.monotonic() - self.vision_pose_arrival
            if pose is not None and self.vision_stable and age <= self.vision_max_age:
                xyz = pose_to_xyz(pose.pose)
                if all(math.isfinite(v) for v in xyz):
                    self.log_stage("STABLE_DETECTION", age=age, xyz=xyz)
                    self.log_stage("FREEZE_OBJECT_POSE", xyz=xyz)
                    return xyz
            time.sleep(0.05)
        self.log_stage(
            "WAIT_FOR_VISION", result="FAIL", stable=self.vision_stable,
            pose_age=time.monotonic() - self.vision_pose_arrival,
        )
        return None

    def run(self) -> int:
        if not self.wait_ready():
            self.get_logger().error("dependencies not ready")
            return 2

        if self.vision_enabled:
            frozen = self.wait_for_frozen_vision()
            if frozen is None:
                return 1
            top_x, top_y, top_z = frozen
        else:
            top_x, top_y, top_z = self.obj_x, self.obj_y, self.obj_z + self.obj_sz * 0.5

        # RGB-D reports the visible top surface. PlanningScene needs the cube center,
        # while the TCP grasp target uses the independently calibrated grasp offsets.
        self.scene_obj_x = top_x
        self.scene_obj_y = top_y
        self.scene_obj_z = top_z - self.obj_sz * 0.5
        grasp_x = top_x + self.gx
        grasp_y = top_y + self.gy
        grasp_z = top_z + self.gz
        place_x, place_y, place_z = self.place_x, self.place_y, self.place_z
        self.setup_scene()

        pre_grasp = self.make_transit_pose(
            grasp_x, grasp_y, grasp_z + self.pre_grasp_h
        )
        grasp = self.make_pose(grasp_x, grasp_y, grasp_z)
        pre_place = self.make_transit_pose(
            place_x, place_y, place_z + self.pre_place_h
        )
        place = self.make_pose(place_x, place_y, place_z)
        self.log_stage(
            "POSES", visual_top=(top_x, top_y, top_z),
            collision_center=(self.scene_obj_x, self.scene_obj_y, self.scene_obj_z),
            pre_grasp=pose_to_xyz(pre_grasp), grasp=pose_to_xyz(grasp), place=pose_to_xyz(place),
        )

        if not self.call_gripper(True):
            return 1
        if not self.move_pose(
            pre_grasp,
            "MOVE_PRE_GRASP",
            self.transit_vel,
            self.transit_acc,
        ):
            return 1
        # The visual object was used for scene construction. Remove it immediately
        # before contact so the gripper is allowed to enter the grasp volume.
        self.publish_box(
            "object", [self.scene_obj_x, self.scene_obj_y, self.scene_obj_z],
            [self.obj_sx, self.obj_sy, self.obj_sz], operation=CollisionObject.REMOVE,
        )
        time.sleep(0.15)
        ok, frac_app = self.vertical_move(
            grasp, "APPROACH", self.precision_vel, self.precision_acc
        )
        if not ok:
            return 1
        if not self.call_gripper(False):
            return 1
        self.attach_object()

        # Construct lift from the actual TCP after grasp. In plan-only mode the
        # planned grasp pose is the current logical TCP.
        lift_start = pose_to_xyz(grasp) if self.plan_only else self.current_tcp()
        if lift_start is None:
            return 1
        lift = self.make_transit_pose(
            lift_start[0], lift_start[1], lift_start[2] + self.lift_h
        )
        ok, frac_lift = self.vertical_move(
            lift, "LIFT", self.lift_retreat_vel, self.lift_retreat_acc
        )
        if not ok:
            return 1
        if not self.move_pose(
            pre_place,
            "MOVE_PRE_PLACE",
            self.transit_vel,
            self.transit_acc,
        ):
            return 1
        ok, frac_desc = self.vertical_move(
            place, "DESCEND", self.precision_vel, self.precision_acc
        )
        if not ok:
            return 1
        if not self.call_gripper(True):
            return 1
        table_top = self.table_z + 0.5 * self.table_sz
        self.detach_object([place_x, place_y, table_top + 0.5 * self.obj_sz])
        retreat_start = pose_to_xyz(place) if self.plan_only else self.current_tcp()
        if retreat_start is None:
            return 1
        retreat = self.make_transit_pose(
            retreat_start[0], retreat_start[1], retreat_start[2] + self.retreat_h
        )
        ok, frac_ret = self.vertical_move(
            retreat, "RETREAT", self.lift_retreat_vel, self.lift_retreat_acc
        )
        if not ok:
            return 1
        if not self.move_joints(
            self.ready_joints,
            "RETURN_HOME",
            self.transit_vel,
            self.transit_acc,
        ):
            self.log_stage("RETURN_HOME", result="WARN_CONTINUE")

        self.log_stage(
            "SUMMARY",
            approach_fraction=frac_app,
            lift_fraction=frac_lift,
            descend_fraction=frac_desc,
            retreat_fraction=frac_ret,
            plan_only=self.plan_only,
        )
        if self.plan_only and self.vision_enabled:
            self.get_logger().info("VISION PICK AND PLACE PLAN: PASS")
        elif self.plan_only:
            self.get_logger().info("FIXED-POSE PICK AND PLACE PLAN: PASS")
        elif self.vision_enabled:
            self.get_logger().info("VISION PICK AND PLACE: PASS")
        else:
            self.get_logger().info("FIXED-POSE PICK AND PLACE: PASS")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def spin():
        executor.spin()

    th = threading.Thread(target=spin, daemon=True)
    th.start()
    code = 1
    try:
        code = node.run()
    except Exception as exc:
        node.get_logger().error(f"pick_place exception: {exc}")
        code = 1
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(code)


if __name__ == "__main__":
    main()
