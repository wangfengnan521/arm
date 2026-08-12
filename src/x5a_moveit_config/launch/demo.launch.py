#!/usr/bin/env python3
"""X5A MoveIt pure-planning demo.

NOT VALIDATED FOR HARDWARE.
Does not start ARX X5Controller, CAN, or any real hardware interface.
Trajectory execution uses mock ros2_control only for RViz visualization.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # allow_trajectory_execution True only for mock/fake controllers in this demo.
    # Real ARX hardware is never launched from this file.
    moveit_config = (
        MoveItConfigsBuilder("X5A", package_name="x5a_moveit_config")
        .robot_description(file_path="config/X5A.urdf.xacro")
        .robot_description_semantic(file_path="config/X5A.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")

    run_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "capabilities": "move_group/MoveGroupCartesianPathService "
                "move_group/MoveGroupExecuteTrajectoryAction "
                "move_group/MoveGroupKinematicsService "
                "move_group/MoveGroupMoveAction "
                "move_group/MoveGroupPlanService "
                "move_group/MoveGroupQueryPlannersService "
                "move_group/MoveGroupStateValidationService "
                "move_group/MoveGroupGetPlanningSceneService",
                "publish_robot_description_semantic": True,
                # Planning allowed; execution only against mock controllers.
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Virtual joint TF world -> base_link (fixed)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
        parameters=[moveit_config.robot_description],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    # Note: for planning-only without fake execution, joint_state_publisher is enough.
    # We intentionally do NOT spawn controller_manager / ARX hardware.

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_gui",
                default_value="false",
                description="If true, use GUI joint publisher instead of default zeros.",
            ),
            static_tf,
            rsp,
            jsp,
            jsp_gui,
            run_move_group,
            rviz,
        ]
    )
