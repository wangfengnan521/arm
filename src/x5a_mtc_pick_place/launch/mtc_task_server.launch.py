#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    plan_only = LaunchConfiguration("plan_only")
    task_mode = LaunchConfiguration("task_mode")
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
    vision_config = os.path.join(
        get_package_share_directory("x5a_vision"), "config", "vision.yaml"
    )
    mtc_config = os.path.join(
        get_package_share_directory("x5a_mtc_pick_place"), "config", "mtc_pick_place.yaml"
    )
    benchmark_targets = os.path.join(
        get_package_share_directory("x5a_mtc_pick_place"),
        "config",
        "benchmark_targets.txt",
    )
    server = Node(
        package="x5a_mtc_pick_place",
        executable="x5a_mtc_task_server",
        name="x5a_mtc_task_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            vision_config,
            mtc_config,
            {"mtc.plan_only": ParameterValue(plan_only, value_type=bool)},
            {"mtc.task_mode": ParameterValue(task_mode, value_type=str)},
            {"mtc.benchmark_targets_file": benchmark_targets},
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "plan_only", default_value="false", description="plan and freeze, skip execution"
            ),
            DeclareLaunchArgument(
                "task_mode",
                default_value="full",
                description="full | pick | pre_grasp | benchmark | reachability",
            ),
            server,
        ]
    )
