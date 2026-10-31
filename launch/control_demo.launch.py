"""Isolated simulation: synthetic route, kinematic plant, controller and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("husky_drivable_area"))
    frames = {"odom_frame": "husky_demo/odom", "base_frame": "husky_demo/base_link"}
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="curve", description="curve, straight or obstacle"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="husky_drivable_area", executable="control_simulator", namespace="husky_demo", output="screen",
             parameters=[frames, {"scenario": ParameterValue(LaunchConfiguration("scenario"), value_type=str)}]),
        Node(package="husky_drivable_area", executable="path_controller", namespace="husky_demo", output="screen",
             parameters=[str(share / "config/controller.yaml"), frames, {"start_enabled": True}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/control_demo.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
