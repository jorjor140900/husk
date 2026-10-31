"""Synthetic LaserScan + downstream swept-footprint stop filter. No robot IO."""

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
    frames = {"odom_frame": "husky_lidar_demo/odom", "base_frame": "husky_lidar_demo/base_link"}
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="obstacle", description="obstacle, straight or curve"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="husky_drivable_area", executable="control_simulator", namespace="husky_lidar_demo", output="screen",
             parameters=[frames, {"simulate_lidar": True,
                          "scenario": ParameterValue(LaunchConfiguration("scenario"), value_type=str)}]),
        Node(package="husky_drivable_area", executable="path_controller", namespace="husky_lidar_demo", output="screen",
             parameters=[str(share / "config/controller.yaml"), frames, {"start_enabled": True}],
             remappings=[("control/cmd_vel", "control/requested_cmd_vel")]),
        Node(package="husky_drivable_area", executable="laser_safety", namespace="husky_lidar_demo", output="screen",
             parameters=[str(share / "config/laser_safety.yaml"), frames]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/lidar_demo.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
