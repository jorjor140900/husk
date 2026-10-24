"""Run metric projection separately from inference; show a top view in RViz."""

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
    return LaunchDescription([
        DeclareLaunchArgument("camera_calibration", description="Absolute path to calibrated raw-camera YAML"),
        DeclareLaunchArgument("camera_mount", description="Absolute path to measured mount JSON"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("max_frame_age_sec", default_value="2.0"),
        DeclareLaunchArgument("max_distance_m", default_value="5.0"),
        Node(package="husky_drivable_area", executable="ground_path_node", output="screen", parameters=[{
            "camera_calibration": ParameterValue(LaunchConfiguration("camera_calibration"), value_type=str),
            "camera_mount": ParameterValue(LaunchConfiguration("camera_mount"), value_type=str),
            "max_frame_age_sec": ParameterValue(LaunchConfiguration("max_frame_age_sec"), value_type=float),
            "max_distance_m": ParameterValue(LaunchConfiguration("max_distance_m"), value_type=float),
        }]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/ground_path.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
