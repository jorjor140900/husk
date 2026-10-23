"""Future robot adapter in preview mode; requires real odometry, TF and a gate."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("husky_drivable_area"))
    arguments = {
        "path_topic": ("/drivable_area/reference_ground", "Metric nav_msgs/Path, never reference_pixels"),
        "odom_topic": ("/odom", "Measured nav_msgs/Odometry from the robot"),
        "permission_topic": ("/husky_safety/motion_permitted", "Fresh Bool heartbeat from independent collision/stop gate"),
        "odom_frame": ("odom", "Fixed odometry frame"),
        "base_frame": ("base_link", "Odometry child frame"),
        "config_file": (str(share / "config/controller.yaml"), "Controller tuning YAML"),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in arguments.items()],
        Node(package="husky_drivable_area", executable="path_controller", namespace="husky_control", output="screen",
             parameters=[LaunchConfiguration("config_file"),
                         {name: ParameterValue(LaunchConfiguration(name), value_type=str)
                          for name in arguments if name != "config_file"}, {"start_enabled": False}]),
    ])
