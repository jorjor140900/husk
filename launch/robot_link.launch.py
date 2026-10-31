"""ROS1 telemetry connection. No motion forwarding unless explicitly configured."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"bridge_url": "ws://127.0.0.1:9090", "ros1_prefix": "/husky_vision", "forward_commands": "false",
                "base_frame": "", "expected_output_topic": "", "command_topic": "/husky_control/control/cmd_vel_stamped",
                "expected_camera_width": "0", "expected_camera_height": "0"}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package="husky_drivable_area", executable="robot_bridge", namespace="husky_link", output="screen",
             parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=bool if name == "forward_commands" else int if name.startswith("expected_camera_") else str)
                          for name in defaults}]),
    ])
