"""Webcam/AI + metric geometry + read-only stationary command preview."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("husky_drivable_area"))
    defaults = {"camera_index": "2", "camera_width": "640", "camera_height": "480", "camera_fps": "15.0",
                "device": "cpu", "model_width": "320", "model_height": "192", "cpu_threads": "2",
                "use_webcam": "true", "image_topic": "/camera/image_raw",
                "rviz": "true", "show_image": "true",
                "controller_config": str(share / "config/controller.yaml")}
    inputs = ("camera_index", "camera_width", "camera_height", "camera_fps", "device", "model_width",
              "model_height", "cpu_threads", "use_webcam", "image_topic", "model_path")
    return LaunchDescription([
        DeclareLaunchArgument("model_path", description="Local trained SegFormer directory"),
        DeclareLaunchArgument("camera_calibration", description="Raw-camera pinhole calibration YAML"),
        DeclareLaunchArgument("camera_mount", description="Measured CURRENT stationary camera mount JSON"),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch/drivable_area.launch.py")),
                                 launch_arguments={name: LaunchConfiguration(name) for name in inputs}.items()),
        Node(package="husky_drivable_area", executable="vision_control_preview", namespace="vision_preview", output="screen",
             parameters=[LaunchConfiguration("controller_config"), {
                 name: ParameterValue(LaunchConfiguration(name), value_type=str)
                 for name in ("camera_calibration", "camera_mount")}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/vision_preview.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
        Node(package="image_view", executable="image_view", name="vision_preview_image", namespace="vision_preview", output="screen",
             condition=IfCondition(LaunchConfiguration("show_image")), remappings=[("image", "/drivable_area/overlay")],
             parameters=[{"image_transport": "compressed", "autosize": False, "width": 960, "height": 720}]),
    ])
