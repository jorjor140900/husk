"""Show known floor-distance markers on a raw webcam image, without inference."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"camera_index": "2", "camera_width": "640", "camera_height": "480", "camera_fps": "15.0",
                "use_webcam": "true", "image_topic": "/camera/image_raw", "show_image": "true", "namespace": "ground_check"}

    def param(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    return LaunchDescription([
        DeclareLaunchArgument("camera_calibration", description="Calibrated RAW camera YAML"),
        DeclareLaunchArgument("camera_mount", description="Current measured camera mount JSON"),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package="husky_drivable_area", executable="webcam_node", output="screen",
             condition=IfCondition(LaunchConfiguration("use_webcam")), parameters=[{
                 "camera_index": param("camera_index", int), "width": param("camera_width", int),
                 "height": param("camera_height", int), "fps": param("camera_fps", float),
                 "image_topic": param("image_topic", str)}]),
        Node(package="husky_drivable_area", executable="ground_geometry_check", namespace=LaunchConfiguration("namespace"), output="screen",
             parameters=[{name: param(name, str) for name in ("camera_calibration", "camera_mount", "image_topic")}]),
        Node(package="image_view", executable="image_view", name="ground_check_image", namespace=LaunchConfiguration("namespace"), output="screen",
             condition=IfCondition(LaunchConfiguration("show_image")), remappings=[("image", "overlay")],
             parameters=[{"image_transport": "compressed", "autosize": False, "width": 960, "height": 720}]),
    ])
