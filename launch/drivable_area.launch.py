"""Start the existing model with either a webcam or an external ROS image topic."""

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
    arguments = {
        "model_path": ("", "Required: absolute path to extracted husky-segformer model"),
        "device": ("cpu", "cpu, cuda or auto"),
        "model_width": ("448", "Model input width, multiple of 32"),
        "model_height": ("256", "Model input height, multiple of 32"),
        "config_file": (str(share / "config/config.example.json"), "Detector JSON configuration"),
        "use_webcam": ("true", "False when another ROS node already publishes camera images"),
        "camera_index": ("2", "Webcam /dev/video index"),
        "camera_width": ("640", "Requested webcam width"),
        "camera_height": ("480", "Requested webcam height"),
        "camera_fps": ("15.0", "Requested webcam and publisher FPS"),
        "image_topic": ("/camera/image_raw", "Raw sensor_msgs/Image input topic"),
        "max_frame_age_sec": ("2.0", "Discard old input/results and clear references on camera timeout"),
    }

    def param(name, kind):
        return ParameterValue(LaunchConfiguration(name), value_type=kind)

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in arguments.items()],
        Node(
            package="husky_drivable_area", executable="webcam_node", output="screen",
            condition=IfCondition(LaunchConfiguration("use_webcam")),
            parameters=[{
                "camera_index": param("camera_index", int),
                "width": param("camera_width", int), "height": param("camera_height", int),
                "fps": param("camera_fps", float), "image_topic": param("image_topic", str),
            }],
        ),
        Node(
            package="husky_drivable_area", executable="drivable_area_node", output="screen",
            parameters=[{
                "model_path": param("model_path", str), "device": param("device", str),
                "model_width": param("model_width", int), "model_height": param("model_height", int),
                "config_file": param("config_file", str), "image_topic": param("image_topic", str),
                "max_frame_age_sec": param("max_frame_age_sec", float),
            }],
        ),
    ])
