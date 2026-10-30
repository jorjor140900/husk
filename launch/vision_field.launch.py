"""Known laptop camera/CUDA setup plus image-goal navigation; starts disabled."""
import json
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    from husky_drivable_area.ground_geometry import CameraMount
    from husky_drivable_area.robot_profile import expanded_path
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory("husky_drivable_area"))
    profile_file = expanded_path(value("profile"))
    profile = json.loads(profile_file.read_text())
    camera = profile["camera"]
    camera_only = value("camera_only").lower() == "true"
    if camera_only and value("allow_motion").lower() != "false":
        raise ValueError("camera_only is a display preview and requires allow_motion=false")
    if not camera_only and not value("map"):
        raise ValueError("navigation requires map; use camera_only=true for a robot-free preview")
    if camera.get("source") != "local_usb":
        raise ValueError("vision_field.launch expects the measured local USB camera")
    device = expanded_path(value("camera_device"))
    match = re.fullmatch(r"video([0-9]+)", device.name)
    if not device.exists() or match is None:
        raise ValueError("camera_device must resolve to the connected /dev/videoN; plug the camera or override camera_device")
    mount = CameraMount.load(expanded_path(camera["mount_path"]))
    common = dict(profile=str(profile_file), map=value("map"), steering_mode="vision",
                  supervised_trial=value("supervised_trial"),
                  allow_motion=value("allow_motion"), rviz=value("rviz"), image_topic="/camera/image_raw")
    perception = dict(use_webcam="false", image_topic="/camera/image_raw", device="cuda",
        model_path=str(expanded_path(camera["model_path"])), model_width="256", model_height="160", cpu_threads="2",
        camera_calibration=str(expanded_path(camera["calibration_path"])),
        camera_mount=str(expanded_path(camera["mount_path"])),
        config_file=str(share/"config/outdoor-supported-green-preview.json"),
        show_nav2_path="false", async_reference_preview="true" if camera_only else "false",
        reference_preview_fps="10.0", max_frame_age_sec="0.5")
    def include(name, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch"/name)),
                                        launch_arguments=arguments.items())
    nodes = [SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        Node(package="husky_drivable_area", executable="webcam_node", output="screen", parameters=[{
            "camera_index": int(match.group(1)), "width": camera["width"], "height": camera["height"],
            "fps": camera["fps"], "frame_id": mount.camera_frame, "image_topic": "/camera/image_raw"}]),
        include("drivable_area.launch.py", perception)]
    if not camera_only:
        nodes.append(include("navigation.launch.py", common))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", description="Measured no-joystick profile"),
        DeclareLaunchArgument("map", default_value="", description="Saved site map; required unless camera_only=true"),
        DeclareLaunchArgument("camera_only", default_value="false", description="Image-only yellow/red preview without robot, map or controller"),
        DeclareLaunchArgument("camera_device", default_value="/dev/v4l/by-id/usb-Generic_Web_Camera_200901010001-video-index0"),
        DeclareLaunchArgument("allow_motion", default_value="false"),
        DeclareLaunchArgument("supervised_trial", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_nodes)])
