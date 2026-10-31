"""Legacy local-path experiment; use navigation.launch.py for map-frame goals.

Starts disarmed. Independent field permission is required; no fake odometry,
no synthetic scan and no fallback to an identity odom transform are provided.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    from husky_drivable_area.robot_profile import load_robot_profile
    profile, mount = load_robot_profile(LaunchConfiguration("profile").perform(context))
    share = Path(get_package_share_directory("husky_drivable_area"))
    robot, camera, bridge = profile["robot"], profile["camera"], profile["bridge"]
    frames = {"odom_frame": robot["odom_frame"], "base_frame": robot["base_frame"]}
    image_topic = LaunchConfiguration("image_topic").perform(context) or ("/husky_link/image_raw" if camera["source"] == "ros1_compressed" else "/camera/image_raw")
    return [
        Node(package="husky_drivable_area", executable="robot_bridge", namespace="husky_link", output="screen", parameters=[{
            "bridge_url": bridge["url"], "forward_commands": bridge["forward_commands"], "base_frame": robot["base_frame"],
            "expected_output_topic": bridge["expected_output_topic"] or "", "expected_camera_width": camera["width"],
            "expected_camera_height": camera["height"]}]),
        Node(package="husky_drivable_area", executable="webcam_node", output="screen", parameters=[{
            "camera_index": camera["index"], "width": camera["width"], "height": camera["height"], "fps": camera["fps"],
            "frame_id": mount.camera_frame}], condition=IfCondition(LaunchConfiguration("use_webcam") if camera["source"] == "local_usb" else "false")),
        Node(package="husky_drivable_area", executable="drivable_area_node", output="screen", parameters=[{
            "model_path": camera["model_path"], "device": camera["device"], "model_width": camera["model_width"],
            "model_height": camera["model_height"], "cpu_threads": camera["cpu_threads"],
            "config_file": str(share / "config/config.example.json"),
            "camera_calibration": camera["calibration_path"], "camera_mount": camera["mount_path"],
            "image_topic": image_topic}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="measured_base_to_ground", output="screen",
             arguments=["--x", "0", "--y", "0", "--z", str(robot["base_to_ground_z_m"]),
                        "--roll", "0", "--pitch", "0", "--yaw", "0",
                        "--frame-id", robot["base_frame"], "--child-frame-id", mount.ground_frame]),
        Node(package="husky_drivable_area", executable="ground_path_node", output="screen", parameters=[{
            "camera_calibration": camera["calibration_path"], "camera_mount": camera["mount_path"],
            "publish_camera_tf": camera["publish_camera_tf"]}]),
        Node(package="husky_drivable_area", executable="field_permission", namespace="husky_control", output="screen",
             parameters=[{"external_topic": robot["external_permission_topic"]}]),
        Node(package="husky_drivable_area", executable="path_controller", namespace="husky_control", output="screen",
             parameters=[str(share / "config/controller.yaml"), frames, {"start_enabled": False,
                 "cruise_speed_mps": 0.10, "max_angular_speed_rps": 0.30,
                 "path_topic": "/drivable_area/reference_ground", "odom_topic": "/husky_link/odom",
                 "permission_topic": "/husky_control/motion_permitted"}],
             remappings=[("control/cmd_vel", "control/requested_cmd_vel")]),
        Node(package="husky_drivable_area", executable="laser_safety", namespace="husky_control", output="screen",
             parameters=[str(share / "config/laser_safety.yaml"), frames, profile["footprint"],
                         {"max_linear_mps": 0.10, "max_angular_rps": 0.30}],
             remappings=[("odom", "/husky_link/odom"), ("scan", "/husky_link/scan")]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("profile", description="Completed measured robot.json"),
                              DeclareLaunchArgument("use_webcam", default_value="true"),
                              DeclareLaunchArgument("image_topic", default_value=""), OpaqueFunction(function=_nodes)])
