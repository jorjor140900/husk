"""Validate a measured robot profile; incomplete values stay visible, not guessed."""
import json
import math
import os
from pathlib import Path
from urllib.parse import urlparse

from husky_link_core.protocol import frame_id
from .ground_geometry import CameraCalibration, CameraMount


def expanded_path(value):
    if not isinstance(value, str) or not value:
        raise ValueError("required file path is missing")
    return Path(os.path.expandvars(value)).expanduser().resolve()


def load_robot_profile(filename, *, allow_supervised_trial=False):
    data = json.loads(Path(filename).expanduser().read_text())
    from husky_link_core.supervised_trial import trial_mode
    trial = trial_mode(data, allow_supervised_trial)
    if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data.get("schema_version") != 1:
        raise ValueError("unsupported robot profile")
    problems = []
    for section in ("bridge", "robot", "camera", "footprint", "field_measurements"):
        if not isinstance(data.get(section), dict):
            raise ValueError("missing profile section: "+section)
    robot, camera, bridge = data["robot"], data["camera"], data["bridge"]
    for key in ("odom_frame", "base_frame"):
        try:
            robot[key] = frame_id(robot[key])
        except (ValueError, KeyError, TypeError):
            problems.append("robot."+key)
    if robot.get("odom_frame") == robot.get("base_frame"):
        problems.append("robot odom/base frames must differ")
    for key in ("odom_topic", "scan_topic", "external_permission_topic"):
        value = robot.get(key)
        if not isinstance(value, str) or not value.startswith("/") or any(c.isspace() for c in value):
            problems.append("robot."+key)
    if robot.get("external_permission_topic") in ("/husky_link/motion_permitted", "/husky_control/motion_permitted"):
        problems.append("robot.external_permission_topic must come from an independent field gate")
    for key in ("ros1_permission_topic", "joy_topic"):
        value = robot.get(key)
        if value is not None and (not isinstance(value, str) or not value.startswith("/") or any(c.isspace() for c in value)):
            problems.append("robot."+key)
    if robot.get("joy_topic") and (type(robot.get("joy_deadman_button")) is not int or robot["joy_deadman_button"] < 0):
        problems.append("robot.joy_deadman_button")
    if robot.get("joy_topic") and robot.get("ros1_permission_topic"):
        problems.append("choose either joy_topic or ros1_permission_topic")
    z = robot.get("base_to_ground_z_m")
    if type(z) not in (float, int) or not math.isfinite(z) or abs(z) > 3:
        problems.append("robot.base_to_ground_z_m (measure base origin relative to ground)")
    for key in ("model_path", "calibration_path", "mount_path"):
        try:
            camera[key] = str(expanded_path(camera[key]))
            if not Path(camera[key]).exists():
                raise ValueError("file missing")
        except (ValueError, KeyError):
            problems.append("camera."+key)
    if type(bridge.get("forward_commands")) is not bool:
        problems.append("bridge.forward_commands must be bool")
    address = urlparse(bridge.get("url") if isinstance(bridge.get("url"), str) else "")
    if address.scheme not in ("ws", "wss") or not address.hostname or address.username or address.password:
        problems.append("bridge.url")
    for key in ("index", "width", "height", "model_width", "model_height", "cpu_threads"):
        value = camera.get(key)
        if type(value) is not int or value < (0 if key == "index" else 1):
            problems.append("camera."+key)
    if any(type(camera.get(key)) is int and camera[key] % 32 for key in ("model_width", "model_height")):
        problems.append("camera model dimensions must be multiples of 32")
    if type(camera.get("fps")) not in (int, float) or not math.isfinite(camera["fps"]) or not 0 < camera["fps"] <= 120:
        problems.append("camera.fps")
    if camera.get("device") not in ("cpu", "cuda", "auto"):
        problems.append("camera.device")
    if camera.get("source") not in ("local_usb", "ros1_compressed", "external_ros2"):
        problems.append("camera.source")
    if type(camera.get("publish_camera_tf")) is not bool:
        problems.append("camera.publish_camera_tf")
    if camera.get("source") == "ros1_compressed":
        topic = camera.get("ros1_compressed_topic")
        if not isinstance(topic, str) or not topic.startswith("/"):
            problems.append("camera.ros1_compressed_topic")
    for key in ("front_m", "rear_m", "half_width_m", "margin_m"):
        value = data["footprint"].get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            problems.append("footprint."+key)
    if "lateral_margin_m" in data["footprint"]:
        value = data["footprint"]["lateral_margin_m"]
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            problems.append("footprint.lateral_margin_m")
    if problems:
        raise ValueError("Complete measured robot profile first: "+", ".join(problems))
    if "navigation" in data:
        from .navigation import inflation_radius
        inflation_radius(data)
    calibration = CameraCalibration.load(camera["calibration_path"])
    mount = CameraMount.load(camera["mount_path"])
    if [camera["width"], camera["height"]] != [calibration.width, calibration.height]:
        raise ValueError("robot capture resolution differs from calibration")
    if mount.ground_frame in (robot["odom_frame"], robot["base_frame"]) or mount.camera_frame in (robot["odom_frame"], robot["base_frame"]):
        raise ValueError("use distinct measured robot ground/camera frames; never an identity odom attachment")
    if bridge["forward_commands"]:
        if not isinstance(bridge.get("expected_output_topic"), str) or not bridge["expected_output_topic"].startswith("/"):
            raise ValueError("set the verified ROS1 velocity-mux input topic")
        checks = {"geometry_checked", "mux_and_base_timeout_checked", "independent_permission_gate_checked"}
        if not trial and (set(data["field_measurements"]) != checks or any(data["field_measurements"][key] is not True for key in checks)):
            raise ValueError("field geometry, mux/base timeout and independent permission gate measurements are still pending")
        if robot["external_permission_topic"] == "/husky_link/external_permission":
            source = robot.get("ros1_permission_topic")
            joy, button = robot.get("joy_topic"), robot.get("joy_deadman_button")
            has_bool = isinstance(source, str) and source.startswith("/")
            has_joy = isinstance(joy, str) and joy.startswith("/") and type(button) is int and button >= 0
            if has_bool == has_joy:
                raise ValueError("choose one verified ROS1 Bool gate or timestamped joystick/deadman source")
        if not (robot.get("ros1_permission_topic") or robot.get("joy_topic")):
            raise ValueError("driver link requires a robot-local permission/deadman source as well as the ROS2 gate")
    return data, mount
