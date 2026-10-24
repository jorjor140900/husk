"""Exercise the floor-marker launch with synthetic raw images in real ROS.

Set QT_QPA_PLATFORM=offscreen when testing without a desktop. No trained model,
robot inputs, odometry, motion commands, or TF broadcasters are used.
"""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", default="husky_drivable_area", choices=("husky_drivable_area", "husky_vision"))
    args = parser.parse_args()
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header, String
    import yaml
    from husky_drivable_area.transport import fill_image, image_to_bgr

    ns = "ground_probe_" + uuid.uuid4().hex[:10]
    rclpy.init()
    node = Node("acceptance_probe", namespace=ns)
    statuses, images = [], []
    subs = [node.create_subscription(String, "status", lambda m: statuses.append(json.loads(m.data)), 1),
            node.create_subscription(Image, "output", images.append,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))]
    publisher = node.create_publisher(Image, "input", 1)
    state = {"publish": True, "width": 640, "frame": "camera_optical_frame", "delay_ns": 0}

    def image():
        if state["publish"]:
            header = Header(frame_id=state["frame"])
            header.stamp.sec, header.stamp.nanosec = divmod(node.get_clock().now().nanoseconds-state["delay_ns"], 1_000_000_000)
            frame = np.full((480, state["width"], 3), 110, dtype=np.uint8)
            publisher.publish(fill_image(Image(), frame, header))

    timer = node.create_timer(0.1, image)
    with tempfile.TemporaryDirectory() as folder:
        root = Path(folder)
        camera = {"image_width": 640, "image_height": 480, "distortion_model": "plumb_bob",
                  "camera_matrix": {"data": [756.8, 0, 316.6, 0, 755.0, 230.0, 0, 0, 1]},
                  "distortion_coefficients": {"data": [0, 0, 0, 0, 0]}}
        mount = {"height_m": 0.775, "pitch_down_deg": 70.0, "roll_deg": 0.0, "yaw_deg": 0.0,
                 "camera_x_m": 0.0, "camera_y_m": 0.0, "ground_frame": "vision_ground", "camera_frame": "camera_optical_frame"}
        (root / "camera.yaml").write_text(yaml.safe_dump(camera))
        (root / "mount.json").write_text(json.dumps(mount))
        child = subprocess.Popen(["ros2", "launch", args.package, "ground_check.launch.py", "use_webcam:=false", f"namespace:={ns}",
            f"image_topic:=/{ns}/input", f"camera_calibration:={root / 'camera.yaml'}", f"camera_mount:={root / 'mount.json'}"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        def wait(predicate, seconds=10):
            end = time.monotonic()+seconds
            while not predicate() and time.monotonic() < end:
                if child.poll() is not None:
                    raise RuntimeError("ground check launch exited")
                rclpy.spin_once(node, timeout_sec=0.02)
            assert predicate(), statuses[-1:]

        def rejected(expected, *, future=False):
            count = len(statuses)
            def matches():
                if len(statuses) <= count or expected not in statuses[-1]["reason"]:
                    return False
                stamp = statuses[-1]["source_header"]["stamp"]
                return not future or stamp["sec"]*1_000_000_000+stamp["nanosec"] > node.get_clock().now().nanoseconds+50_000_000
            wait(matches)
            assert not statuses[-1]["valid_ground_overlay"] and not statuses[-1]["visible_targets"]
            wait(lambda: images and np.mean(image_to_bgr(images[-1])[160:]) < 1)

        try:
            wait(lambda: statuses and statuses[-1]["reason"] == "measurement_overlay" and len(images) >= 3)
            status = statuses[-1]
            assert status["check_only"] and not status["ready_for_control"] and not status["physical_geometry_verified"]
            assert [t["id"] for t in status["visible_targets"] if t["relative_xy_m"][1] == 0] == ["C010", "C025", "C050"]
            decoded = image_to_bgr(images[-1])
            assert decoded.shape == (596, 640, 3) and images[-1].header.frame_id == "camera_optical_frame"
            center = next(t for t in status["visible_targets"] if t["id"] == "C025")
            u, v = np.rint(center["raw_pixel_uv"]).astype(int)
            pixel = decoded[v+116, u]
            assert pixel[1] > 170 and pixel[2] > 170 and pixel[0] < 100, pixel
            print("PASS: calibrated raw Image -> floor-distance markers -> JPEG -> official image_view decoded image", flush=True)
            state["frame"] = "wrong_camera"
            rejected("source_frame_mismatch")
            state["frame"] = "camera_optical_frame"
            state["width"] = 800
            rejected("resolution")
            state["width"] = 640
            state["delay_ns"] = 2_000_000_000
            rejected("stale_or_invalid_source_timestamp")
            state["delay_ns"] = -2_000_000_000
            rejected("stale_or_invalid_source_timestamp", future=True)
            state["delay_ns"] = 0
            wait(lambda: statuses[-1]["valid_ground_overlay"])
            state["publish"] = False
            rejected("camera_timeout")
            publishers = node.get_publisher_names_and_types_by_node("ground_geometry_check", "/"+ns)
            assert publishers
            assert not any("cmd_vel" in topic or topic in ("/tf", "/tf_static") for topic, _ in publishers), publishers
            forbidden = {"geometry_msgs/msg/Twist", "nav_msgs/msg/Path", "nav_msgs/msg/Odometry"}
            assert not any(forbidden.intersection(kinds) for _, kinds in publishers), publishers
            print("PASS: frame/resolution/stale/future/timeout reject and remove grid; fresh input recovers; no TF/Path/motor publishers", flush=True)
        finally:
            child.send_signal(signal.SIGINT)
            try:
                output, _ = child.communicate(timeout=8)
            except subprocess.TimeoutExpired:
                child.kill(); output, _ = child.communicate()
            if sys.exc_info()[0] is not None or child.returncode not in (0, -signal.SIGINT):
                print(output[-6000:], file=sys.stderr)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
