"""Check stationary vision preview with synthetic camera metadata in real ROS."""

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
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Path as RosPath
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray, Marker
    import yaml

    namespace = "vision_probe_" + uuid.uuid4().hex[:10]
    ground_frame, camera_frame = namespace + "/ground", namespace + "/camera"
    rclpy.init()
    node = Node("acceptance_probe", namespace=namespace)
    status, paths, markers = [], [], []
    subs = [node.create_subscription(String, "status", lambda m: status.append(json.loads(m.data)), 1),
            node.create_subscription(RosPath, "reference_ground", paths.append, 1),
            node.create_subscription(MarkerArray, "markers", markers.append, 1)]
    publisher = node.create_publisher(String, "input", 1)
    with tempfile.TemporaryDirectory() as folder:
        root = Path(folder)
        camera = {"image_width": 640, "image_height": 480, "distortion_model": "plumb_bob",
                  "camera_matrix": {"data": [500, 0, 319.5, 0, 500, 239.5, 0, 0, 1]},
                  "distortion_coefficients": {"data": [0, 0, 0, 0, 0]}}
        mount = {"height_m": 0.6, "pitch_down_deg": 30.0, "roll_deg": 0.0, "yaw_deg": 0.0,
                 "camera_x_m": 0.0, "camera_y_m": 0.0, "ground_frame": ground_frame, "camera_frame": camera_frame}
        (root / "camera.yaml").write_text(yaml.safe_dump(camera))
        (root / "mount.json").write_text(json.dumps(mount))
        child = subprocess.Popen([sys.executable, "-c", f"from {args.package}.vision_preview_node import main; main()", "--ros-args",
            "-r", f"__ns:=/{namespace}", "-p", f"camera_calibration:={root / 'camera.yaml'}", "-p",
            f"camera_mount:={root / 'mount.json'}", "-p", f"status_topic:=/{namespace}/input"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        def wait(predicate, seconds=5):
            end = time.monotonic() + seconds
            while not predicate() and time.monotonic() < end:
                if child.poll() is not None:
                    raise RuntimeError("preview node exited")
                rclpy.spin_once(node, timeout_sec=0.02)
            assert predicate(), status[-1:]

        def data(delay_ms=0):
            sec, ns = divmod(node.get_clock().now().nanoseconds-int(delay_ms*1_000_000), 1_000_000_000)
            return {"valid_image_path": True, "reason": "ok",
                    "source_header": {"frame_id": camera_frame, "stamp": {"sec": sec, "nanosec": ns}},
                    "path_coordinate_system": "processed_image_pixels_xy_near_to_far",
                    "path": [[319.5, v] for v in range(450, 199, -1)],
                    "processed_shape_hw": [480, 640], "input_shape_hw": [480, 640]}

        def send(value, expected):
            count = len(status)
            publisher.publish(String(data=json.dumps(value)))
            wait(lambda: len(status) > count and status[-1]["reason"] == expected and status[-1]["result_receipt_age_ms"] < 300)

        def marker(index):
            return next(item for item in markers[-1].markers if item.id == index)

        try:
            wait(lambda: publisher.get_subscription_count() == 1)
            send(data(), "candidate_available")
            candidate = status[-1]["candidate_before_acceleration_limits"]
            assert abs(candidate["linear_x_mps"] - 0.2) < 1e-9 and abs(candidate["angular_z_rps"]) < 1e-9
            assert status[-1]["preview_only"] and not status[-1]["ready_for_control"]
            wait(lambda: paths and len(paths[-1].poses) > 2 and markers and marker(3).action == Marker.ADD)
            assert paths[-1].header.frame_id == ground_frame
            print("PASS: real DDS camera metadata -> metric path -> stationary Pure Pursuit candidate", flush=True)
            wait(lambda: status[-1]["reason"] == "stale_for_control", seconds=1.5)
            wait(lambda: marker(3).action == Marker.DELETE and marker(4).color.g < 0.6)
            assert paths[-1].poses and status[-1]["candidate_before_acceleration_limits"] is None
            for delay_ms in (655, 1330, 1671):
                value = data(delay_ms)
                send(value, "stale_for_control")
                stamp = value["source_header"]["stamp"]
                wait(lambda: paths[-1].header.stamp.sec == stamp["sec"] and paths[-1].header.stamp.nanosec == stamp["nanosec"])
                assert paths[-1].poses and status[-1]["ground_path_points"] > 2
                assert status[-1]["candidate_before_acceleration_limits"] is None
                assert status[-1]["valid_image_path"] and status[-1]["controller_reason"] == "tracking"
            print("PASS: 655-1671ms CPU results stay visible with original stamps; candidate/target expire at controller limit", flush=True)
            value = data()
            value["source_header"]["stamp"]["sec"] -= 4
            send(value, "display_expired")
            wait(lambda: not paths[-1].poses and marker(3).action == Marker.DELETE and marker(4).action == Marker.DELETE)
            value = data()
            value["input_shape_hw"] = [720, 1280]
            send(value, "invalid_detection_or_calibration_geometry")
            assert "resolution" in status[-1]["detail"]
            value = data()
            value["source_header"]["frame_id"] = "other_camera"
            send(value, "source_frame_mismatch")
            value = data()
            value["path"] = [[319.5, v] for v in range(250, 199, -1)]
            send(value, "controller:too_far_from_path")
            assert status[-1]["candidate_before_acceleration_limits"] is None
            send({"valid_image_path": False, "reason": "no_path"}, "image_path_invalid:no_path")
            wait(lambda: not paths[-1].poses and marker(4).action == Marker.DELETE)
            wait(lambda: status[-1]["result_receipt_age_ms"] > 700)
            assert status[-1]["reason"] == "image_path_invalid:no_path"
            send(data(), "candidate_available")
            wait(lambda: status[-1]["reason"] == "display_expired")
            wait(lambda: not paths[-1].poses and marker(3).action == Marker.DELETE and marker(4).action == Marker.DELETE)
            publishers = node.get_publisher_names_and_types_by_node("vision_control_preview", "/" + namespace)
            assert publishers
            types = {kind for _, kinds in publishers for kind in kinds}
            assert not types.intersection({"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped", "nav_msgs/msg/Odometry"}), types
            assert not any("cmd_vel" in topic for topic, _ in publishers), publishers
            print("PASS: invalid/mismatched inputs clear immediately; diagnostics persist; display eventually expires; no Twist/Odometry/cmd_vel", flush=True)
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                output, _ = child.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); output, _ = child.communicate()
            if sys.exc_info()[0] is not None or child.returncode not in (0, -signal.SIGINT):
                print(output[-6000:], file=sys.stderr)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
