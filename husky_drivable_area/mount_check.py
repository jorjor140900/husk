"""Read-only check of a mount against independently measured floor points."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .ground_geometry import CameraCalibration, CameraMount, ground_to_image
from .mount_fit import serve_points
from .transport import image_to_bgr


def raw_measurement_frame(message, calibration, mount, ros_ns):
    """Validate a fresh original frame, never an overlay or resized screenshot."""
    stamp = message.header.stamp
    source_ns = stamp.sec*1_000_000_000+stamp.nanosec
    age = (ros_ns-source_ns)/1e9
    if (stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or source_ns <= 0 or
            not -.05 <= age <= .5):
        raise ValueError("raw frame source timestamp is stale or invalid")
    if message.header.frame_id != mount.camera_frame:
        raise ValueError("raw frame differs from the measured camera frame")
    image = image_to_bgr(message)
    if image.shape[:2] != (calibration.height, calibration.width):
        raise ValueError("raw capture resolution differs from calibration")
    return image, dict(frame_id=message.header.frame_id,
                      stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec), source_age_ms=age*1000)


def capture_frame(topic, calibration, mount):
    """One read-only ROS subscription; stop ROS before opening the point UI."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    rclpy.init(args=[])
    node = Node("mount_measurement_capture")
    answer, errors = [], []
    def received(message):
        try:
            answer[:] = [raw_measurement_frame(message, calibration, mount, node.get_clock().now().nanoseconds)]
        except (ValueError, TypeError, AttributeError) as exc:
            errors[:] = [str(exc)]
    subscription = node.create_subscription(Image, topic, received,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        end = time.monotonic()+8
        while not answer and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        if not answer:
            raise ValueError("no fresh calibrated raw image: "+(errors[-1] if errors else "camera timeout"))
        return answer[0]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def check_floor_points(calibration, mount, pixels, expected):
    pixels, expected = np.asarray(pixels, dtype=float), np.asarray(expected, dtype=float)
    if (pixels.ndim != 2 or pixels.shape[1] != 2 or pixels.shape != expected.shape or
            not 1 <= len(pixels) <= 1000 or not np.isfinite(pixels).all() or not np.isfinite(expected).all()):
        raise ValueError("need matching finite pixel XY and measured ground XY points")
    if (pixels < 0).any() or (pixels[:, 0] > calibration.width-1).any() or (pixels[:, 1] > calibration.height-1).any():
        raise ValueError("point outside calibrated raw image")
    normalized = cv2.undistortPoints(pixels.reshape(-1, 1, 2), calibration.matrix, calibration.distortion).reshape(-1, 2)
    rays = np.column_stack([normalized, np.ones(len(pixels))]) @ mount.rotation.T
    if not np.isfinite(rays).all() or (rays[:, 2] >= -1e-6).any():
        raise ValueError("selected point does not intersect the floor in front of the camera")
    actual = (mount.origin-mount.height_m/rays[:, 2, None]*rays)[:, :2]
    projected, visible = ground_to_image(expected, calibration, mount)
    return {"expected_xy_m": expected.tolist(), "estimated_xy_m": actual.tolist(),
            "error_xy_cm": ((actual-expected)*100).tolist(),
            "distance_error_cm": (np.linalg.norm(actual-expected, axis=1)*100).tolist(),
            "observed_pixels_xy": pixels.tolist(), "expected_pixels_xy": projected.tolist(),
            "expected_points_visible": visible.tolist(),
            "mount_changed": False, "verified_geometry": False, "ready_for_control": False}


def corridor_points(near_x, far_x, width):
    """Three measured crossbars centered on ground-frame Y=0, left then right."""
    values = np.asarray([near_x, far_x, width], dtype=float)
    if not np.isfinite(values).all() or not 0 < near_x < far_x or width <= 0:
        raise ValueError("require finite 0 < near-x < far-x and positive corridor-width")
    return np.array([[x, y] for x in (near_x, (near_x+far_x)/2, far_x)
                     for y in (width/2, -width/2)])


def check_corridor_points(calibration, mount, pixels, expected):
    report = check_floor_points(calibration, mount, pixels, expected)
    if np.asarray(expected).shape != (6, 2):
        raise ValueError("corridor check requires three measured left/right pairs")
    expected = np.asarray(expected, float).reshape(3, 2, 2)
    actual = np.asarray(report["estimated_xy_m"]).reshape(3, 2, 2)
    widths = []
    for measured, projected in zip(expected, actual):
        width = float(np.linalg.norm(measured[0]-measured[1]))
        projected_width = float(np.linalg.norm(projected[0]-projected[1]))
        widths.append(dict(forward_x_m=float(measured[:, 0].mean()),
            measured_width_m=width, estimated_width_m=projected_width,
            estimated_lateral_width_m=float(projected[0, 1]-projected[1, 1]),
            width_error_cm=(projected_width-width)*100,
            center_error_xy_cm=((projected.mean(axis=0)-measured.mean(axis=0))*100).tolist()))
    report.update(width_checks=widths, ai_mask_used=False)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("camera-calibration", "camera-mount"):
        parser.add_argument("--"+name, type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="original raw image file")
    source.add_argument("--capture-topic", help="capture one fresh raw ROS image, e.g. /camera/image_raw")
    parser.add_argument("--near-x", type=float, default=1.0)
    parser.add_argument("--far-x", type=float, default=1.5)
    parser.add_argument("--left-y", type=float, default=.2)
    parser.add_argument("--corridor-width", type=float,
                        help="six-point width check in meters; marks centered on robot ground Y=0, independent of AI")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mount-check"))
    args = parser.parse_args(argv)
    calibration, mount = CameraCalibration.load(args.camera_calibration), CameraMount.load(args.camera_mount)
    source_header = None
    if args.capture_topic:
        try:
            image, source_header = capture_frame(args.capture_topic, calibration, mount)
        except (ValueError, ImportError) as exc:
            parser.error(str(exc))
    else:
        image = cv2.imread(str(args.image.expanduser()))
    if image is None or image.shape[:2] != (calibration.height, calibration.width):
        parser.error("use the original raw image at the calibrated resolution")
    if args.corridor_width is not None:
        try:
            expected = corridor_points(args.near_x, args.far_x, args.corridor_width)
        except ValueError as exc:
            parser.error(str(exc))
        labels = ["N-L", "N-R", "M-L", "M-R", "F-L", "F-R"]
        prompt = "คลิกจุดบนพื้น: ใกล้ซ้าย → ใกล้ขวา → กลางซ้าย → กลางขวา → ไกลซ้าย → ไกลขวา (ซ้ายของหุ่น)"
    else:
        expected = np.array([[args.near_x, 0.], [args.far_x, 0.], [args.far_x, args.left_y]])
        if not np.isfinite(expected).all() or not args.far_x > args.near_x or args.left_y <= 0:
            parser.error("require finite far-x > near-x and left-y > 0")
        labels = ["E", "F", "G"]
        prompt = "คลิกฐานสัมผัสพื้น: E ใกล้กลาง → F ไกลกลาง → G ไกลซ้าย"

    def submit(pixels):
        checker = check_corridor_points if args.corridor_width is not None else check_floor_points
        report = checker(calibration, mount, pixels, expected)
        report.update({"labels": labels, "mount": asdict(mount),
                       "source_header": source_header,
                       "note": "Independent point check only; no mount fitting or robot commands."})
        folder = args.output_dir.expanduser() / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        folder.mkdir(parents=True, exist_ok=False)
        (folder / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
        cv2.imwrite(str(folder / "raw.png"), image)
        return {"saved_to": str(folder.absolute()), **report}
    return serve_points(image, labels, prompt, submit)
