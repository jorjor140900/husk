"""Inspect measured floor markers through a calibrated webcam; no AI/control."""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String

from .ground_check import GroundCheckOverlay, add_banner
from .ground_geometry import CameraCalibration, CameraMount
from .transport import LatestInput, fill_compressed_image, frame_age_seconds, image_to_bgr


class GroundCheckNode(Node):
    def __init__(self):
        super().__init__("ground_geometry_check")
        defaults = {"camera_calibration": "", "camera_mount": "", "image_topic": "/camera/image_raw",
                    "publish_rate_hz": 5.0, "max_frame_age_sec": 1.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        if not value("camera_calibration") or not value("camera_mount"):
            raise ValueError("camera_calibration YAML and current camera_mount JSON are required")
        self.calibration = CameraCalibration.load(value("camera_calibration"))
        self.mount = CameraMount.load(value("camera_mount"))
        rate, self.max_age = float(value("publish_rate_hz")), float(value("max_frame_age_sec"))
        if not math.isfinite(rate) or not 0 < rate <= 30 or not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError("require finite 0 < publish_rate_hz <= 30 and positive max_frame_age_sec")
        cv2.setNumThreads(1)
        self.grid = GroundCheckOverlay(self.calibration, self.mount)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.overlay_pub = self.create_publisher(CompressedImage, "overlay/compressed", qos)
        self.status_pub = self.create_publisher(String, "status", 1)
        self.latest = LatestInput()
        self.subscription = self.create_subscription(Image, value("image_topic"), self.latest.put, qos)
        self.last_error_at = float("-inf")
        self.last_error = None
        self.timer = self.create_timer(1/rate, self._tick)
        self.get_logger().info("Ground measurement check only: raw camera + calibration, no model, TF or motor commands. "
                               "Restart after changing calibration/mount. C=Y0, L=Y+0.20m, R=Y-0.20m.")

    def _tick(self):
        pending = self.latest.take()
        if pending is None:
            received_at, header, _ = self.latest.state()
            if time.perf_counter()-received_at > self.max_age:
                self._reject("camera_timeout", header)
            return
        header = pending.message.header
        try:
            if header.frame_id != self.mount.camera_frame:
                raise ValueError("source_frame_mismatch")
            source_ns = header.stamp.sec*1_000_000_000+header.stamp.nanosec
            source_age = (self.get_clock().now().nanoseconds-source_ns)/1e9
            if source_ns <= 0 or source_age < -0.05 or frame_age_seconds(pending, self.get_clock().now().nanoseconds) > self.max_age:
                raise ValueError("stale_or_invalid_source_timestamp")
            overlay = self.grid.draw(image_to_bgr(pending.message))
            if frame_age_seconds(pending, self.get_clock().now().nanoseconds) > self.max_age:
                raise ValueError("stale_after_drawing")
            targets = self.grid.metadata()["visible_targets"]
            self._publish(overlay, header, "measurement_overlay" if targets else "no_visible_targets", bool(targets))
        except (ValueError, TypeError, OSError, cv2.error) as exc:
            self._reject(str(exc), header)

    def _reject(self, reason, header):
        if reason == self.last_error and time.perf_counter()-self.last_error_at < 1.0:
            return
        self.last_error_at = time.perf_counter()
        self.last_error = reason
        blank = np.zeros((self.calibration.height, self.calibration.width, 3), dtype=np.uint8)
        overlay = add_banner(blank, ["GROUND CHECK REJECTED - NO MEASUREMENT GRID", reason[:92],
                                    "Check camera input, frame, capture size and calibration."], error=True)
        self._publish(overlay, header or Header(), reason, False)
        self.get_logger().warning("Ground check rejected: " + reason, throttle_duration_sec=2.0)

    def _publish(self, overlay, header, reason, valid):
        # Keep original source header even when rejecting an old image. Output
        # is a diagnostic JPEG; its banner pixels are not raw camera coordinates.
        self.overlay_pub.publish(fill_compressed_image(CompressedImage(), overlay, header))
        if valid:
            self.last_error = None
        metadata = self.grid.metadata()
        metadata.update({"valid_ground_overlay": valid, "reason": reason,
                         "raw_shape_hw": [self.calibration.height, self.calibration.width],
                         "source_header": {"frame_id": header.frame_id,
                                           "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec}}})
        if not valid:
            metadata["visible_targets"] = []
        self.status_pub.publish(String(data=json.dumps(metadata, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GroundCheckNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
