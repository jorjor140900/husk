"""Capture a local ChArUco calibration session from the raw ROS image topic."""

from datetime import datetime, timezone
import json
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .charuco import CharucoSession, save_calibration
from .transport import LatestInput, frame_age_seconds, image_to_bgr, fill_compressed_image


class CharucoNode(Node):
    def __init__(self):
        super().__init__("charuco_calibration")
        for key, value in dict(image_topic="/camera/image_raw", squares_x=7, squares_y=5,
                               square_m=.024, marker_m=.017, dictionary="DICT_5X5_100",
                               width=640, height=480, min_views=20,
                               output_dir=str(Path.home()/"husky_vision_ws/camera_calibration")).items():
            self.declare_parameter(key, value)
        value = lambda key: self.get_parameter(key).value
        cv2.setNumThreads(1)
        self.session = CharucoSession(value("squares_x"), value("squares_y"), value("square_m"),
                                      value("marker_m"), value("dictionary"), value("min_views"))
        self.expected = (value("height"), value("width"))
        self.folder = Path(value("output_dir")).expanduser()/datetime.now(timezone.utc).strftime("charuco-%Y%m%dT%H%M%S%fZ")
        self.folder.mkdir(parents=True, exist_ok=False)
        self.latest, self.last_capture, self.saved = LatestInput(), float("-inf"), False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(CompressedImage, "/charuco/overlay/compressed", qos)
        self.subscription = self.create_subscription(Image, value("image_topic"), self.latest.put, qos)
        self.service = self.create_service(Trigger, "/charuco/calibrate", self._calibrate)
        self.timer = self.create_timer(.15, self._tick)
        self.get_logger().info(f"ChArUco automatic capture; output {self.folder}; display /charuco/overlay with compressed transport")

    def _tick(self):
        pending = self.latest.take()
        if pending is None:
            return
        try:
            if frame_age_seconds(pending, self.get_clock().now().nanoseconds) > 1:
                raise ValueError("stale camera frame")
            frame = image_to_bgr(pending.message)
            if frame.shape[:2] != self.expected:
                raise ValueError("require raw 640x480 calibrated capture mode (or set matching width/height)")
            corners, ids = self.session.detect(frame)
            reason = "saved; start a new session to capture again" if self.saved else "hold_or_move_board"
            if not self.saved and time.monotonic()-self.last_capture >= .8:
                reason = self.session.add(frame, corners, ids)
                if reason == "captured":
                    self.last_capture = time.monotonic()
                    index = len(self.session.views)
                    if not cv2.imwrite(str(self.folder/f"view-{index:03}.png"), frame):
                        self.session.views.pop()
                        raise ValueError("could not save raw capture")
                    views = [dict(ids=v["ids"].tolist(), pixels=v["pixels"].tolist()) for v in self.session.views]
                    (self.folder/"observations.json").write_text(json.dumps(dict(board=self.session.config, views=views), indent=2))
            overlay = frame.copy()
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(overlay, corners, ids)
            progress = self.session.progress()
            lines = [f"Views {progress['views']}/{self.session.min_views} | coverage {100*progress['coverage']:.0f}% | tilts {progress['tilted_views']}/3",
                     "READY: call /charuco/calibrate" if progress["ready_to_calibrate"] else "Need: "+"; ".join(progress["pending"]),
                     reason]
            cv2.rectangle(overlay, (0, 0), (frame.shape[1], 65), (20,20,20), -1)
            for i, text in enumerate(lines):
                cv2.putText(overlay, text, (5, 17+20*i), cv2.FONT_HERSHEY_SIMPLEX, .43, (255,255,255), 1)
            self.publisher.publish(fill_compressed_image(CompressedImage(), overlay, pending.message.header, 85))
            self.get_logger().info(" | ".join(lines), throttle_duration_sec=3.)
        except (ValueError, cv2.error, OSError) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.)

    def _calibrate(self, request, response):
        try:
            if self.saved:
                raise ValueError(f"already saved to {self.folder}; start a new session for another calibration")
            mapping, report = self.session.calibrate()
            save_calibration(self.folder, mapping, report)
            self.saved = response.success = True
            response.message = f"Saved {self.folder/'camera.yaml'} | training RMS {report['training_rms_px']:.3f}px | held-out {report['held_out_rms_px']} | warnings {report['warnings']}"
            self.get_logger().info(response.message)
        except (ValueError, cv2.error, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CharucoNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
