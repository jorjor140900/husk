"""Display real vision references and hypothetical commands; no actuator publisher."""

from dataclasses import asdict
import json
import math
import textwrap
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .control import ControllerConfig
from .ground_geometry import CameraCalibration, CameraMount
from .vision_preview import VisionPreview, age_preview, preview_detection


class VisionPreviewNode(Node):
    def __init__(self):
        super().__init__("vision_control_preview")
        defaults = {**asdict(ControllerConfig()), "camera_calibration": "", "camera_mount": "",
                    "status_topic": "/drivable_area/status", "path_timeout_sec": 0.5,
                    "display_timeout_sec": 3.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        if not value("camera_calibration") or not value("camera_mount"):
            raise ValueError("measured camera_calibration and camera_mount files are required")
        self.calibration = CameraCalibration.load(value("camera_calibration"))
        self.mount = CameraMount.load(value("camera_mount"))
        self.config = ControllerConfig(**{name: value(name) for name in ControllerConfig.__dataclass_fields__})
        self.timeout = float(value("path_timeout_sec"))
        self.display_timeout = float(value("display_timeout_sec"))
        if (not math.isfinite(self.timeout) or self.timeout <= 0 or
                not math.isfinite(self.display_timeout) or self.display_timeout < self.timeout):
            raise ValueError("display_timeout_sec must be finite and >= the positive path_timeout_sec")
        self.path_pub = self.create_publisher(Path, "reference_ground", 1)
        self.status_pub = self.create_publisher(String, "status", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "markers", 1)
        # Measured stationary CAMERA rig only. No odom/base_link transform or
        # pose feedback is fabricated to satisfy the robot controller contract.
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = self.mount.ground_frame, self.mount.camera_frame
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.mount.origin.tolist()
        q = self.mount.quaternion_xyzw.tolist()
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = q
        self.tf.sendTransform(transform)
        self.received_at = float("-inf")
        self.last_result = None
        self.path_visible = False
        self.subscription = self.create_subscription(String, value("status_topic"), self._detection, 1)
        self.timer = self.create_timer(0.2, self._watchdog)
        self.get_logger().info("VISION PREVIEW ONLY: assumed stationary origin; no odometry or motor-command output.")
        self.get_logger().info(f"Control source-age limit {self.timeout:.2f}s; visual display limit {self.display_timeout:.2f}s. "
                               f"Mount h={self.mount.height_m:.3f}m, pitch={self.mount.pitch_down_deg:.1f}deg; verify current camera position.")

    def _detection(self, message):
        self.received_at = time.monotonic()
        try:
            data = json.loads(message.data)
            result = preview_detection(data, self.calibration, self.mount, self.config,
                                       self.get_clock().now().nanoseconds, self.timeout, self.display_timeout)
        except (ValueError, TypeError, OverflowError):
            result = VisionPreview("invalid_detection_message", np.empty((0, 3)))
        result = age_preview(result, self.get_clock().now().nanoseconds, self.timeout, self.display_timeout,
                             receipt_age_sec=time.monotonic()-self.received_at)
        self.last_result = result
        self._publish(result)

    def _watchdog(self):
        if self.last_result is None:
            result = VisionPreview("waiting_for_detection", np.empty((0, 3)))
        else:
            result = age_preview(self.last_result, self.get_clock().now().nanoseconds, self.timeout, self.display_timeout,
                                 receipt_age_sec=time.monotonic()-self.received_at)
        # Refresh age/status without serializing the whole polyline every tick.
        self._publish(result, publish_path=bool(len(result.path)) != self.path_visible)

    def _publish(self, result, *, publish_path=True):
        metadata = result.metadata()
        receipt_age = time.monotonic()-self.received_at
        metadata.update({"result_received": self.last_result is not None,
                         "result_receipt_age_ms": receipt_age*1000 if math.isfinite(receipt_age) else None,
                         "control_age_limit_ms": self.timeout*1000, "display_age_limit_ms": self.display_timeout*1000,
                         "max_path_offset_m": self.config.max_path_offset_m,
                         "camera_height_m": self.mount.height_m, "camera_pitch_down_deg": self.mount.pitch_down_deg})
        path = Path()
        path.header.frame_id = self.mount.ground_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if result.source_ns is not None and result.source_ns > 0 and len(result.path):
            path.header.stamp.sec, path.header.stamp.nanosec = divmod(result.source_ns, 1_000_000_000)
        if publish_path:
            for x, y, z in result.path:
                pose = PoseStamped(header=path.header)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.path_pub.publish(path)
            self.path_visible = bool(path.poses)
        self.status_pub.publish(String(data=json.dumps(metadata, allow_nan=False)))
        self._markers(result, metadata, path.header)
        self.get_logger().info(
            f"Preview {result.reason} | display {metadata['display_state']} | "
            f"source age {result.age_ms:.0f}ms | geometry {metadata['controller_reason']}"
            if result.age_ms is not None else f"Preview {result.reason} | detection {result.detection_reason}",
            throttle_duration_sec=5.0)

    def _markers(self, result, metadata, header):
        def marker(index, kind):
            item = Marker(header=header, ns="preview_only", id=index, type=kind, action=Marker.ADD)
            item.pose.orientation.w = 1.0
            return item

        text = marker(0, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y = 2.0, 1.65
        text.scale.z = 0.13
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
        candidate = metadata["candidate_before_acceleration_limits"]
        lines = ["VISION PREVIEW | NO MOTOR OUTPUT", "Stationary origin; geometry unverified"]
        if result.image_path_valid is not None:
            lines.append(f"Image path: {'YES' if result.image_path_valid else 'NO'} ({result.detection_reason})")
        lines.append("Ground path: " + metadata["display_state"])
        if candidate is None:
            lines.append("Candidate BLOCKED: " + result.reason)
            if result.detail:
                lines.append(result.detail[:120])
        else:
            v, w = candidate["linear_x_mps"], candidate["angular_z_rps"]
            turn = "LEFT" if w > 0.01 else "RIGHT" if w < -0.01 else "STRAIGHT"
            lines.append(f"v={v:.2f} m/s | omega={w:+.2f} rad/s ({turn})")
        if result.age_ms is not None:
            lines.append(f"Source age: {result.age_ms:.0f} ms | control limit {self.timeout*1000:.0f} ms")
        if result.demand is not None and result.demand.cross_track_m is not None:
            lines.append("Geometry gate: " + result.demand.reason)
            lines.append(f"Origin-to-path: {result.demand.cross_track_m:.2f} m | limit {self.config.max_path_offset_m:.2f} m")
        if len(result.path):
            lines.append(f"Path start: X={result.path[0, 0]:.2f}, Y={result.path[0, 1]:+.2f} m")
        lines.append(f"Mount: h={self.mount.height_m:.2f} m | pitch={self.mount.pitch_down_deg:.1f} deg")
        text.text = "\n".join(line for original in lines for line in textwrap.wrap(original, width=58))
        axis = marker(1, Marker.ARROW)
        axis.scale.x, axis.scale.y, axis.scale.z = 0.55, 0.06, 0.1
        axis.color.b, axis.color.a = 1.0, 1.0
        # Color the retained diagnostic reference explicitly. This marker is
        # separate from /reference_ground, which remains a preview-only Path.
        line = marker(4, Marker.LINE_STRIP)
        line.scale.x = 0.03
        line.color.r, line.color.g, line.color.a = 1.0, 0.9 if result.control_fresh else 0.45, 1.0
        line.points = [Point(x=float(x), y=float(y), z=float(z)+0.02) for x, y, z in result.path]
        if not line.points:
            line.action = Marker.DELETE
        target, arrow = marker(2, Marker.SPHERE), marker(3, Marker.ARROW)
        if candidate is None or result.demand is None or result.demand.target_robot_xy is None:
            target.action = arrow.action = Marker.DELETE
        else:
            x, y = result.demand.target_robot_xy
            target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, 0.03
            target.scale.x = target.scale.y = target.scale.z = 0.09
            target.color.r, target.color.g, target.color.a = 1.0, 1.0, 1.0
            arrow.points = [Point(z=0.03), Point(x=x, y=y, z=0.03)]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.025, 0.06, 0.1
            arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0
        self.marker_pub.publish(MarkerArray(markers=[text, axis, line, target, arrow]))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionPreviewNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
