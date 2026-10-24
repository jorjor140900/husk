"""Display real vision references and hypothetical commands; no actuator publisher."""

from dataclasses import asdict
import json
import math
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
from .vision_preview import VisionPreview, preview_detection


class VisionPreviewNode(Node):
    def __init__(self):
        super().__init__("vision_control_preview")
        defaults = {**asdict(ControllerConfig()), "camera_calibration": "", "camera_mount": "",
                    "status_topic": "/drivable_area/status", "path_timeout_sec": 0.5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        if not value("camera_calibration") or not value("camera_mount"):
            raise ValueError("measured camera_calibration and camera_mount files are required")
        self.calibration = CameraCalibration.load(value("camera_calibration"))
        self.mount = CameraMount.load(value("camera_mount"))
        self.config = ControllerConfig(**{name: value(name) for name in ControllerConfig.__dataclass_fields__})
        self.timeout = float(value("path_timeout_sec"))
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("path_timeout_sec must be finite and positive")
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
        self.subscription = self.create_subscription(String, value("status_topic"), self._detection, 1)
        self.timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info("VISION PREVIEW ONLY: assumed stationary origin; no odometry or motor-command output.")

    def _detection(self, message):
        self.received_at = time.monotonic()
        try:
            data = json.loads(message.data)
            result = preview_detection(data, self.calibration, self.mount, self.config,
                                       self.get_clock().now().nanoseconds, self.timeout)
        except (ValueError, TypeError, OverflowError):
            result = VisionPreview("invalid_detection_message", np.empty((0, 3)))
        if result.source_ns is not None and len(result.path):
            age = (self.get_clock().now().nanoseconds-result.source_ns)/1e9
            if not -0.05 <= age <= self.timeout:
                result = VisionPreview("stale_after_projection", np.empty((0, 3)), source_ns=result.source_ns, age_ms=age*1000)
        self.last_result = result
        self._publish(result)

    def _watchdog(self):
        result = self.last_result
        now_ns = self.get_clock().now().nanoseconds
        stale = (time.monotonic()-self.received_at > self.timeout or
                 (result is not None and result.source_ns is not None and
                  not -0.05 <= (now_ns-result.source_ns)/1e9 <= self.timeout))
        if stale:
            self._publish(VisionPreview("no_recent_detection", np.empty((0, 3))))

    def _publish(self, result):
        metadata = result.metadata()
        path = Path()
        path.header.frame_id = self.mount.ground_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if result.source_ns is not None and result.source_ns > 0 and len(result.path):
            path.header.stamp.sec, path.header.stamp.nanosec = divmod(result.source_ns, 1_000_000_000)
        for x, y, z in result.path:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=json.dumps(metadata, allow_nan=False)))
        self._markers(result, metadata, path.header)

    def _markers(self, result, metadata, header):
        def marker(index, kind):
            item = Marker(header=header, ns="preview_only", id=index, type=kind, action=Marker.ADD)
            item.pose.orientation.w = 1.0
            return item

        text = marker(0, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y = 2.0, 1.8
        text.scale.z = 0.17
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
        candidate = metadata["candidate_before_acceleration_limits"]
        text.text = "VISION PREVIEW ONLY - NO MOTOR OUTPUT\nAssumed stationary origin; ground geometry unverified\n"
        if candidate is None:
            text.text += "NO CANDIDATE: " + result.reason
            if result.detail:
                text.text += "\n" + result.detail[:120]
        else:
            v, w = candidate["linear_x_mps"], candidate["angular_z_rps"]
            turn = "LEFT" if w > 0.01 else "RIGHT" if w < -0.01 else "STRAIGHT"
            text.text += f"candidate v={v:.2f} m/s | omega={w:+.2f} rad/s ({turn})"
        if result.age_ms is not None:
            text.text += f"\nImage age {result.age_ms:.0f} ms | before acceleration limits"
        if result.demand is not None and result.demand.cross_track_m is not None:
            text.text += f"\nDistance from assumed origin to reference: {result.demand.cross_track_m:.2f} m"
        axis = marker(1, Marker.ARROW)
        axis.scale.x, axis.scale.y, axis.scale.z = 0.55, 0.06, 0.1
        axis.color.b, axis.color.a = 1.0, 1.0
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
        self.marker_pub.publish(MarkerArray(markers=[text, axis, target, arrow]))


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
