"""Visualize the existing pixel reference in meters on a measured flat plane."""

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .ground_geometry import CameraCalibration, CameraMount, project_path


class GroundPathNode(Node):
    def __init__(self):
        super().__init__("ground_path")
        for name, value in {
            "camera_calibration": "", "camera_mount": "",
            "status_topic": "/drivable_area/status", "max_frame_age_sec": 2.0,
            "max_distance_m": 5.0, "max_segment_m": 0.5,
            "publish_camera_tf": True,
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        if not value("camera_calibration") or not value("camera_mount"):
            raise ValueError("camera_calibration YAML and measured camera_mount JSON paths are required")
        self.calibration = CameraCalibration.load(value("camera_calibration"))
        self.mount = CameraMount.load(value("camera_mount"))
        self.max_age = float(value("max_frame_age_sec"))
        self.max_distance = float(value("max_distance_m"))
        self.max_segment = float(value("max_segment_m"))
        if any(not math.isfinite(v) or v <= 0 for v in (self.max_age, self.max_distance, self.max_segment)):
            raise ValueError("age, distance and segment limits must be finite and positive")
        self.path_pub = self.create_publisher(Path, "drivable_area/reference_ground", 1)
        self.status_pub = self.create_publisher(String, "drivable_area/ground_status", 1)
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.mount.ground_frame
        transform.child_frame_id = self.mount.camera_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.mount.origin.tolist()
        q = self.mount.quaternion_xyzw.tolist()
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = q
        if value("publish_camera_tf"):
            self.tf.sendTransform(transform)
        # Status already contains path + both image sizes + source stamp in one
        # message. Do not pair asynchronous image/pixel topics or add image load.
        self.subscription = self.create_subscription(String, value("status_topic"), self._status, 1)
        self.received_at = time.monotonic()
        self.source_ns = None
        self.active = False
        self.timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f"Flat ground projection -> /drivable_area/reference_ground in {self.mount.ground_frame}; "
            "visualization only, no footprint check or motor commands."
        )

    def _publish(self, points, reason, source_stamp=None):
        path = Path()
        path.header.frame_id = self.mount.ground_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if source_stamp is not None:
            path.header.stamp.sec, path.header.stamp.nanosec = source_stamp
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
            previous, following = points[max(0, index - 1)], points[min(len(points) - 1, index + 1)]
            yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.active = bool(path.poses)
        self.status_pub.publish(String(data=json.dumps({
            "valid_ground_projection": self.active, "reason": reason,
            "frame_id": self.mount.ground_frame, "point_count": len(path.poses),
            "projection_model": "flat_ground", "verified_traversability": False,
            "footprint_checked": False, "ready_for_control": False,
        })))

    def _status(self, message):
        self.received_at = time.monotonic()
        self.source_ns = None
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("status must be a JSON object")
            if data.get("valid_image_path") is not True:
                self._publish([], "image_path_invalid:" + str(data.get("reason")))
                return
            if data.get("path_coordinate_system") != "processed_image_pixels_xy_near_to_far":
                raise ValueError("unsupported pixel coordinate system")
            source = data["source_header"]
            if source["frame_id"] != self.mount.camera_frame:
                raise ValueError("source camera frame differs from measured mount")
            sec, ns = source["stamp"]["sec"], source["stamp"]["nanosec"]
            if type(sec) is not int or type(ns) is not int or not 0 <= ns < 1_000_000_000:
                raise ValueError("invalid source timestamp")
            self.source_ns = sec * 1_000_000_000 + ns
            age = (self.get_clock().now().nanoseconds - self.source_ns) / 1e9
            if self.source_ns <= 0 or age < -0.1 or age > self.max_age:
                self._publish([], "invalid_or_stale_source_timestamp")
                return
            points, reason = project_path(
                data["path"], data["processed_shape_hw"], data["input_shape_hw"], self.calibration, self.mount,
                max_distance_m=self.max_distance, max_segment_m=self.max_segment,
            )
            self._publish(points, reason, (sec, ns))
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            self._publish([], "projection_rejected")
            self.get_logger().warning(f"Projection rejected: {exc}", throttle_duration_sec=2.0)

    def _watchdog(self):
        if not self.active:
            return
        age = ((self.get_clock().now().nanoseconds - self.source_ns) / 1e9) if self.source_ns else self.max_age + 1
        if time.monotonic() - self.received_at > self.max_age or age > self.max_age or age < -0.1:
            self._publish([], "source_timeout")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GroundPathNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
