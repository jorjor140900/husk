"""Pure Pursuit ROS adapter. Starts disabled; external permission heartbeat needed."""

from dataclasses import asdict
import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .control import CommandLimiter, ControllerConfig, Demand, fresh_stamp, pursuit, quaternion_matrix, validate_path


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class PathControllerNode(Node):
    def __init__(self):
        super().__init__("path_controller")
        defaults = {
            **asdict(ControllerConfig()), "control_rate_hz": 20.0,
            "path_timeout_sec": 0.5, "odom_timeout_sec": 0.3, "permission_timeout_sec": 0.3,
            "odom_frame": "odom", "base_frame": "base_link", "start_enabled": False,
            "path_topic": "reference", "odom_topic": "odom", "permission_topic": "motion_permitted",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.config = ControllerConfig(**{name: value(name) for name in ControllerConfig.__dataclass_fields__})
        self.limiter = CommandLimiter(self.config)
        self.rate = float(value("control_rate_hz"))
        self.path_timeout, self.odom_timeout, self.permission_timeout = (
            float(value(name)) for name in ("path_timeout_sec", "odom_timeout_sec", "permission_timeout_sec"))
        if any(not math.isfinite(v) or v <= 0 for v in (self.rate, self.path_timeout, self.odom_timeout, self.permission_timeout)) or self.rate > 100:
            raise ValueError("timeouts must be positive; control_rate_hz must be in (0, 100]")
        self.odom_frame, self.base_frame = value("odom_frame"), value("base_frame")
        if (not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame or
                any(c.isspace() for c in self.odom_frame + self.base_frame) or
                self.odom_frame.startswith("/") or self.base_frame.startswith("/")):
            raise ValueError("distinct nonempty odom/base TF frames without spaces or leading slash required")
        self.enabled = bool(value("start_enabled"))
        self.path = self.pose = None
        self.path_source_ns = self.odom_source_ns = 0
        self.path_received = self.odom_received = self.permission_received = float("-inf")
        self.permitted = False
        self.path_error, self.odom_error = "no_path", "no_odometry"
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.command_pub = self.create_publisher(Twist, "control/cmd_vel", 1)
        self.preview_pub = self.create_publisher(Twist, "control/cmd_vel_preview", 1)
        self.status_pub = self.create_publisher(String, "control/status", 1)
        self.path_sub = self.create_subscription(Path, value("path_topic"), self._path, 1)
        self.odom_sub = self.create_subscription(Odometry, value("odom_topic"), self._odom,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.permission_sub = self.create_subscription(Bool, value("permission_topic"), self._permission, 1)
        self.enable_srv = self.create_service(SetBool, "control/set_enabled", self._enable)
        self.last_tick = time.monotonic()
        self.timer = self.create_timer(1 / self.rate, self._tick)
        self.get_logger().info(
            f"Controller {'ENABLED' if self.enabled else 'DISABLED (preview only)'}; "
            f"requires fresh metric path, {self.odom_frame}/{self.base_frame} odometry and permission heartbeat. "
            "Output: control/cmd_vel; no global /cmd_vel remap."
        )

    def _path(self, message):
        self.path = None
        self.path_error = "invalid_path"
        self.path_received = time.monotonic()
        self.path_source_ns = stamp_ns(message.header.stamp)
        try:
            if not fresh_stamp(self.path_source_ns, self.path_received, self.get_clock().now().nanoseconds,
                               time.monotonic(), self.path_timeout):
                self.path_error = "stale_path"
                return
            if len(message.poses) < 2:
                self.path_error = "empty_path"
                return
            frame = message.header.frame_id
            if not frame or any(p.header.frame_id not in ("", frame) for p in message.poses):
                raise ValueError("all path poses must use the path header frame")
            points = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in message.poses])
            if not np.isfinite(points).all():
                raise ValueError("path positions must be finite")
            if frame != self.odom_frame:
                # Freeze the reference into odom at CAPTURE time. Applying a
                # current base transform to an old image would hide robot motion.
                transform = self.buffer.lookup_transform(self.odom_frame, frame, Time.from_msg(message.header.stamp),
                                                          timeout=Duration(seconds=0))
                t, q = transform.transform.translation, transform.transform.rotation
                rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
                translation = np.array([t.x, t.y, t.z])
                if not np.isfinite(translation).all():
                    raise ValueError("TF translation must be finite")
                points = points @ rotation.T + translation
            self.path = validate_path(points[:, :2], self.config.max_segment_m)
            self.path_error = "ok"
        except TransformException:
            self.path_error = "tf_unavailable"
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().warning(f"Path rejected: {exc}", throttle_duration_sec=2.0)

    def _odom(self, message):
        self.pose = None
        self.odom_error = "invalid_odometry"
        self.odom_received = time.monotonic()
        self.odom_source_ns = stamp_ns(message.header.stamp)
        if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
            self.odom_error = "odometry_frame_mismatch"
            return
        try:
            p, q = message.pose.pose.position, message.pose.pose.orientation
            matrix = quaternion_matrix(q.x, q.y, q.z, q.w)
            pose = np.array([p.x, p.y, math.atan2(matrix[1, 0], matrix[0, 0])])
            if not np.isfinite(pose).all() or not math.isfinite(p.z):
                raise ValueError("odometry position must be finite")
            self.pose = pose
            self.odom_error = "ok"
        except (ValueError, TypeError, OverflowError):
            return

    def _permission(self, message):
        self.permitted = message.data
        self.permission_received = time.monotonic()

    def _enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self.limiter.reset()
            self.command_pub.publish(Twist())
        response.success = True
        response.message = "enabled; input/permission checks still required" if self.enabled else "disabled; zero command published"
        return response

    def _demand(self, now_ros_ns, now):
        if now - self.permission_received > self.permission_timeout:
            return Demand(reason="permission_timeout")
        if not self.permitted:
            return Demand(reason="motion_not_permitted")
        if self.pose is None:
            return Demand(reason=self.odom_error)
        if not fresh_stamp(self.odom_source_ns, self.odom_received, now_ros_ns, now, self.odom_timeout):
            return Demand(reason="stale_odometry")
        if self.path is None:
            return Demand(reason=self.path_error)
        if not fresh_stamp(self.path_source_ns, self.path_received, now_ros_ns, now, self.path_timeout):
            return Demand(reason="stale_path")
        return pursuit(self.path, self.pose, self.config)

    def _tick(self):
        now = time.monotonic()
        dt, self.last_tick = max(1e-6, now - self.last_tick), now
        demand = self._demand(self.get_clock().now().nanoseconds, now)
        preview = Twist()
        preview.linear.x, preview.angular.z = demand.linear, demand.angular
        self.preview_pub.publish(preview)
        effective = demand if self.enabled else Demand(reason="disabled")
        v, w = self.limiter.step(effective, dt)
        command = Twist()
        command.linear.x, command.angular.z = v, w
        self.command_pub.publish(command)
        self.status_pub.publish(String(data=json.dumps({
            "enabled": self.enabled, "reason": effective.reason, "candidate_reason": demand.reason,
            "linear_x": v, "angular_z": w, "cross_track_m": demand.cross_track_m,
            "remaining_m": demand.remaining_m, "target_robot_xy": demand.target_robot_xy,
            "collision_checked_by_controller": False,
        }, allow_nan=False)))

    def stop(self):
        self.limiter.reset()
        self.command_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathControllerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        rclpy.try_shutdown()
