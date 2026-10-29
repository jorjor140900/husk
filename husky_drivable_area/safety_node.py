"""Experimental downstream LaserScan command filter; never bypass on lost data."""

from collections import deque
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
from geometry_msgs.msg import Point, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from .collision import CollisionConfig, check_sweep, scan_points
from .control import CommandLimiter, ControllerConfig, Demand, fresh_stamp, quaternion_matrix
from .controller_node import stamp_ns


class LaserSafetyNode(Node):
    def __init__(self):
        super().__init__("laser_safety")
        for name, value in {**asdict(CollisionConfig()), "odom_frame": "odom", "base_frame": "base_link",
                            "scan_timeout_sec": 0.3, "odom_timeout_sec": 0.3,
                            "request_timeout_sec": 0.3, "permission_timeout_sec": 0.3,
                            "release_hold_sec": 0.5, "output_acceleration_mps2": 0.25,
                            "output_angular_acceleration_rps2": 0.8}.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.config = CollisionConfig(**{name: value(name) for name in CollisionConfig.__dataclass_fields__})
        self.odom_frame, self.base_frame = value("odom_frame"), value("base_frame")
        if not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame:
            raise ValueError("distinct odom_frame/base_frame required")
        self.timeouts = {name: float(value(name + "_timeout_sec")) for name in ("scan", "odom", "request", "permission")}
        if any(not math.isfinite(v) or v <= 0 for v in self.timeouts.values()):
            raise ValueError("timeouts must be finite and positive")
        self.release_hold = float(value("release_hold_sec"))
        if not math.isfinite(self.release_hold) or self.release_hold <= 0:
            raise ValueError("release_hold_sec must be finite and positive")
        self.limiter = CommandLimiter(ControllerConfig(cruise_speed_mps=self.config.max_linear_mps,
            max_angular_speed_rps=self.config.max_angular_rps, max_acceleration_mps2=value("output_acceleration_mps2"),
            max_angular_acceleration_rps2=value("output_angular_acceleration_rps2")))
        self.clear_since = None
        self.last_tick = time.monotonic()
        self.scans = deque(maxlen=2)
        self.scan_error = "no_scan"
        self.odom = None
        self.request = (0.0, 0.0)
        self.proposed = (0.0, 0.0)
        self.request_received = self.permission_received = float("-inf")
        self.permitted = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self._scan, qos)
        self.odom_sub = self.create_subscription(Odometry, "odom", self._odom, qos)
        self.request_sub = self.create_subscription(Twist, "control/requested_cmd_vel", self._request, 1)
        self.permission_sub = self.create_subscription(Bool, "motion_permitted", self._permission, 1)
        self.output = self.create_publisher(Twist, "control/cmd_vel", 1)
        self.stamped_output = self.create_publisher(TwistStamped, "control/cmd_vel_stamped", 1)
        self.status = self.create_publisher(String, "safety/status", 1)
        self.marker = self.create_publisher(Marker, "safety/sweep", 1)
        self.timer = self.create_timer(0.05, self._tick)

    def _scan(self, message):
        try:
            received = time.monotonic()
            if not fresh_stamp(stamp_ns(message.header.stamp), received, self.get_clock().now().nanoseconds,
                               received, self.timeouts["scan"]):
                raise ValueError("stale_scan")
            if not message.header.frame_id:
                raise ValueError("invalid_scan_frame")
            points = scan_points(message.ranges, message.angle_min, message.angle_max, message.angle_increment,
                                 message.range_min, message.range_max, message.time_increment)
            if self.scans and (message.header.frame_id != self.scans[-1][0].header.frame_id or
                    stamp_ns(message.header.stamp) < stamp_ns(self.scans[-1][0].header.stamp)):
                self.scans.clear()
            self.scans.append((message, received, points))
            self.scan_error = "ok"
        except (ValueError, TypeError, OverflowError) as exc:
            self.scans.clear()
            self.scan_error = str(exc)

    def _odom(self, message):
        self.odom = None
        try:
            if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
                return
            p, q = message.pose.pose.position, message.pose.pose.orientation
            rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
            if abs(rotation[2, 2] - 1) > 1e-3:
                return
            position = np.array([p.x, p.y])
            velocity = (message.twist.twist.linear.x, message.twist.twist.angular.z)
            if not np.isfinite(position).all() or not math.isfinite(p.z) or not all(math.isfinite(v) for v in velocity):
                return
            self.odom = (message, time.monotonic(), position, rotation[:2, :2], velocity)
        except (ValueError, TypeError, OverflowError):
            return

    def _request(self, message):
        self.request = (message.linear.x, message.angular.z)
        self.request_received = time.monotonic()

    def _permission(self, message):
        self.permitted = message.data
        self.permission_received = time.monotonic()

    def _check(self):
        now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
        if now - self.permission_received > self.timeouts["permission"]:
            return "permission_timeout", None
        if not self.permitted:
            return "motion_not_permitted", None
        if now - self.request_received > self.timeouts["request"]:
            return "stale_request", None
        if self.odom is None:
            return "invalid_or_missing_odometry", None
        odom, received, position, rotation, velocity = self.odom
        if not fresh_stamp(stamp_ns(odom.header.stamp), received, ros_ns, now, self.timeouts["odom"]):
            return "stale_odometry", None
        if not self.scans:
            return self.scan_error, None
        reason = "stale_scan"
        for scan, received, points in reversed(self.scans):
            if not fresh_stamp(stamp_ns(scan.header.stamp), received, ros_ns, now, self.timeouts["scan"]):
                continue
            try:
                transform = self.buffer.lookup_transform(self.odom_frame, scan.header.frame_id,
                    Time.from_msg(scan.header.stamp), timeout=Duration(seconds=0))
            except TransformException:
                reason = "scan_tf_unavailable"
                continue
            q, t = transform.transform.rotation, transform.transform.translation
            scan_rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
            if abs(scan_rotation[2, 2] - 1) > 1e-3 or not all(math.isfinite(v) for v in (t.x, t.y, t.z)):
                return "nonplanar_or_invalid_scan_tf", None
            # Store/project in fixed odom at scan time, then express in current
            # base using odometry; never use a current TF for old scan points.
            scan_origin = np.array([t.x, t.y])
            fixed = points @ scan_rotation[:2, :2].T + scan_origin
            current = (fixed - position) @ rotation
            origin = (scan_origin - position) @ rotation
            age = max(0, (ros_ns-stamp_ns(scan.header.stamp))/1e9) + max(0, (ros_ns-stamp_ns(odom.header.stamp))/1e9)
            return check_sweep(current, self.request, velocity, self.config, extra_reaction_sec=age,
                               sensor_origin_base=origin, sensor_range_m=scan.range_max,
                               additional_velocities=(self.proposed,))
        return reason, None

    def _tick(self):
        now = time.monotonic()
        dt, self.last_tick = max(1e-6, now-self.last_tick), now
        demand = Demand(*self.request, "tracking") if any(self.request) else Demand(reason="zero_request")
        self.proposed = self.limiter.step(demand, dt)
        try:
            reason, poses = self._check()
        except (ValueError, TypeError, OverflowError, FloatingPointError):
            reason, poses = "invalid_geometry_or_motion", None
        if reason == "clear":
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since < self.release_hold:
                reason = "waiting_for_clear"
        else:
            self.clear_since = None
        command = Twist()
        if reason == "clear":
            command.linear.x, command.angular.z = self.proposed
        else:
            self.limiter.reset()
        self._publish_command(command)
        self.status.publish(String(data=json.dumps({"reason": reason, "command_allowed": reason == "clear",
            "linear_x": command.linear.x, "angular_z": command.angular.z,
            "model": "planar_scan_and_command_stop_sweeps", "physical_robot_validated": False})))
        if self.marker.get_subscription_count() > 0:
            self._draw(poses, reason)

    def _publish_command(self, command):
        self.output.publish(command)
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.base_frame
        stamped.twist = command
        self.stamped_output.publish(stamped)

    def _draw(self, poses, reason):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        if self.odom is not None:
            marker.header.stamp = self.odom[0].header.stamp
        marker.ns, marker.id, marker.type = "sweep", 0, Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r, marker.color.g, marker.color.a = (0.1 if reason == "clear" else 1.0), (1.0 if reason == "clear" else 0.1), 0.8
        if poses is not None:
            corners = self.config.padded_corners()
            for x, y, yaw in poses[::max(1, len(poses)//20)]:
                c, s = math.cos(yaw), math.sin(yaw)
                polygon = corners @ np.array([[c, s], [-s, c]]) + [x, y]
                for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                    marker.points.extend([Point(x=float(a[0]), y=float(a[1]), z=0.03), Point(x=float(b[0]), y=float(b[1]), z=0.03)])
        self.marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaserSafetyNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._publish_command(Twist())
            node.destroy_node()
        rclpy.try_shutdown()
