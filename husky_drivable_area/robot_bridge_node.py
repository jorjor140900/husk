"""ROS1 String gateway -> native ROS2 odometry/scan/TF; observe by default."""

import json
import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from husky_link_core.protocol import finite, frame_id, stamp_ns
from .robot_link import RosbridgeLink
from .robot_camera import decode_camera
from .transport import fill_image


def ros2_fields(value):
    if isinstance(value, dict):
        if set(value) == {"secs", "nsecs"}:
            stamp_ns(value)
            return {"sec": value["secs"], "nanosec": value["nsecs"]}
        return {key: frame_id(item) if key in ("frame_id", "child_frame_id") else ros2_fields(item)
                for key, item in value.items() if not (key == "seq" and "stamp" in value and "frame_id" in value)}
    if isinstance(value, list):
        return [ros2_fields(item) for item in value]
    return value


class RobotBridgeNode(Node):
    def __init__(self):
        super().__init__("robot_bridge")
        defaults = {"bridge_url": "ws://127.0.0.1:9090", "ros1_prefix": "/husky_vision", "forward_commands": False,
                    "base_frame": "", "expected_output_topic": "", "command_topic": "/husky_control/control/cmd_vel_stamped",
                    "expected_camera_width": 0, "expected_camera_height": 0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos)
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos)
        self.image_pub = self.create_publisher(Image, "image_raw", qos)
        self.camera_width, self.camera_height = value("expected_camera_width"), value("expected_camera_height")
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.static_pub = self.create_publisher(TFMessage, "/tf_static",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(String, "status", 1)
        self.permission_pub = self.create_publisher(Bool, "motion_permitted", 1)
        self.external_permission_pub = self.create_publisher(Bool, "external_permission", 1)
        self.command_lock, self.last_command = threading.Lock(), None
        self.decode_errors = {}
        self.link = RosbridgeLink(value("bridge_url"), prefix=value("ros1_prefix"), forward_commands=value("forward_commands"),
                                 base_frame=value("base_frame"), expected_output_topic=value("expected_output_topic"),
                                 command_provider=self._motion, ros_clock=lambda: self.get_clock().now().nanoseconds)
        self.command_sub = self.create_subscription(TwistStamped, value("command_topic"), self._command, 1)
        self.enable_service = self.create_service(SetBool, "set_enabled", self._enable)
        self.timer = self.create_timer(0.05, self._tick)
        self.link.start()
        self.get_logger().info("ROS1 link starts disabled; observe mode="+str(not value("forward_commands"))+". Source timestamps and scan unknowns are preserved.")

    def _command(self, message):
        with self.command_lock:
            self.last_command = (message, time.monotonic())

    def _motion(self, receiver):
        with self.command_lock:
            if self.last_command is None or self.decode_errors:
                return None
            message, received = self.last_command
            source = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-source)/1e9
            twist = message.twist
            v, w = twist.linear.x, twist.angular.z
            if (source <= 0 or not -0.05 <= age <= 0.20 or time.monotonic()-received > 0.20 or
                    frame_id(message.header.frame_id) != self.link.base_frame or
                    any(x != 0 for x in (twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y)) or
                    not finite(v) or not finite(w) or not 0 <= v <= min(0.10, receiver["max_linear_mps"]) or
                    abs(w) > min(0.30, receiver["max_angular_rps"])):
                return None
            return {"source_ns": source, "linear_x": v, "angular_z": w}

    def _enable(self, request, response):
        response.success, response.message = self.link.enable(request.data)
        return response

    def _tick(self):
        for kind, packet in self.link.take().items():
            if kind == "state":
                continue
            try:
                if kind == "image":
                    image = decode_camera(packet["msg"], self.camera_width, self.camera_height)
                    header = Header()
                    set_message_fields(header, ros2_fields(packet["msg"]["header"]))
                    self.image_pub.publish(fill_image(Image(), image, header))
                    with self.command_lock:
                        self.decode_errors.pop(kind, None)
                    continue
                cls = {"odom": Odometry, "scan": LaserScan, "tf": TFMessage, "tf_static": TFMessage, "permission": Bool}[kind]
                message = cls()
                set_message_fields(message, ros2_fields(packet["msg"]))
                if kind in ("tf", "tf_static"):
                    if not 1 <= len(message.transforms) <= 1000:
                        raise ValueError("invalid_tf_count")
                    for transform in message.transforms:
                        self._transform_valid(transform.transform)
                        if transform.header.frame_id == transform.child_frame_id:
                            raise ValueError("tf_parent_equals_child")
                elif kind == "odom":
                    self._transform_valid(message.pose.pose)
                    vals = [*message.pose.covariance, *message.twist.covariance,
                            message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z,
                            message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z]
                    if not all(math.isfinite(x) for x in vals):
                        raise ValueError("invalid_odometry")
                elif kind == "scan" and not 180 <= len(message.ranges) <= 10000:
                    raise ValueError("invalid_scan_size")
                elif kind == "permission":
                    received_ns = stamp_ns(packet["received_stamp"])
                    if received_ns <= 0 or not -0.05 <= (self.get_clock().now().nanoseconds-received_ns)/1e9 <= 0.3:
                        raise ValueError("stale_robot_permission")
                publisher = {"odom": self.odom_pub, "scan": self.scan_pub, "tf": self.tf_pub,
                             "tf_static": self.static_pub, "permission": self.external_permission_pub}[kind]
                publisher.publish(message)
                with self.command_lock:
                    self.decode_errors.pop(kind, None)
            except (ValueError, TypeError, KeyError, AttributeError, AssertionError, OverflowError) as exc:
                if kind == "permission":
                    self.external_permission_pub.publish(Bool(data=False))
                with self.command_lock:
                    self.decode_errors[kind] = str(exc)[:160]
        status = self.link.snapshot()
        with self.command_lock:
            status["decode_errors"] = dict(self.decode_errors)
        status["ready_for_autonomous_control"] = False
        self.permission_pub.publish(Bool(data=status["motion_permitted"] and not status["decode_errors"]))
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    @staticmethod
    def _transform_valid(value):
        q = value.rotation if hasattr(value, "rotation") else value.orientation
        p = value.translation if hasattr(value, "translation") else value.position
        if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 0.002):
            raise ValueError("invalid_pose_or_transform")

    def stop(self):
        self.link.enable(False)
        self.link.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()
