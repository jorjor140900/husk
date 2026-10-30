"""Nav2's private stamped requests -> existing collision filter, disabled first."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException

from .control import fresh_stamp
from .controller_node import stamp_ns


class NavigationGate(Node):
    def __init__(self, **kwargs):
        super().__init__("navigation_gate", **kwargs)
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("supervised_trial", False)
        self.supervised_trial=self.get_parameter("supervised_trial").value
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.allow_motion = self.get_parameter("allow_motion").value
        self.base, self.odom_frame = self.get_parameter("base_frame").value, self.get_parameter("odom_frame").value
        self.enabled = False
        self.command = self.vision = self.odom = None
        self.permission = (False, float("-inf"))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.output = self.create_publisher(Twist, "/husky_nav/guarded_request", 1)
        self.status = self.create_publisher(String, "/husky_nav/control_status", 1)
        self.subs = [
            self.create_subscription(TwistStamped, "/husky_nav/nav2_cmd_vel", self.receive_command, 1),
            self.create_subscription(String, "/husky_nav/vision_status", self.receive_vision, 1),
            self.create_subscription(Bool, "/husky_control/motion_permitted", self.receive_permission, 1),
            self.create_subscription(Odometry, "/husky_link/odom", self.receive_odom,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
        ]
        self.service = self.create_service(SetBool, "/husky_nav/set_enabled", self.enable)
        self.timer = self.create_timer(.05, self.tick)

    def receive_command(self, message):
        self.command = (message, time.monotonic())

    def receive_vision(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("invalid vision status")
            self.vision = (value, time.monotonic())
        except (ValueError, TypeError):
            self.vision = None

    def receive_permission(self, message):
        self.permission = (message.data, time.monotonic())

    def receive_odom(self, message):
        self.odom = (message, time.monotonic())

    def enable(self, request, response):
        self.enabled = bool(request.data and self.allow_motion)
        response.success = not request.data or self.allow_motion
        response.message = "enabled; all live gates still required" if self.enabled else "disabled / planning-only launch"
        if not self.enabled:
            self.output.publish(Twist())
        return response

    def demand(self):
        now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
        if not self.allow_motion:
            return "planning_only", None
        if not self.enabled:
            return "disabled", None
        if not self.permission[0] or now-self.permission[1] > .3:
            return "permission_missing_or_stale", None
        if self.vision is None or self.vision[0].get("valid") is not True:
            return "vision_invalid", None
        if not fresh_stamp(self.vision[0].get("source_stamp_ns",0),self.vision[1],ros_ns,now,.5):
            return "vision_stale", None
        if self.odom is None:
            return "no_odometry", None
        odom, received = self.odom
        if (odom.header.frame_id != self.odom_frame or odom.child_frame_id != self.base or
                not fresh_stamp(stamp_ns(odom.header.stamp),received,ros_ns,now,.3)):
            return "odometry_invalid_or_stale", None
        try:
            tf = self.buffer.lookup_transform("map",self.base,Time())
            if not fresh_stamp(stamp_ns(tf.header.stamp),now,ros_ns,now,.3):
                return "localization_stale", None
        except TransformException:
            return "localization_unavailable", None
        if self.command is None:
            return "no_nav2_command", None
        command, received = self.command
        if not fresh_stamp(stamp_ns(command.header.stamp),received,ros_ns,now,.2):
            return "nav2_command_stale", None
        t = command.twist
        values = (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)
        if (command.header.frame_id != self.base or not all(math.isfinite(v) for v in values) or
                any(v != 0 for v in (t.linear.y,t.linear.z,t.angular.x,t.angular.y)) or
                not 0 <= t.linear.x <= .050001 or abs(t.angular.z) > .200001):
            return "invalid_nav2_command", None
        return "request_passed_to_collision_filter", t

    def tick(self):
        try:
            reason, demand = self.demand()
        except (ValueError, TypeError, OverflowError):
            reason, demand = "invalid_navigation_inputs", None
        command = demand if demand is not None else Twist()
        self.output.publish(command)
        self.status.publish(String(data=json.dumps(dict(enabled=self.enabled, allow_motion=self.allow_motion,
            reason=reason, request_allowed=demand is not None, linear_x=command.linear.x, angular_z=command.angular.z,
            downstream_collision_filter_required=not getattr(self,"supervised_trial",False),
            supervised_trial=getattr(self,"supervised_trial",False), ready_for_autonomous_control=False))))


def stop_node(node):
    try:
        if rclpy.ok():
            try:
                node.output.publish(Twist())
            except Exception:
                # SIGINT may invalidate the context between ok() and publish.
                if rclpy.ok():
                    raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stop_node(node)
