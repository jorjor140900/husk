"""Combine bridge health with an independent, fresh field permission source."""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class PermissionNode(Node):
    def __init__(self):
        super().__init__("field_permission")
        for name, value in {"link_topic": "/husky_link/motion_permitted", "external_topic": "/husky_safety/motion_permitted", "timeout_sec": 0.3}.items():
            self.declare_parameter(name, value)
        self.timeout = float(self.get_parameter("timeout_sec").value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("permission timeout must be finite and positive")
        self.inputs = {"link": (False, float("-inf")), "external": (False, float("-inf"))}
        self.subs = [self.create_subscription(Bool, self.get_parameter(name+"_topic").value,
                     lambda message, kind=name: self._input(kind, message), 1) for name in self.inputs]
        self.output = self.create_publisher(Bool, "motion_permitted", 1)
        self.timer = self.create_timer(0.05, self._tick)

    def _input(self, kind, message):
        self.inputs[kind] = (message.data, time.monotonic())

    def _tick(self):
        now = time.monotonic()
        self.output.publish(Bool(data=all(value and now-received <= self.timeout for value, received in self.inputs.values())))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PermissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.output.publish(Bool(data=False))
            node.destroy_node()
        rclpy.try_shutdown()
