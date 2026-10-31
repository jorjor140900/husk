"""Publish a mapping-only scan; raw safety input is deliberately untouched."""
import copy
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .navigation import mapping_scan_geometry
from .controller_node import stamp_ns


class MappingScanNode(Node):
    def __init__(self):
        super().__init__("mapping_scan")
        self.declare_parameter("input_topic", "/husky_link/scan")
        self.output = self.create_publisher(LaserScan, "/husky_nav/mapping_scan", 5)
        self.status = self.create_publisher(String, "/husky_nav/mapping_scan_status", 1)
        self.subscription = self.create_subscription(LaserScan, self.get_parameter("input_topic").value,
            self.receive, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def receive(self, message):
        try:
            age = (self.get_clock().now().nanoseconds-stamp_ns(message.header.stamp))/1e9
            if not message.header.frame_id or not -.05 <= age <= .3 or stamp_ns(message.header.stamp) <= 0:
                raise ValueError("stale_or_invalid_mapping_scan")
            geometry = mapping_scan_geometry(message)
            output = copy.deepcopy(message)
            for key, value in geometry.items():
                setattr(output, key, value)
            self.output.publish(output)
            reason = "mapping_only_not_safety_validated"
        except (ValueError, TypeError) as exc:
            reason = str(exc)
        self.status.publish(String(data=json.dumps(dict(reason=reason, mapping_only=True,
            motion_safety_validated=False, note="Original stamps, acquisition timing and no-return values retained."))))


def main(args=None):
    rclpy.init(args=args)
    node = MappingScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
