"""Capture-time AI veto mask in odom, independent of the yellow image path."""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .ground_geometry import CameraCalibration, CameraMount
from .control import quaternion_matrix
from .vision_grid import decode_segmentation, project_veto_grid


class VisionGridNode(Node):
    def __init__(self):
        super().__init__("vision_grid")
        for key, value in dict(camera_calibration="", camera_mount="", odom_frame="odom", max_age_sec=.5).items():
            self.declare_parameter(key, value)
        self.calibration = CameraCalibration.load(self.get_parameter("camera_calibration").value)
        self.mount = CameraMount.load(self.get_parameter("camera_mount").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.max_age = float(self.get_parameter("max_age_sec").value)
        if not 0 < self.max_age <= .5:
            raise ValueError("vision max_age_sec must be positive and <=0.5")
        cv2.setNumThreads(1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(OccupancyGrid, "/husky_nav/vision_mask", qos)
        self.status = self.create_publisher(String, "/husky_nav/vision_status", 1)
        self.subscription = self.create_subscription(String, "/drivable_area/segmentation_packet", self.receive, 1)
        self.last_source, self.received, self.valid = 0, float("-inf"), False
        self.timer = self.create_timer(.1, self.watchdog)

    def report(self, valid, reason, **extra):
        self.valid = valid
        self.status.publish(String(data=json.dumps(dict(valid=valid, reason=reason,
            source_stamp_ns=self.last_source, motion_safety_validated=False, **extra), allow_nan=False)))

    def receive(self, message):
        self.received, self.last_source = time.monotonic(), 0
        try:
            packet, labels = decode_segmentation(message.data)
            header = packet["source_header"]
            stamp = header["stamp"]
            if header["frame_id"] != self.mount.camera_frame or type(stamp["sec"]) is not int or type(stamp["nanosec"]) is not int or not 0 <= stamp["nanosec"] < 10**9:
                raise ValueError("invalid_segmentation_source")
            self.last_source = stamp["sec"]*10**9+stamp["nanosec"]
            age = (self.get_clock().now().nanoseconds-self.last_source)/1e9
            if self.last_source <= 0 or not -.05 <= age <= self.max_age:
                raise ValueError("stale_segmentation")
            tf = self.buffer.lookup_transform(self.odom_frame, self.mount.ground_frame, Time(nanoseconds=self.last_source, clock_type=self.get_clock().clock_type))
            q, t = tf.transform.rotation, tf.transform.translation
            matrix = np.eye(4)
            matrix[:3,:3] = quaternion_matrix(q.x,q.y,q.z,q.w)
            matrix[:3,3] = [t.x,t.y,t.z]
            costs, origin, detail = project_veto_grid(labels, packet["raw_shape_hw"], self.calibration, self.mount, matrix)
            if (self.get_clock().now().nanoseconds-self.last_source)/1e9 > self.max_age:
                raise ValueError("vision_projection_too_old")
            grid = OccupancyGrid()
            grid.header.frame_id = self.odom_frame
            grid.header.stamp.sec, grid.header.stamp.nanosec = divmod(self.last_source,10**9)
            grid.info.resolution = .05
            grid.info.height, grid.info.width = costs.shape
            grid.info.origin.position.x, grid.info.origin.position.y = map(float,origin)
            grid.info.origin.orientation.w = 1.
            grid.data = costs.reshape(-1).tolist()
            self.output.publish(grid)
            self.report(detail["observed_drivable_cells"] > 0, "ok" if detail["observed_drivable_cells"] else "no_observed_drivable_ground", **detail)
        except (ValueError, KeyError, TypeError, OverflowError, cv2.error, TransformException) as exc:
            self.report(False,str(exc)[:160])

    def watchdog(self):
        age = (self.get_clock().now().nanoseconds-self.last_source)/1e9
        if self.last_source <= 0 or not -.05 <= age <= self.max_age or time.monotonic()-self.received > self.max_age:
            self.report(False,"vision_timeout")


def main(args=None):
    rclpy.init(args=args)
    node = VisionGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
