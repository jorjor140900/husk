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
        self.last_timings = {}
        self.pending = None
        self.waiting_for_tf = False
        # A capture can arrive a few milliseconds before the matching odometry
        # TF. Retry the one latest packet without blocking TF's subscription.
        self.retry_timer = self.create_timer(.02, self.retry_projection)
        self.timer = self.create_timer(.1, self.watchdog)

    def report(self, valid, reason, **extra):
        self.valid = valid
        self.status.publish(String(data=json.dumps(dict(valid=valid, reason=reason,
            source_stamp_ns=self.last_source, motion_safety_validated=False,
            timings_ms=getattr(self, "last_timings", {}), **extra), allow_nan=False)))

    def receive(self, message):
        self.received, self.last_source = time.monotonic(), 0
        self.pending = None
        self.waiting_for_tf = False
        started = time.perf_counter()
        self.last_timings = {}
        try:
            packet, labels = decode_segmentation(message.data)
            self.last_timings["decode"] = (time.perf_counter()-started)*1000
            header = packet["source_header"]
            stamp = header["stamp"]
            if header["frame_id"] != self.mount.camera_frame or type(stamp["sec"]) is not int or type(stamp["nanosec"]) is not int or not 0 <= stamp["nanosec"] < 10**9:
                raise ValueError("invalid_segmentation_source")
            self.last_source = stamp["sec"]*10**9+stamp["nanosec"]
            age = (self.get_clock().now().nanoseconds-self.last_source)/1e9
            self.last_timings["source_age_at_receive"] = age*1000
            if self.last_source <= 0 or not -.05 <= age <= self.max_age:
                raise ValueError("stale_segmentation")
            self.pending = (packet, labels, started)
            self.retry_projection()
        except (ValueError, KeyError, TypeError, OverflowError, cv2.error) as exc:
            self.pending = None
            self.last_timings["callback"] = (time.perf_counter()-started)*1000
            self.report(False,str(exc)[:160])

    def retry_projection(self):
        if self.pending is None:
            return
        packet, labels, started = self.pending
        try:
            age = (self.get_clock().now().nanoseconds-self.last_source)/1e9
            if not -.05 <= age <= self.max_age or time.monotonic()-self.received>self.max_age:
                raise ValueError("stale_segmentation_waiting_for_tf")
            tf_started = time.perf_counter()
            try:
                tf = self.buffer.lookup_transform(self.odom_frame, self.mount.ground_frame,
                    Time(nanoseconds=self.last_source, clock_type=self.get_clock().clock_type))
            except TransformException as exc:
                if not self.waiting_for_tf:
                    self.waiting_for_tf = True
                    self.report(False,"waiting_for_capture_tf",tf_error=str(exc)[:160])
                return
            self.last_timings["capture_tf_wait"] = (time.perf_counter()-started)*1000
            q, t = tf.transform.rotation, tf.transform.translation
            matrix = np.eye(4)
            matrix[:3,:3] = quaternion_matrix(q.x,q.y,q.z,q.w)
            matrix[:3,3] = [t.x,t.y,t.z]
            self.last_timings["capture_tf"] = (time.perf_counter()-tf_started)*1000
            projection_started = time.perf_counter()
            costs, origin, detail = project_veto_grid(labels, packet["raw_shape_hw"], self.calibration, self.mount, matrix)
            self.last_timings["projection"] = (time.perf_counter()-projection_started)*1000
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
            if (self.get_clock().now().nanoseconds-self.last_source)/1e9 > self.max_age:
                raise ValueError("vision_projection_too_old")
            self.output.publish(grid)
            self.pending = None
            self.last_timings["callback"] = (time.perf_counter()-started)*1000
            self.last_timings["source_age_at_publish"] = (self.get_clock().now().nanoseconds-self.last_source)/1e6
            self.report(detail["observed_drivable_cells"] > 0, "ok" if detail["observed_drivable_cells"] else "no_observed_drivable_ground", **detail)
        except (ValueError, KeyError, TypeError, OverflowError, cv2.error, TransformException) as exc:
            self.pending = None
            self.last_timings["callback"] = (time.perf_counter()-started)*1000
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
