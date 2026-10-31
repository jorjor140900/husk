"""Live deskewed observed VLP16 returns. No free-space inference or motor output."""
from array import array
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .control import quaternion_matrix
from .vlp16 import read_timed_cloud, deskew_returns, stamp_ns


class Vlp16Observer(Node):
    def __init__(self):
        super().__init__("vlp16_observer")
        for key, value in dict(base_frame="base_link", odom_frame="odom", input_mode="inspect").items():
            self.declare_parameter(key, value)
        self.base = self.get_parameter("base_frame").value
        self.odom = self.get_parameter("odom_frame").value
        self.mode = self.get_parameter("input_mode").value
        if self.mode not in ("inspect", "raw_timed"):
            raise ValueError("input_mode must be inspect or raw_timed (after driver timing verification)")
        if not self.base or not self.odom or self.base == self.odom:
            raise ValueError("distinct base/odom frames required")
        self.poses = deque(maxlen=300)
        self.pending = None
        self.last_ns = 0
        self.received = float("-inf")
        self.valid = False
        self.last_reason = "waiting_for_cloud"
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output = self.create_publisher(PointCloud2, "/husky_nav/vlp16_observed_points", qos)
        self.status = self.create_publisher(String, "/husky_nav/vlp16_status", 1)
        self.subs = [self.create_subscription(PointCloud2, "/husky_link/points", self.cloud, qos),
                     self.create_subscription(Odometry, "/husky_link/odom", self.odometry, qos)]
        self.timer = self.create_timer(.02, self.retry)
        self.watchdog = self.create_timer(.1, self.health)

    def report(self, valid, reason, **details):
        self.valid, self.last_reason = valid, reason
        self.status.publish(String(data=json.dumps(dict(valid=valid, reason=reason,
            source_stamp_ns=self.last_ns, motion_safety_validated=False, coverage_validated=False,
            control_output=False, input_mode=self.mode, **details), allow_nan=False)))

    def odometry(self, message):
        try:
            if message.header.frame_id != self.odom or message.child_frame_id != self.base:
                raise ValueError("odom_frame_mismatch")
            ns = stamp_ns(message.header.stamp)
            age = (self.get_clock().now().nanoseconds-ns)/1e9
            if not -.05 <= age <= .3:
                raise ValueError("stale_odometry")
            p, q = message.pose.pose.position, message.pose.pose.orientation
            rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in (p.x,p.y,p.z)) or abs(rotation[2,2]-1) > .001:
                raise ValueError("nonplanar_or_invalid_odometry")
            if self.poses and ns < self.poses[-1][0]:
                self.poses.clear()
                self.pending = None
                raise ValueError("odometry_time_reversed")
            if not self.poses or ns > self.poses[-1][0]:
                self.poses.append((ns, [p.x,p.y,math.atan2(rotation[1,0],rotation[0,0])]))
        except (ValueError, TypeError, AttributeError) as exc:
            self.poses.clear()
            self.pending = None
            self.report(False,str(exc))

    def cloud(self, message):
        try:
            cloud = read_timed_cloud(message)
            if cloud.stamp_ns <= self.last_ns:
                return  # Repeated delivery cannot renew the source deadline.
            self.last_ns, self.received = cloud.stamp_ns, time.monotonic()
            age = (self.get_clock().now().nanoseconds-cloud.stamp_ns)/1e9
            if not -.05 <= age <= .3:
                raise ValueError("stale_cloud_input")
            if self.mode == "inspect":
                self.pending = None
                self.report(False,"source_timing_mode_required", layout_valid=True, points=len(cloud.xyz),
                    source_frame=cloud.frame_id, source_age_ms=age*1000,
                    acquisition_span_ms=float(np.ptp(cloud.offsets_sec)*1000))
                return
            self.pending = (message, cloud)
            self.valid = False
            self.retry()
        except (ValueError, TypeError, AttributeError) as exc:
            self.pending = None
            self.report(False,str(exc))

    def retry(self):
        if self.pending is None:
            return
        message, cloud = self.pending
        ns = self.get_clock().now().nanoseconds
        if not -.05 <= (ns-cloud.stamp_ns)/1e9 <= .3 or time.monotonic()-self.received > .3:
            self.pending = None
            self.report(False,"cloud_expired_waiting_for_odometry_or_tf")
            return
        try:
            transform = self.buffer.lookup_transform(self.base,cloud.frame_id,
                Time(nanoseconds=cloud.stamp_ns,clock_type=self.get_clock().clock_type))
            q,t = transform.transform.rotation,transform.transform.translation
            matrix = np.eye(4)
            matrix[:3,:3] = quaternion_matrix(q.x,q.y,q.z,q.w)
            matrix[:3,3] = [t.x,t.y,t.z]
            stamps = np.array([item[0] for item in self.poses],dtype=np.int64)
            poses = np.array([item[1] for item in self.poses],dtype=float)
            points = deskew_returns(cloud,stamps,poses,matrix)
            age = (self.get_clock().now().nanoseconds-cloud.stamp_ns)/1e9
            if age > .3 or time.monotonic()-self.received > .3:
                raise ValueError("cloud_expired_during_deskew")
            output = PointCloud2()
            # Own a new Header; never mutate the received message/frame.
            from std_msgs.msg import Header
            output.header = Header(stamp=message.header.stamp,frame_id=self.base)
            output.height,output.width = 1,len(points)
            output.fields = [PointField(name=name,offset=offset,datatype=7,count=1)
                             for name,offset in (("x",0),("y",4),("z",8))]
            output.is_bigendian,output.is_dense = False,True
            output.point_step,output.row_step = 12,len(points)*12
            coordinates = np.asarray(points,dtype="<f4")
            if not np.isfinite(coordinates).all():
                raise ValueError("transformed_points_not_representable")
            output.data = array("B",coordinates.tobytes())
            self.output.publish(output)
            self.pending = None
            self.report(True,"deskewed_observed_returns_only",points=len(points),source_age_ms=age*1000,
                acquisition_span_ms=float(np.ptp(cloud.offsets_sec)*1000),
                oldest_return_age_ms=age*1000-float(cloud.offsets_sec.min()*1000))
        except TransformException:
            self.report(False,"waiting_for_capture_sensor_tf")
        except ValueError as exc:
            # A slightly late odometry sample may complete the exact bracket.
            # Retry this one cloud only; never extrapolate or use latest TF.
            if "bracket" in str(exc) or "trajectory" in str(exc):
                self.report(False,"waiting_for_bracketing_odometry")
            else:
                self.pending = None
                self.report(False,str(exc))

    def health(self):
        ns = self.get_clock().now().nanoseconds
        if not self.last_ns or not -.05 <= (ns-self.last_ns)/1e9 <= .3 or time.monotonic()-self.received > .3:
            self.valid = False
            self.report(False,"cloud_timeout")


def main(args=None):
    rclpy.init(args=args)
    node = Vlp16Observer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
