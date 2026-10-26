#!/usr/bin/env python
"""ROS1-side telemetry and LOCAL command watchdog; starts disarmed."""
from __future__ import print_function

import threading
import time
import json
import os
import base64
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Joy, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse
from tf2_msgs.msg import TFMessage

from husky_link_core.protocol import CommandWatchdog, PROTOCOL, dumps, joy_permitted, loads, stamp_dict, wire

try:
    monotonic = time.monotonic
except AttributeError:
    from monotonic import monotonic


class RobotGateway(object):
    def __init__(self):
        self.lock = threading.RLock()
        profile_path = rospy.get_param("~profile_path", "")
        if profile_path:
            with open(os.path.expanduser(profile_path)) as source:
                profile = json.load(source)
        else:
            profile = {}
        robot = profile.get("robot", {})
        self.gate = CommandWatchdog(robot.get("base_frame") or rospy.get_param("~base_frame", "base_link"))
        self.allow_output = rospy.get_param("~allow_robot_output", False)
        if type(self.allow_output) is not bool:
            raise ValueError("allow_robot_output must be bool")
        requested = rospy.get_param("~robot_input_topic", "") or profile.get("bridge", {}).get("expected_output_topic") or ""
        if self.allow_output:
            checks = profile.get("field_measurements", {})
            required = {"geometry_checked", "mux_and_base_timeout_checked", "independent_permission_gate_checked"}
            if set(checks) != required or any(checks[key] is not True for key in required):
                raise ValueError("use the measured profile after field geometry, command mux/base watchdog and independent gate validation")
        if self.allow_output and (not requested.startswith("/") or requested.startswith("/husky_vision/")):
            raise ValueError("set the VERIFIED robot velocity-mux input topic before enabling driver output")
        self.output_topic = requested if self.allow_output else "/husky_vision/watchdog_preview"
        self.output = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.state = rospy.Publisher("state", String, queue_size=1)
        self.publishers = dict((kind, rospy.Publisher(kind + "_json", String, queue_size=1, latch=kind == "tf_static"))
                               for kind in ("odom", "scan", "tf", "tf_static", "permission", "image"))
        self.pending, self.transforms, self.static = {}, {}, {}
        self.local_bool = (False, float("-inf"))
        self.local_joy = None
        self.permission_topic = robot.get("ros1_permission_topic")
        self.joy_topic, self.joy_button = robot.get("joy_topic"), robot.get("joy_deadman_button")
        if self.permission_topic and self.joy_topic:
            raise ValueError("choose either robot-local Bool gate or joystick deadman, not both")
        if self.joy_topic and (type(self.joy_button) is not int or self.joy_button < 0):
            raise ValueError("measure joy_deadman_button index before enabling the joystick source")
        if self.allow_output and not (self.permission_topic or self.joy_topic):
            raise ValueError("real driver output requires a robot-local Bool gate or measured joystick deadman source")
        self.subs = [rospy.Subscriber("command", String, self.command, queue_size=1),
                     rospy.Subscriber("/tf", TFMessage, self.tf, queue_size=20),
                     rospy.Subscriber("/tf_static", TFMessage, self.tf_static, queue_size=20)]
        for kind, cls in (("odom", Odometry), ("scan", LaserScan)):
            topic = robot.get(kind + "_topic") or rospy.get_param("~" + kind + "_topic", "")
            if topic:
                self.subs.append(rospy.Subscriber(topic, cls, self.telemetry, callback_args=kind, queue_size=1))
        if self.permission_topic:
            self.subs.append(rospy.Subscriber(self.permission_topic, Bool, self.permission, queue_size=1))
        if self.joy_topic:
            self.subs.append(rospy.Subscriber(self.joy_topic, Joy, self.joy, queue_size=1))
        camera = profile.get("camera", {})
        self.image_max_fps = float(rospy.get_param("~image_max_fps", 10.0))
        self.image_max_age = float(rospy.get_param("~image_max_age_sec", 0.5))
        if (not 1 <= self.image_max_fps <= 30 or not 0 < self.image_max_age <= 2 or
                math.isnan(self.image_max_age) or math.isinf(self.image_max_age)):
            raise ValueError("image_max_fps must be 1..30 and image_max_age_sec 0..2")
        self.last_image_sent = float("-inf")
        if camera.get("source") == "ros1_compressed":
            topic = camera.get("ros1_compressed_topic")
            if not topic or not topic.startswith("/"):
                raise ValueError("set the ROS1 sensor_msgs/CompressedImage topic")
            self.subs.append(rospy.Subscriber(topic, CompressedImage, self.telemetry, callback_args="image",
                                             queue_size=1, buff_size=2*1024*1024, tcp_nodelay=True))
        self.enable = rospy.Service("set_enabled", SetBool, self.set_enabled)
        # The stop deadline must continue even if /clock or ROS time freezes.
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.loop)
        self.thread.daemon = True
        self.thread.start()
        # Image/JSON serialization must not be in the local stop-deadline loop.
        self.telemetry_thread = threading.Thread(target=self.telemetry_loop)
        self.telemetry_thread.daemon = True
        self.telemetry_thread.start()
        rospy.on_shutdown(self.stop)
        rospy.loginfo("Robot gateway DISARMED; output=%s; driver enabled=%s", self.output_topic, self.allow_output)

    def telemetry(self, message, kind):
        with self.lock:
            self.pending[kind] = message

    def permission(self, message):
        # Bool has no source stamp. Record the actual local receipt once;
        # never refresh it or republish an old true value as a heartbeat.
        with self.lock:
            self.local_bool = (message.data, monotonic())
            self.pending["permission"] = (message, stamp_dict(rospy.Time.now().to_nsec()))

    def joy(self, message):
        with self.lock:
            self.local_joy = (message, monotonic())

    def local_permission(self, ros_ns, now):
        if self.joy_topic:
            if self.local_joy is None:
                return False
            message, received = self.local_joy
            source = message.header.stamp.to_nsec()
            return joy_permitted(message.buttons, self.joy_button, source, received, ros_ns, now)
        value, received = self.local_bool
        return bool(self.permission_topic and value and now-received <= 0.3)

    def tf(self, message):
        with self.lock:
            for transform in message.transforms:
                self.transforms[transform.child_frame_id] = transform
            if len(self.transforms) > 1000:
                self.transforms.clear()
                rospy.logerr("Too many dynamic TF children")
            self.pending["tf"] = TFMessage(transforms=list(self.transforms.values()))

    def tf_static(self, message):
        with self.lock:
            for transform in message.transforms:
                self.static[transform.child_frame_id] = transform
            if len(self.static) > 1000:
                self.static.clear()
                rospy.logerr("Too many static TF children")
            self.pending["tf_static"] = TFMessage(transforms=list(self.static.values()))

    def command(self, message):
        with self.lock:
            try:
                packet = loads(message.data)
                self.gate.accept(packet, rospy.Time.now().to_nsec(), monotonic())
            except (ValueError, TypeError, OverflowError, RuntimeError):
                self.gate.disarm("malformed_command", monotonic())

    def set_enabled(self, request):
        with self.lock:
            if request.data and self.allow_output and not self.local_permission(rospy.Time.now().to_nsec(), monotonic()):
                return SetBoolResponse(success=False, message="robot-local permission/deadman must be held and fresh first")
            self.gate.set_enabled(request.data, monotonic())
        return SetBoolResponse(success=True, message="armed; fresh lease/command still required" if request.data else "disarmed")

    def tick(self, _):
        with self.lock:
            ros_ns, now = rospy.Time.now().to_nsec(), monotonic()
            local_allowed = self.local_permission(ros_ns, now)
            if self.joy_topic:
                # A new decision from a timestamp-checked Joy sample. False is
                # emitted continuously when the device/held button is lost.
                self.pending["permission"] = (Bool(data=local_allowed), stamp_dict(ros_ns))
            if self.allow_output and not local_allowed:
                self.gate.disarm("local_permission_lost", now)
            command = self.gate.tick(ros_ns, now)
            if command is not None:
                message = Twist()
                message.linear.x, message.angular.z = command
                self.output.publish(message)
            self.state.publish(String(data=dumps({"protocol": PROTOCOL, "stamp": stamp_dict(ros_ns),
                "armed": self.gate.armed, "lease": self.gate.lease, "reason": self.gate.reason,
                "driver_output_enabled": self.allow_output, "output_topic": self.output_topic,
                "base_frame": self.gate.base_frame, "max_linear_mps": self.gate.max_linear,
                "max_angular_rps": self.gate.max_angular, "command_timeout_sec": self.gate.timeout})))

    def flush_telemetry(self):
        with self.lock:
            pending, self.pending = self.pending, {}
            if "image" in pending and monotonic()-self.last_image_sent < 1.0/self.image_max_fps:
                # The callback overwrites this slot with a newer image. Never
                # build a FIFO of camera frames while limiting transport rate.
                self.pending["image"] = pending.pop("image")
        for kind, message in pending.items():
            try:
                received_stamp = None
                if kind == "permission":
                    message, received_stamp = message
                if kind == "image":
                    age = (rospy.Time.now().to_nsec()-message.header.stamp.to_nsec())/1e9
                    if not -0.05 <= age <= self.image_max_age:
                        raise ValueError("stale_camera_input")
                    if len(message.data) > 1048576:
                        raise ValueError("camera_payload_too_large")
                    value = {"header": wire(message.header), "format": message.format,
                             "data_b64": base64.b64encode(bytearray(message.data)).decode("ascii")}
                else:
                    value = wire(message)
                packet = {"protocol": PROTOCOL, "kind": kind, "msg": value}
                if received_stamp is not None:
                    packet["received_stamp"] = received_stamp
                self.publishers[kind].publish(String(data=dumps(packet)))
                if kind == "image":
                    self.last_image_sent = monotonic()
            except (ValueError, TypeError, OverflowError) as exc:
                rospy.logwarn_throttle(2.0, "Telemetry packet rejected: " + kind + " (" + str(exc) + ")")

    def loop(self):
        while not self.stopping.is_set() and not rospy.is_shutdown():
            self.tick(None)
            self.stopping.wait(0.05)

    def telemetry_loop(self):
        while not self.stopping.is_set() and not rospy.is_shutdown():
            self.flush_telemetry()
            self.stopping.wait(0.025)

    def stop(self):
        self.stopping.set()
        with self.lock:
            self.gate.disarm("shutdown", monotonic())
            self.output.publish(Twist())


def main():
    rospy.init_node("robot_gateway")
    RobotGateway()
    rospy.spin()


if __name__ == "__main__":
    main()
