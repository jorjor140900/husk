"""ROS1 gateway stop loop remains independent of blocked image serialization.

ROS message/publisher stubs isolate scheduling and run the real lease watchdog.
Actual Noetic transport is tested separately; this is not a physical stop test.
"""
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest import mock

from husky_link_core.protocol import CommandWatchdog, loads, stamp_dict, wire


class Stamp:
    __slots__ = ("secs", "nsecs")
    def __init__(self, ns): self.secs, self.nsecs = divmod(ns, 1_000_000_000)
    def to_nsec(self): return self.secs*1_000_000_000+self.nsecs


class Header:
    __slots__ = ("stamp", "frame_id")
    def __init__(self, ns): self.stamp, self.frame_id = Stamp(ns), "camera"


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class Twist:
    def __init__(self): self.linear, self.angular = NS(x=0.), NS(z=0.)


class GatewayMediaTests(unittest.TestCase):
    def setUp(self):
        self.clock = [2., 10_000_000_000]
        ros = NS(Time=NS(now=lambda: Stamp(self.clock[1])), logwarn_throttle=lambda *args: None)
        modules = {"rospy": ros}
        for name, types in {
            "geometry_msgs": {"Twist": Twist}, "nav_msgs": {"Odometry": object},
            "sensor_msgs": {"CompressedImage": object, "Joy": object, "LaserScan": object},
            "std_msgs": {"Bool": lambda data: NS(data=data), "String": lambda data: NS(data=data)},
            "std_srvs": {"SetBool": object, "SetBoolResponse": object}, "tf2_msgs": {"TFMessage": object},
        }.items():
            suffix = ".srv" if name == "std_srvs" else ".msg"
            modules[name], modules[name+suffix] = NS(), NS(**types)
        path = Path(__file__).parents[1] / "ros1/husky_vision_bridge/scripts/robot_gateway.py"
        spec = importlib.util.spec_from_file_location("gateway_media_fixture", path)
        self.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict("sys.modules", modules): spec.loader.exec_module(self.module)
        self.module.monotonic = lambda: self.clock[0]
        self.gateway = g = self.module.RobotGateway.__new__(self.module.RobotGateway)
        g.lock, g.gate = threading.RLock(), CommandWatchdog("base_link")
        g.allow_output, g.joy_topic, g.permission_topic = False, None, None
        g.local_bool, g.local_joy = (False, float("-inf")), None
        g.pending, g.image_max_fps, g.image_max_age = {}, 10., .5
        g.last_image_sent = float("-inf")
        g.output, g.state = Publisher(), Publisher()
        g.output_topic = "/husky_vision/watchdog_preview"
        g.publishers = {"image": Publisher()}

    def image(self, data=b"jpeg", stamp=None):
        return NS(header=Header(self.clock[1] if stamp is None else stamp), format="jpeg", data=data)

    def test_blocked_image_encoding_does_not_delay_watchdog_stop_and_disarm(self):
        g = self.gateway
        g.gate.set_enabled(True, self.clock[0])
        packet = {"protocol": 1, "lease": g.gate.lease, "sequence": 1, "stamp": stamp_dict(self.clock[1]),
                  "base_frame": "base_link", "linear_x": .08, "angular_z": .1}
        self.assertTrue(g.gate.accept(packet, self.clock[1], self.clock[0]))
        g.telemetry(self.image(), "image")
        entered, release = threading.Event(), threading.Event()
        def blocked(value):
            entered.set()
            if not release.wait(2): raise RuntimeError("test did not release encoder")
            return wire(value)
        with mock.patch.object(self.module, "wire", side_effect=blocked):
            worker = threading.Thread(target=g.flush_telemetry)
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.clock[:] = [2.3, 10_300_000_000]
                g.tick(None)
                self.assertTrue(worker.is_alive())
                self.assertFalse(g.gate.armed)
                self.assertEqual(g.gate.reason, "command_timeout")
                self.assertEqual((g.output.messages[-1].linear.x, g.output.messages[-1].angular.z), (0., 0.))
            finally:
                release.set(); worker.join(timeout=2)
        self.assertFalse(worker.is_alive())

    def test_rate_limit_keeps_only_newest_image_and_original_stamp(self):
        g = self.gateway
        g.telemetry(self.image(b"first"), "image"); g.flush_telemetry()
        self.clock[:] = [2.02, 10_020_000_000]
        g.telemetry(self.image(b"second"), "image"); g.flush_telemetry()
        self.clock[:] = [2.04, 10_040_000_000]
        latest = self.image(b"latest")
        g.telemetry(latest, "image"); g.flush_telemetry()
        self.assertIs(g.pending["image"], latest)
        self.assertEqual(len(g.publishers["image"].messages), 1)
        self.clock[:] = [2.11, 10_110_000_000]
        g.flush_telemetry()
        packet = loads(g.publishers["image"].messages[-1].data)
        self.assertEqual(packet["msg"]["header"]["stamp"], stamp_dict(10_040_000_000))
        self.assertEqual(packet["msg"]["data_b64"], "bGF0ZXN0")

    def test_stale_future_or_oversized_images_are_dropped_not_restamped(self):
        g = self.gateway
        for image in (self.image(stamp=9_000_000_000), self.image(stamp=11_000_000_000), self.image(b"x"*1048577)):
            g.telemetry(image, "image"); g.flush_telemetry()
        self.assertEqual(g.publishers["image"].messages, [])


if __name__ == "__main__": unittest.main()
