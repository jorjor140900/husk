import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from husky_drivable_area.control import ControllerConfig, CommandLimiter


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0., y=0., z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=0.)


class ControllerGeometryPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {
            "rclpy": SimpleNamespace(), "rclpy.node": SimpleNamespace(Node=object),
            "rclpy.duration": SimpleNamespace(Duration=object),
            "rclpy.executors": SimpleNamespace(ExternalShutdownException=RuntimeError),
            "rclpy.qos": SimpleNamespace(QoSProfile=object, ReliabilityPolicy=object),
            "rclpy.time": SimpleNamespace(Time=object),
            "geometry_msgs": SimpleNamespace(), "geometry_msgs.msg": SimpleNamespace(Twist=Twist),
            "nav_msgs": SimpleNamespace(), "nav_msgs.msg": SimpleNamespace(Odometry=object, Path=object),
            "std_msgs": SimpleNamespace(), "std_msgs.msg": SimpleNamespace(Bool=object, String=SimpleNamespace),
            "std_srvs": SimpleNamespace(), "std_srvs.srv": SimpleNamespace(SetBool=object),
            "tf2_ros": SimpleNamespace(Buffer=object, TransformException=RuntimeError, TransformListener=object),
        }
        source = Path(__file__).resolve().parents[1]/"husky_drivable_area/controller_node.py"
        spec = importlib.util.spec_from_file_location("husky_drivable_area._geometry_preview_test", source)
        cls.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict("sys.modules", modules):
            spec.loader.exec_module(cls.module)

    def setUp(self):
        self.node = object.__new__(self.module.PathControllerNode)
        self.node.config = ControllerConfig(cruise_speed_mps=.05, max_angular_speed_rps=.2)
        self.node.limiter = CommandLimiter(self.node.config)
        x = np.linspace(0., 2., 21)
        self.node.path = np.column_stack([x, .1*x])
        self.node.pose = np.zeros(3)
        self.node.path_source_ns = self.node.odom_source_ns = 10_000_000_000
        self.node.path_received = self.node.odom_received = 100.
        self.node.path_timeout, self.node.odom_timeout, self.node.permission_timeout = .5, .3, .3
        self.node.permission_received, self.node.permitted, self.node.enabled = float("-inf"), False, False
        self.node.path_error = self.node.odom_error = "ok"
        self.node.last_tick = 99.95
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
        self.node.preview_pub, self.node.command_pub, self.node.status_pub = mock.Mock(), mock.Mock(), mock.Mock()

    def tick(self):
        with mock.patch.object(self.module.time, "monotonic", return_value=100.):
            self.node._tick()
        return json.loads(self.node.status_pub.publish.call_args.args[0].data)

    def assert_zero(self, publisher):
        msg = publisher.publish.call_args.args[0]
        self.assertEqual((msg.linear.x, msg.angular.z), (0., 0.))

    def test_disabled_missing_permission_shows_geometry_but_both_twist_outputs_are_zero(self):
        status = self.tick()
        self.assertEqual(status["reason"], "disabled")
        self.assertEqual(status["candidate_reason"], "permission_timeout")
        self.assertEqual(status["geometry_preview"]["reason"], "tracking")
        self.assertGreater(status["geometry_preview"]["linear_x"], 0)
        self.assertTrue(status["geometry_preview"]["for_display_only"])
        self.assert_zero(self.node.command_pub)
        self.assert_zero(self.node.preview_pub)

    def test_enabled_controller_still_blocks_commands_without_permission(self):
        self.node.enabled = True
        status = self.tick()
        self.assertEqual(status["reason"], "permission_timeout")
        self.assertEqual(status["geometry_preview"]["reason"], "tracking")
        self.assert_zero(self.node.command_pub)
        self.node.permission_received = 100.
        self.assertEqual(self.tick()["reason"], "motion_not_permitted")
        self.assert_zero(self.node.command_pub)

    def test_stale_path_and_odometry_also_block_geometry_preview(self):
        self.node.path_source_ns = 9_000_000_000
        status = self.tick()
        self.assertEqual(status["geometry_preview"]["reason"], "stale_path")
        self.assertEqual(status["geometry_preview"]["linear_x"], 0)
        self.node.odom_source_ns = 9_000_000_000
        self.assertEqual(self.tick()["geometry_preview"]["reason"], "stale_odometry")
        self.assert_zero(self.node.command_pub)

    def test_permission_loss_stops_real_commands_despite_valid_preview(self):
        self.node.enabled = self.node.permitted = True
        self.node.permission_received = 100.
        self.assertEqual(self.tick()["reason"], "tracking")
        self.assertGreater(self.node.command_pub.publish.call_args.args[0].linear.x, 0)
        self.node.permitted = False
        status = self.tick()
        self.assertEqual(status["reason"], "motion_not_permitted")
        self.assertEqual(status["geometry_preview"]["reason"], "tracking")
        self.assert_zero(self.node.command_pub)
