import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest import mock


class Twist:
    def __init__(self):
        self.linear = NS(x=0.,y=0.,z=0.)
        self.angular = NS(x=0.,y=0.,z=0.)


class NavigationGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import sys
        from test_controller_geometry_preview import ControllerGeometryPreviewTests
        ControllerGeometryPreviewTests.setUpClass()
        base = {
            "rclpy":NS(),"rclpy.node":NS(Node=object),"rclpy.time":NS(Time=object),
            "rclpy.executors":NS(ExternalShutdownException=RuntimeError),
            "rclpy.qos":NS(QoSProfile=object,ReliabilityPolicy=object),
            "geometry_msgs":NS(),"geometry_msgs.msg":NS(Twist=Twist,TwistStamped=object),
            "nav_msgs":NS(),"nav_msgs.msg":NS(Odometry=object),
            "std_msgs":NS(),"std_msgs.msg":NS(Bool=object,String=NS),
            "std_srvs":NS(),"std_srvs.srv":NS(SetBool=object),
            "tf2_ros":NS(Buffer=object,TransformListener=object,TransformException=RuntimeError),
            "husky_drivable_area.controller_node":NS(stamp_ns=lambda s:s.sec*10**9+s.nanosec),
        }
        source = Path(__file__).resolve().parents[1]/"husky_drivable_area/navigation_gate.py"
        spec = importlib.util.spec_from_file_location("husky_drivable_area._nav_gate_test",source)
        cls.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,base):
            spec.loader.exec_module(cls.module)

    def setUp(self):
        self.node = object.__new__(self.module.NavigationGate)
        self.node.allow_motion = self.node.enabled = True
        self.node.base,self.node.odom_frame = "base_link","odom"
        self.header = NS(frame_id="base_link",stamp=NS(sec=10,nanosec=0))
        self.twist = Twist();self.twist.linear.x = .05
        self.node.command = (NS(header=self.header,twist=self.twist),100.)
        self.node.odom = (NS(header=NS(frame_id="odom",stamp=NS(sec=10,nanosec=0)),child_frame_id="base_link"),100.)
        self.node.vision = (dict(valid=True,source_stamp_ns=10*10**9),100.)
        self.node.permission = (True,100.)
        self.node.buffer = mock.Mock()
        self.node.buffer.lookup_transform.return_value = NS(header=NS(stamp=NS(sec=10,nanosec=0)))
        self.node.get_clock = lambda:NS(now=lambda:NS(nanoseconds=10*10**9))
        self.node.output,self.node.status = mock.Mock(),mock.Mock()

    def tick(self):
        with mock.patch.object(self.module.time,"monotonic",return_value=100.):
            self.node.tick()
        return json.loads(self.node.status.publish.call_args.args[0].data)

    def test_planning_only_cannot_be_enabled_even_with_live_inputs(self):
        self.node.allow_motion = False
        response = self.node.enable(NS(data=True),NS())
        self.assertFalse(response.success)
        self.assertFalse(self.node.enabled)
        self.assertEqual(self.tick()["reason"],"planning_only")
        self.assertEqual(self.node.output.publish.call_args.args[0].linear.x,0)

    def test_shutdown_with_invalid_context_skips_publish_but_cleans_up(self):
        runtime = NS(ok=mock.Mock(return_value=False),try_shutdown=mock.Mock())
        node = mock.Mock()
        with mock.patch.object(self.module,"rclpy",runtime):self.module.stop_node(node)
        node.output.publish.assert_not_called()
        node.destroy_node.assert_called_once()
        runtime.try_shutdown.assert_called_once()

    def test_shutdown_race_is_tolerated_and_valid_context_publish_errors_are_not_hidden(self):
        runtime = NS(ok=mock.Mock(side_effect=[True,False]),try_shutdown=mock.Mock())
        node = mock.Mock();node.output.publish.side_effect = RuntimeError("context invalid")
        with mock.patch.object(self.module,"rclpy",runtime):self.module.stop_node(node)
        node.destroy_node.assert_called_once()
        runtime.ok = mock.Mock(return_value=True)
        with mock.patch.object(self.module,"rclpy",runtime):
            with self.assertRaises(RuntimeError):self.module.stop_node(node)

    def test_live_valid_inputs_only_reach_private_collision_request(self):
        status = self.tick()
        self.assertEqual(status["reason"],"request_passed_to_collision_filter")
        self.assertEqual(status["linear_x"],.05)
        self.assertTrue(status["downstream_collision_filter_required"])
        self.assertFalse(status["ready_for_autonomous_control"])

    def test_stale_vision_permission_and_commands_each_stop_the_request(self):
        self.node.permission = (False,100.)
        self.assertEqual(self.tick()["linear_x"],0)
        self.node.permission = (True,100.)
        self.node.vision[0]["source_stamp_ns"] = 9*10**9
        self.assertEqual(self.tick()["reason"],"vision_stale")
        self.node.vision[0]["source_stamp_ns"] = 10*10**9
        self.header.stamp.sec = 9
        self.assertEqual(self.tick()["reason"],"nav2_command_stale")
        self.assertEqual(self.node.output.publish.call_args.args[0].linear.x,0)

    def test_missing_localization_and_reverse_or_excessive_speed_are_blocked(self):
        self.node.buffer.lookup_transform.side_effect = RuntimeError()
        self.assertEqual(self.tick()["reason"],"localization_unavailable")
        self.node.buffer.lookup_transform.side_effect = None
        for speed in (-.05,.1,float("nan")):
            self.twist.linear.x = speed
            self.assertEqual(self.tick()["reason"],"invalid_nav2_command")
            self.assertEqual(self.node.output.publish.call_args.args[0].linear.x,0)
