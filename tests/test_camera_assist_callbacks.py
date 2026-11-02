"""Held-forward commands must be gated by current camera evidence and input."""
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest import mock
import numpy as np
from husky_drivable_area.control import ControllerConfig
from husky_drivable_area.vision_goal import VisionReferenceConfig


class Twist:
    def __init__(self):
        self.linear=NS(x=0.,y=0.,z=0.);self.angular=NS(x=0.,y=0.,z=0.)


class AssistCallbacksTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stubs={'rclpy':NS(),'rclpy.node':NS(Node=object),'rclpy.time':NS(Time=NS),
            'rclpy.executors':NS(ExternalShutdownException=RuntimeError),
            'rclpy.qos':NS(QoSProfile=object,ReliabilityPolicy=object),
            'geometry_msgs.msg':NS(Twist=Twist),'nav_msgs.msg':NS(Odometry=object),
            'sensor_msgs.msg':NS(Image=object,CompressedImage=object),'std_msgs.msg':NS(String=NS),
            'std_srvs.srv':NS(SetBool=object),'tf2_ros':NS(Buffer=object,TransformListener=object,TransformException=RuntimeError),
            'husky_drivable_area.controller_node':NS(stamp_ns=lambda s:s.sec*10**9+s.nanosec)}
        source=Path(__file__).parents[1]/'husky_drivable_area/camera_assist_node.py'
        spec=importlib.util.spec_from_file_location('husky_drivable_area._assist_test',source)
        cls.module=importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,stubs):spec.loader.exec_module(cls.module)
        source=Path(__file__).parents[1]/'husky_drivable_area/assist_teleop.py'
        spec=importlib.util.spec_from_file_location('husky_drivable_area._assist_keyboard_test',source)
        cls.keyboard=importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,stubs):spec.loader.exec_module(cls.keyboard)

    def setUp(self):
        n=self.node=object.__new__(self.module.CameraAssistNode)
        n.allow_motion=n.enabled=True;n.hold=2.;n.start_x=.8;n.config=VisionReferenceConfig()
        n.source_ns=10*10**9;n.source_received=time.monotonic();n.reason='tracking';n.detail={}
        n.reference=np.column_stack([np.linspace(1,3,41),np.linspace(0,.5,41)])
        n.controller=ControllerConfig(cruise_speed_mps=.15,max_angular_speed_rps=.15,max_path_offset_m=1.5)
        self.now=10_100_000_000
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=self.now))
        n.pose=(np.zeros(3),self.now,time.monotonic());n.teleop=(Twist(),time.monotonic());n.teleop[0].linear.x=.15
        n.output=mock.Mock();n.status=mock.Mock();n.gate=mock.Mock()
        n.pending=None;n.render=mock.Mock();n.tf_matrix=mock.Mock(return_value=np.eye(4))
        n.mount=NS(camera_frame='camera');n.camera=NS(height=480,width=640)

    def test_only_held_forward_produces_ai_steering_and_overrides_manual_angular(self):
        n=self.node;n.teleop[0].angular.z=-.15
        d=n.demand();self.assertEqual(d.reason,'tracking');self.assertGreater(d.angular,0.)
        self.assertGreater(d.linear,0.);self.assertLessEqual(d.linear,.15)
        n.teleop[0].linear.x=0.;n.tick()
        self.assertEqual(n.output.publish.call_args.args[0].linear.x,0.)
        gate=json.loads(n.gate.publish.call_args.args[0].data)
        self.assertTrue(gate['request_allowed'])  # Releasing never latches the session fault.

    def test_planner_detail_is_serialized_without_colliding_with_status_fields(self):
        self.node.detail=dict(red_edges_used_for_planning=False,max_curvature=.4)
        self.node.tick()
        status=json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertEqual(status['reason'],'tracking')
        self.assertFalse(status['red_edges_used_for_planning'])

    def test_input_stale_reverse_and_missing_reference_never_move(self):
        n=self.node;n.teleop=(n.teleop[0],time.monotonic()-.3)
        self.assertEqual(n.demand().linear,0.)
        n.teleop=(n.teleop[0],time.monotonic());n.teleop[0].linear.x=-.15
        self.assertEqual(n.demand().linear,0.)
        n.teleop[0].linear.x=.15;n.reference=None
        self.assertEqual(n.demand().linear,0.)

    def test_enable_drops_old_teleop_input_and_preview_cannot_enable(self):
        n=self.node;n.enable(NS(data=True),NS());self.assertIsNone(n.teleop)
        self.assertEqual(n.demand().linear,0.)
        n.allow_motion=False
        self.assertFalse(n.enable(NS(data=True),NS()).success);self.assertFalse(n.enabled)

    def test_held_reference_slow_until_absolute_expiry_without_renewing_source(self):
        n=self.node;stamp=n.source_ns;receipt=n.source_received
        self.now=10_800_000_000;n.pose=(n.pose[0],self.now,time.monotonic())
        n.segmentation(NS(data=json.dumps(dict(valid=False,reason='stale_result'))))
        self.assertEqual(n.demand().reason,'tracking');self.assertLessEqual(n.demand().linear,.05)
        self.assertEqual((n.source_ns,n.source_received),(stamp,receipt))
        self.now=12_100_000_000;n.pose=(n.pose[0],self.now,time.monotonic())
        self.assertEqual(n.demand().linear,0.);self.assertEqual(n.demand().reason,'image_reference_stale')

    def test_current_non_drivable_mask_clears_even_a_recent_reference(self):
        n=self.node;n.pending=(self.now,np.zeros((480,640),np.uint8),time.monotonic())
        with mock.patch.object(self.module,'sample_ground_labels',side_effect=lambda p,*_:np.zeros(len(p),np.uint8)):
            n.retry()
        self.assertIsNone(n.reference);self.assertEqual(n.output.publish.call_args.args[0].linear.x,0.)

    def test_malformed_packet_never_holds_last_path(self):
        self.node.segmentation(NS(data='not json'))
        self.assertIsNone(self.node.reference)

    def test_keyboard_repeats_expire_and_other_keys_stop(self):
        held=self.keyboard.HeldForward();self.assertFalse(held.active(10.))
        held.key('i',10.);self.assertTrue(held.active(10.5));self.assertFalse(held.active(10.7))
        held.key('w',11.);held.key('k',11.1);self.assertFalse(held.active(11.1))
        held.key('\x1b[A',12.);self.assertTrue(held.active(12.1))


if __name__=='__main__':unittest.main()
