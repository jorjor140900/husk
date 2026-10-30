"""Goal/image identity, fail-closed source times and cancellation for image steering."""
import importlib.util
from collections import OrderedDict
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS, MethodType
import unittest
from unittest import mock

import numpy as np

from husky_drivable_area.control import ControllerConfig, CommandLimiter


def header(sec=10):
    return NS(frame_id="odom",stamp=NS(sec=sec,nanosec=0))


def twist():
    return NS(header=header(),twist=NS(linear=NS(x=0.),angular=NS(z=0.)))


class VisionGoalCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stubs={"rclpy":NS(),"rclpy.node":NS(Node=object),"rclpy.time":NS(Time=NS),
            "rclpy.qos":NS(QoSProfile=object,ReliabilityPolicy=object,DurabilityPolicy=object),
            "geometry_msgs.msg":NS(PoseStamped=lambda:NS(header=header(),pose=NS(
                position=NS(x=0.,y=0.),orientation=NS(w=1.))),TwistStamped=twist),
            "nav_msgs.msg":NS(Path=lambda:NS(header=header(),poses=[]),Odometry=object),
            "sensor_msgs.msg":NS(Image=object,CompressedImage=object),
            "std_msgs.msg":NS(String=NS),"std_srvs.srv":NS(Trigger=object),
            "tf2_ros":NS(Buffer=object,TransformListener=object,TransformException=RuntimeError),
            "husky_drivable_area.controller_node":NS(stamp_ns=lambda s:s.sec*10**9+s.nanosec)}
        source=Path(__file__).resolve().parents[1]/"husky_drivable_area/vision_goal_node.py"
        spec=importlib.util.spec_from_file_location("husky_drivable_area._vision_goal_test",source)
        cls.module=importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,stubs):spec.loader.exec_module(cls.module)

    def setUp(self):
        self.node=object.__new__(self.module.VisionGoalNode)
        n=self.node;n.base="base_link";n.odom_frame="odom";n.completed=False
        n.goal=NS(header=header(),pose=NS(position=NS(x=3.,y=0.,z=0.)))
        n.goal.header.frame_id="map"
        points=np.column_stack([np.linspace(0,3,31),np.zeros(31)])
        n.route=(NS(header=header()),points,time.monotonic())
        n.reference=points[10:];n.source_ns=10*10**9;n.source_received=time.monotonic()
        n.reason="ok";n.detail={};n.last_display=None;n.pending=None;n.overlay_pending=None
        n.pose=(np.zeros(3),n.source_ns,time.monotonic())
        n.camera=NS(height=480,width=640);n.mount=NS(camera_frame="camera",ground_frame="ground")
        n.path_pub=mock.Mock();n.command_pub=mock.Mock();n.status_pub=mock.Mock()
        n.overlay=mock.Mock();n.tf_matrix=mock.Mock(return_value=np.eye(4))
        n.controller=ControllerConfig(cruise_speed_mps=.05,max_angular_speed_rps=.2,
            max_path_offset_m=1.5,lookahead_m=.5)
        n.limiter=CommandLimiter(n.controller);n.last_tick=time.monotonic()
        self.now=10_100_000_000
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=self.now,to_msg=lambda:header().stamp),clock_type="ros")

    def test_stale_capture_stops_even_while_odometry_and_goal_remain_valid(self):
        self.assertEqual(self.node.demand().reason,"tracking")
        self.node.source_ns-=600_000_000
        self.assertEqual(self.node.demand().reason,"image_reference_stale")
        self.node.tick()
        self.assertEqual(self.node.command_pub.publish.call_args.args[0].twist.linear.x,0.)
        self.assertIsNone(self.node.reference)

    def test_cancel_clears_goal_route_reference_and_publishes_zero(self):
        self.node.last_display=(10,np.ones((16,16),np.uint8))
        result=self.node.cancel(None,NS())
        self.assertTrue(result.success)
        self.assertIsNone(self.node.goal);self.assertIsNone(self.node.route)
        self.assertIsNone(self.node.reference)
        self.assertEqual(self.node.command_pub.publish.call_args.args[0].twist.linear.x,0.)
        self.assertEqual(len(self.node.overlay.call_args.args[2]),0)

    def test_goal_reached_is_latched_until_a_new_goal(self):
        self.node.goal.pose.position.x=.1
        self.node.tick()
        self.assertTrue(self.node.completed)
        self.node.goal.pose.position.x=3.
        self.assertEqual(self.node.demand().reason,"goal_reached")
        self.node.new_goal(self.node.goal)
        self.assertFalse(self.node.completed)
        self.assertIsNone(self.node.reference);self.assertIsNone(self.node.route)

    def test_old_route_for_other_goal_is_not_used(self):
        message=NS(header=NS(frame_id="map",stamp=header().stamp),poses=[
            NS(header=NS(frame_id="map"),pose=NS(position=NS(x=x,y=0.,z=0.))) for x in np.linspace(0,1,11)])
        self.node.new_route(message)
        self.assertIsNone(self.node.route);self.assertIsNone(self.node.reference)

    def test_capture_tf_retry_uses_exact_source_and_expiry_never_publishes_path(self):
        self.node.pending=(10*10**9,np.ones((480,640),np.uint8),time.monotonic())
        self.node.tf_matrix.side_effect=RuntimeError("TF future by 13 ms")
        self.node.retry()
        self.assertIsNotNone(self.node.pending)
        self.assertEqual(self.node.tf_matrix.call_args.args[2],10*10**9)
        self.now+=500_000_000
        self.node.retry()
        self.assertIsNone(self.node.pending)
        self.assertTrue(all(not call.args[0].poses for call in self.node.path_pub.publish.call_args_list))

    def test_streaming_captures_do_not_starve_a_delayed_exact_tf(self):
        n=self.node;n.reference=None;n.start_x=1.;n.config=object()
        labels=np.ones((480,640),np.uint8)
        reference=np.column_stack([np.linspace(1.,2.,11),np.zeros(11)])
        def decode(data):
            ns=int(data)
            return {"source_header":{"frame_id":"camera","stamp":{
                "sec":ns//10**9,"nanosec":ns%10**9}},"raw_shape_hw":[480,640]},labels
        def delayed_tf(target,source,ns):
            if ns>self.now-90_000_000:raise RuntimeError("future extrapolation")
            return np.eye(4)
        n.tf_matrix.side_effect=delayed_tf
        with mock.patch.object(self.module,"decode_segmentation",side_effect=decode), \
             mock.patch.object(self.module,"plan_vision_reference",return_value=(reference,"ok",{})):
            for i in range(30):
                ns=10_000_000_000+i*66_666_667
                self.now=ns+20_000_000
                n.segmentation(NS(data=str(ns)))
                n.retry()
                if i>=2:
                    self.assertIsNotNone(n.reference)
                    self.assertLessEqual(self.now-n.source_ns,500_000_000)
            paths=[c.args[0] for c in n.path_pub.publish.call_args_list if c.args[0].poses]
            self.assertGreater(len(paths),8)
            stamps=[p.header.stamp.sec*10**9+p.header.stamp.nanosec for p in paths]
            captures={10_000_000_000+i*66_666_667 for i in range(30)}
            self.assertTrue(set(stamps)<=captures)
            self.assertEqual(stamps,sorted(set(stamps)))

    def test_tf_wait_preserves_reference_stamp_but_stops_at_original_expiry(self):
        n=self.node;reference=n.reference;source=n.source_ns;received=n.source_received
        n.pending=(10_050_000_000,np.ones((480,640),np.uint8),time.monotonic())
        n.tf_matrix.side_effect=RuntimeError("TF delayed")
        n.retry()
        self.assertIs(n.reference,reference)
        self.assertEqual((n.source_ns,n.source_received),(source,received))
        self.assertTrue(n.detail["capture_tf_waiting"])
        self.now=10_550_000_000
        n.retry()
        self.assertIsNone(n.reference)
        self.assertEqual(n.reason,"waiting_for_capture_tf")
        self.assertEqual(n.command_pub.publish.call_args.args[0].twist.linear.x,0.)

    def test_malformed_packet_clears_old_reference_immediately(self):
        self.node.pending=(10*10**9,np.ones((480,640),np.uint8),time.monotonic())
        self.node.segmentation(NS(data='{"valid": false}'))
        self.assertIsNone(self.node.pending)
        self.assertIsNone(self.node.reference)
        self.assertEqual(self.node.command_pub.publish.call_args.args[0].twist.linear.x,0.)

    def test_current_goal_tf_uses_odometry_stamp_not_wall_now(self):
        self.node.demand()
        self.assertEqual(self.node.tf_matrix.call_args.args[2],self.node.pose[1])

    def test_overlay_waits_for_exact_raw_capture_and_never_uses_another_image(self):
        n=self.node;n.images=OrderedDict();n.overlay_pending=None
        n.overlay_pub=mock.Mock();n.overlay_pub.get_subscription_count.return_value=1
        n.last_seg_ns=10*10**9;n.last_overlay_stamp=10*10**9
        n.overlay=MethodType(self.module.VisionGoalNode.overlay,n)
        labels=np.ones((480,640),np.uint8);points=np.array([[1.,0.],[1.1,0.]])
        frame=np.zeros((480,640,3),np.uint8)
        with mock.patch.object(self.module,"image_to_bgr",return_value=frame), \
             mock.patch.object(self.module,"draw_vision_reference",return_value=frame) as draw, \
             mock.patch.object(self.module,"fill_compressed_image",side_effect=lambda _,im,h,q:NS(header=h)):
            n.overlay(10*10**9,labels,points,"ready")
            self.assertIsNotNone(n.overlay_pending)
            other=NS(header=NS(frame_id="camera",stamp=NS(sec=10,nanosec=20_000_000)),height=480,width=640)
            n.image(other)
            draw.assert_not_called()
            correct=NS(header=NS(frame_id="camera",stamp=NS(sec=10,nanosec=0)),height=480,width=640)
            n.image(correct)
            np.testing.assert_array_equal(draw.call_args.args[2],points)
            self.assertIsNone(n.overlay_pending)
            self.assertIs(n.overlay_pub.publish.call_args.args[0].header,correct.header)


if __name__=="__main__":
    unittest.main()
