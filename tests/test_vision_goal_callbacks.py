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
from husky_drivable_area.vision_goal import VisionReferenceConfig
from husky_drivable_area.vision_local import LocalLidarGrid, LocalFusionConfig
from test_vision_local import scan as local_scan


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

    def test_reference_validity_is_from_actual_tracking_and_keeps_capture_stamp(self):
        n=self.node;source=n.source_ns
        n.tick()
        status=json.loads(n.status_pub.publish.call_args.args[0].data)
        self.assertTrue(status['valid']);self.assertEqual(status['source_stamp_ns'],source)
        self.now+=500_000_000
        n.pose=(n.pose[0],self.now,time.monotonic())
        n.tick()
        status=json.loads(n.status_pub.publish.call_args.args[0].data)
        self.assertFalse(status['valid']);self.assertEqual(status['source_stamp_ns'],source)
        self.assertEqual(status['reason'],'image_reference_stale')
        self.assertEqual(status['linear_x'],0.)
        n.cancel(None,NS());n.tick()
        self.assertFalse(json.loads(n.status_pub.publish.call_args.args[0].data)['valid'])

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

    def prepare_local(self):
        n=self.node;n.local_fusion=True;n.route=None;n.reference=None;n.start_x=1.
        n.config=VisionReferenceConfig();n.local_config=LocalFusionConfig()
        n.local_scan=(LocalLidarGrid(local_scan(),np.eye(4)),np.eye(4),10*10**9,time.monotonic())
        n.local_grid_pub=mock.Mock();n.local_grid_pub.get_subscription_count.return_value=0
        n.scan_pending=None;n.last_scan_ns=0;n.measured_motion=(0.,0.)
        n.pending=(10*10**9,np.ones((480,640),np.uint8),time.monotonic())
        return n

    def test_local_reference_and_command_do_not_require_global_plan(self):
        n=self.prepare_local()
        with mock.patch.object(self.module,'sample_ground_labels',side_effect=lambda p,*a:np.where(np.abs(p[:,1])<1.2,1,0).astype(np.uint8)):
            n.retry()
        self.assertIsNotNone(n.reference);self.assertIsNone(n.route)
        n.new_route(NS(poses=[]))  # Failed global planner cannot erase this mode's path.
        self.assertIsNotNone(n.reference)
        n.tick();status=json.loads(n.status_pub.publish.call_args.args[0].data)
        self.assertTrue(status['valid']);self.assertEqual(status['map_use'],'goal_position_only')
        self.assertEqual(status['lidar_source_stamp_ns'],10*10**9)
        self.assertGreater(status['linear_x'],0.)

    def test_image_only_plans_direct_goal_without_route_or_lidar_and_ignores_scan_callbacks(self):
        n=self.node;n.local_fusion=False;n.image_only=True;n.guidance_mode='image_only'
        n.supervised_trial=True;n.route=None;n.reference=None;n.start_x=1.;n.config=VisionReferenceConfig()
        n.pending=(10*10**9,np.ones((480,640),np.uint8),time.monotonic())
        n.live_local_scan=mock.Mock(side_effect=AssertionError('image_only must not request LiDAR'))
        n.local_command_reason=mock.Mock(side_effect=AssertionError('image_only must not check a scan sweep'))
        with mock.patch.object(self.module,'sample_ground_labels',side_effect=lambda p,*_:np.ones(len(p),np.uint8)), \
             mock.patch.object(self.module,'plan_vision_reference',wraps=self.module.plan_vision_reference) as planner:
            n.retry()
        self.assertTrue(planner.call_args.kwargs['goal_priority'])
        self.assertIsNotNone(n.reference)
        reference=n.reference
        n.new_route(NS(poses=[]));n.laser_scan(object())
        self.assertIs(n.reference,reference)
        n.tick();status=json.loads(n.status_pub.publish.call_args.args[0].data)
        self.assertTrue(status['valid']);self.assertGreater(status['linear_x'],0.)
        self.assertEqual(status['guidance_mode'],'image_only')
        self.assertEqual(status['map_use'],'goal_position_only')
        self.assertFalse(status['lidar_obstacle_checks_enabled'])
        self.assertFalse(status['lidar_stop_sweep_required'])
        n.live_local_scan.assert_not_called();n.local_command_reason.assert_not_called()

    def test_image_only_still_stops_for_image_veto_and_old_reference(self):
        n=self.node;n.local_fusion=False;n.image_only=True;n.supervised_trial=True
        n.route=None;n.start_x=1.;n.config=VisionReferenceConfig()
        n.pending=(10*10**9,np.ones((480,640),np.uint8),time.monotonic())
        with mock.patch.object(self.module,'sample_ground_labels',side_effect=lambda p,*_:np.zeros(len(p),np.uint8)):
            n.retry()
        self.assertIsNone(n.reference);self.assertEqual(n.reason,'start_blocked_narrow_or_unobserved')
        n.tick();self.assertEqual(n.command_pub.publish.call_args.args[0].twist.linear.x,0.)
        n.reference=np.column_stack([np.linspace(1,3,41),np.zeros(41)])
        n.source_ns=self.now-600_000_000
        n.tick();self.assertEqual(n.command_pub.publish.call_args.args[0].twist.linear.x,0.)
        self.assertEqual(json.loads(n.status_pub.publish.call_args.args[0].data)['reason'],'image_reference_stale')

    def test_current_mask_can_revalidate_a_fresh_reference_when_candidate_search_fails(self):
        n=self.node;n.local_fusion=False;n.image_only=True;n.supervised_trial=True
        n.route=None;n.start_x=1.;n.config=VisionReferenceConfig()
        n.reference=np.column_stack([np.linspace(1,2.5,61),np.zeros(61)])
        n.pending=(10_050_000_000,np.ones((480,640),np.uint8),time.monotonic())
        with mock.patch.object(self.module,'sample_ground_labels',side_effect=lambda p,*_:np.ones(len(p),np.uint8)), \
             mock.patch.object(self.module,'plan_vision_reference',return_value=(np.empty((0,2)),'no_observed_image_corridor',{})):
            n.retry()
        self.assertIsNotNone(n.reference);self.assertEqual(n.source_ns,10_050_000_000)
        self.assertTrue(n.detail['reference_reused_after_current_mask_check'])
        n.tick();self.assertTrue(json.loads(n.status_pub.publish.call_args.args[0].data)['valid'])
        n.pending=(10_060_000_000,np.ones((480,640),np.uint8),time.monotonic())
        with mock.patch.object(self.module,'sample_ground_labels',side_effect=lambda p,*_:np.zeros(len(p),np.uint8)), \
             mock.patch.object(self.module,'plan_vision_reference',return_value=(np.empty((0,2)),'no_observed_image_corridor',{})):
            n.retry()
        self.assertIsNone(n.reference)

    def test_no_goal_cannot_reuse_reference_or_invent_a_route(self):
        n=self.node;n.image_only=True;n.goal=None
        n.pending=(10_050_000_000,np.ones((480,640),np.uint8),time.monotonic())
        n.retry()
        self.assertIsNone(n.reference);self.assertEqual(n.reason,'waiting_for_waypoint_or_start')

    def test_image_only_steers_toward_side_goal_at_junction(self):
        n=self.node;n.image_only=True;n.local_fusion=False;n.supervised_trial=True
        n.route=None;n.start_x=.8;n.config=VisionReferenceConfig()
        n.goal.pose.position.x=0.;n.goal.pose.position.y=3.
        n.pending=(10*10**9,np.ones((480,640),np.uint8),time.monotonic())
        with mock.patch.object(self.module,'sample_ground_labels',side_effect=lambda p,*_:np.ones(len(p),np.uint8)):
            n.retry()
        self.assertIsNotNone(n.reference)
        self.assertEqual(n.detail['reference_kind'],'junction_arc')
        demand=n.demand()
        self.assertEqual(demand.reason,'tracking');self.assertGreater(demand.angular,0.)

    def test_new_near_obstacle_stops_an_existing_local_reference_immediately(self):
        n=self.prepare_local();n.reference=np.column_stack([np.linspace(1,3,41),np.zeros(41)])
        n.tick();self.assertGreater(n.command_pub.publish.call_args.args[0].twist.linear.x,0.)
        n.local_scan=(LocalLidarGrid(local_scan([(.85,0,.15)]),np.eye(4)),np.eye(4),10*10**9,time.monotonic())
        n.tick();status=json.loads(n.status_pub.publish.call_args.args[0].data)
        self.assertFalse(status['valid']);self.assertEqual(status['linear_x'],0.)
        self.assertEqual(status['reason'],'local_lidar_stop_sweep_blocked_or_unknown')

    def test_fresh_image_does_not_override_stale_or_missing_lidar(self):
        n=self.prepare_local();n.reference=np.column_stack([np.linspace(1,3,41),np.zeros(41)])
        self.now=10_400_000_000;n.pose=(n.pose[0],self.now,time.monotonic())
        n.tick();status=json.loads(n.status_pub.publish.call_args.args[0].data)
        self.assertEqual(status['reason'],'local_lidar_stale');self.assertEqual(status['linear_x'],0.)
        n.local_scan=None
        self.assertEqual(n.demand().reason,'local_lidar_missing')

    def test_local_scan_tf_retry_retains_original_stamp_and_expires(self):
        n=self.prepare_local();n.local_scan=None
        def delayed(target,source,ns):
            if ns>self.now-90_000_000:raise RuntimeError('future extrapolation')
            return np.eye(4)
        n.tf_matrix.side_effect=delayed
        message=local_scan();message.header=NS(frame_id='laser',stamp=NS(sec=10,nanosec=50_000_000))
        n.laser_scan(message);self.assertIsNone(n.local_scan);self.assertIsNotNone(n.scan_pending)
        self.now=10_160_000_000;n.retry_lidar()
        self.assertIsNotNone(n.local_scan);self.assertEqual(n.local_scan[2],10_050_000_000)
        self.assertTrue(all(c.args[2]==10_050_000_000 for c in n.tf_matrix.call_args_list))
        self.now=10_400_000_000
        with self.assertRaisesRegex(ValueError,'stale'):n.live_local_scan()


if __name__=="__main__":
    unittest.main()
