"""Camera route display uses an existing plan and real capture-time geometry."""
import unittest
import time
from types import SimpleNamespace as NS
from unittest import mock
import cv2
import numpy as np

from husky_drivable_area.nav2_camera_path import project_nav2_path, draw_nav2_plan
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, project_path
from test_ground_geometry import camera_mapping, mount_mapping


class Nav2CameraPathTests(unittest.TestCase):
    def setUp(self):
        self.camera=CameraCalibration.from_mapping(camera_mapping())
        self.mount=CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,
            camera_x_m=-.26,camera_y_m=.21))
        self.path=np.column_stack([np.arange(.8,3.01,.05),np.full(45,.1)])

    def project(self,points=None,tf=None):
        return project_nav2_path(self.path if points is None else points,
            np.eye(4) if tf is None else tf,self.camera,self.mount)

    def test_visible_curve_reprojects_to_the_existing_route(self):
        pixels=self.project()
        pixels=pixels[np.isfinite(pixels).all(axis=1)]
        self.assertGreater(len(pixels),20)
        # Inverse projection expects near-to-far pixels and contiguous segments.
        points,reason=project_path(pixels,[480,640],[480,640],self.camera,self.mount,
                                  max_distance_m=10,max_segment_m=.1)
        self.assertEqual(reason,"ok")
        np.testing.assert_allclose(points[:,1],.1,atol=1e-5)
        self.assertGreaterEqual(points[:,0].min(),.8-1e-5)
        self.assertLessEqual(points[:,0].max(),3.01)

    def test_capture_transform_changes_coordinates_without_assuming_zero_robot_pose(self):
        tf=np.eye(4);tf[:2,:2]=[[0,-1],[1,0]];tf[:2,3]=[10.,-3.]
        # Construct the same route expressed in a different map coordinate frame.
        mapped=(self.path-tf[:2,3])@tf[:2,:2]
        actual=self.project(mapped,tf)
        np.testing.assert_allclose(actual,self.project(),atol=1e-7,equal_nan=True)

    def test_route_is_not_connected_to_robot_or_extended_to_fake_goal(self):
        behind=np.column_stack([np.arange(-2,2.01,.05),np.full(81,.15)])
        pixels=self.project(behind)
        # Closest projection starts at x=0, never adds a connector to y=0.
        visible=pixels[np.isfinite(pixels).all(axis=1)]
        ground,reason=project_path(visible,[480,640],[480,640],self.camera,self.mount,
                                   max_distance_m=10,max_segment_m=.1)
        self.assertEqual(reason,"ok")
        np.testing.assert_allclose(ground[:,1],.15,atol=1e-5)
        self.assertGreaterEqual(ground[:,0].min(),-1e-5)

    def test_nonplanar_tf_invalid_path_and_segment_gaps_are_rejected(self):
        tf=np.eye(4);tf[2,2]=.5
        with self.assertRaises(ValueError):self.project(tf=tf)
        for path in ([[1,0],[3,0]],[[1,0],[float("nan"),0]],[[1,0]]):
            with self.assertRaises(ValueError):self.project(np.array(path))

    def test_drawing_does_not_bridge_hidden_intervals_or_cover_banner(self):
        image=np.zeros((480,640,3),np.uint8)
        pixels=np.array([[100,150],[100,200],[np.nan,np.nan],[300,200],[300,250]])
        self.assertTrue(draw_nav2_plan(image,pixels,"preview"))
        self.assertTrue(image[170,100,1]>200)
        self.assertTrue(image[220,300,1]>200)
        self.assertFalse(image[200,150:250].any())
        self.assertFalse(draw_nav2_plan(np.zeros_like(image),[],"waiting_for_nav2_plan"))

    def test_strong_distortion_cannot_fold_a_second_route_into_view(self):
        mapping=camera_mapping()
        mapping["camera_matrix"]["data"]=[602.399186,0,332.306618,0,601.196229,248.008322,0,0,1]
        mapping["distortion_coefficients"]["data"]=[-.0777978,.9307534,-.00059086,.00003155,-2.4624453]
        camera=CameraCalibration.from_mapping(mapping)
        path=np.column_stack([np.arange(.1,8.01,.05),np.zeros(159)])
        pixels=project_nav2_path(path,np.eye(4),camera,self.mount)
        valid=np.isfinite(pixels).all(axis=1)
        starts=np.flatnonzero(np.diff(np.r_[False,valid,False].astype(int)))
        self.assertEqual(len(starts),2)


class CameraPathCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from test_ros_adapter import load_node_callbacks
        cls.module=load_node_callbacks()

    def setUp(self):
        self.node=object.__new__(self.module.DrivableAreaNode)
        self.node._route_topic="/husky_nav/goal_plan"
        self.node._nav2_goal_status=({"reason":"path_ready_preview_only","planning_only":True},time.monotonic())
        self.node._nav2_goal_start_ns=9*10**9
        self.node._axis_geometry=(CameraCalibration.from_mapping(camera_mapping()),
            CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,camera_x_m=-.26,camera_y_m=.21)))
        self.header=NS(frame_id="camera_optical_frame",stamp=NS(sec=10,nanosec=123))
        self.node._nav2_path=NS(header=NS(frame_id="map",stamp=NS(sec=9,nanosec=900000000)),poses=[
            NS(header=NS(frame_id="map"),pose=NS(position=NS(x=float(x),y=.1,z=0.))) for x in np.arange(.8,3.01,.05)])
        self.node._route_tf=mock.Mock()
        self.node._route_tf.lookup_transform.return_value=NS(transform=NS(
            rotation=NS(x=0.,y=0.,z=0.,w=1.),translation=NS(x=0.,y=0.,z=.132)))
        self.node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10*10**9+123),clock_type="test_ros_clock")
        self.image=np.zeros((480,640,3),np.uint8)

    def draw(self):
        modules={"rclpy.time":NS(Time=lambda **kwargs:NS(**kwargs)),
                 "tf2_ros":NS(TransformException=RuntimeError)}
        with mock.patch.dict("sys.modules",modules):return self.node._draw_nav2_path(self.image,self.header)

    def test_draw_uses_image_capture_transform_not_latest_or_plan_time(self):
        info=self.draw()
        self.assertTrue(info["drawn"],info)
        args=self.node._route_tf.lookup_transform.call_args.args
        self.assertEqual(args[:2],("vision_ground","map"))
        self.assertEqual(args[2].nanoseconds,10*10**9+123)

    def test_stale_image_goal_heartbeat_and_tf_loss_hide_route(self):
        self.header.stamp.sec=8
        self.assertFalse(self.draw()["drawn"])
        self.node._route_tf.lookup_transform.assert_not_called()
        self.header.stamp.sec=10
        self.node._nav2_goal_status[0]["reason"]="goal_submitted"
        self.assertFalse(self.draw()["drawn"])
        self.node._nav2_goal_status[0]["reason"]="path_ready_preview_only"
        self.node._route_tf.lookup_transform.side_effect=RuntimeError("TF missing")
        info=self.draw();self.assertFalse(info["drawn"])
        self.assertEqual(info["reason"],"waiting_for_capture_tf")
        self.assertIn("TF missing",info["tf_error"])

    def test_capture_tf_retry_draws_only_when_exact_transform_arrives_within_deadline(self):
        self.node._route_tf.lookup_transform.side_effect=RuntimeError("future")
        self.assertFalse(self.draw()["drawn"])
        self.node._pending_route_display=(self.image.copy(),self.header,(480,640))
        self.node._publish_nav2_display=mock.Mock()
        modules={"rclpy.time":NS(Time=lambda **kwargs:NS(**kwargs)),
                 "tf2_ros":NS(TransformException=RuntimeError)}
        with mock.patch.dict("sys.modules",modules):
            self.node._retry_nav2_display()
            self.node._publish_nav2_display.assert_not_called()
            self.node._route_tf.lookup_transform.side_effect=None
            self.node._retry_nav2_display()
        self.assertTrue(self.node._nav2_display_info["drawn"])
        self.node._publish_nav2_display.assert_called_once()
        self.assertIsNone(self.node._pending_route_display)
        for call in self.node._route_tf.lookup_transform.call_args_list:
            self.assertEqual(call.args[2].nanoseconds,10*10**9+123)

    def test_expired_capture_cannot_gain_a_route_on_late_tf(self):
        self.node._pending_route_display=(self.image.copy(),self.header,(480,640))
        self.node._publish_nav2_display=mock.Mock()
        self.node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_600_000_123),clock_type="ros")
        with mock.patch.dict("sys.modules",{"rclpy.time":NS(Time=object),"tf2_ros":NS(TransformException=RuntimeError)}):
            self.node._retry_nav2_display()
        self.assertFalse(self.node._nav2_display_info["drawn"])
        self.node._route_tf.lookup_transform.assert_not_called()
        self.assertIsNone(self.node._pending_route_display)

    def test_invalid_preview_keeps_camera_without_any_cached_route_or_retimestamp(self):
        self.node._camera_preview=(np.full((480,640,3),120,np.uint8),self.header)
        self.node._overlay_pub=mock.Mock()
        self.node._overlay_pub.get_subscription_count.return_value=1
        self.node._compressed_pub=mock.Mock()
        self.node._compressed_pub.get_subscription_count.return_value=0
        self.node._clear_nav2_display(self.header,"stale_result")
        message=self.node._overlay_pub.publish.call_args.args[0]
        from husky_drivable_area.transport import image_to_bgr
        image=image_to_bgr(message)
        np.testing.assert_array_equal(image[88:],120)
        self.assertIs(message.header,self.header)
        self.assertFalse(self.node._nav2_display_info["drawn"])
        self.assertIsNone(self.node._pending_route_display)

    def test_failed_goal_diagnostic_includes_error_and_never_draws_a_plan(self):
        self.node._nav2_goal_status[0].update(reason="goal_failed",error_code=207,error_name="TIMEOUT")
        info=self.draw()
        self.assertFalse(info["drawn"])
        self.assertIn("TIMEOUT",info["reason"])
        self.node._route_tf.lookup_transform.assert_not_called()

    def test_new_goal_cannot_show_old_plan_and_no_path_does_not_invent_line(self):
        self.node._nav2_goal_start_ns=10*10**9
        self.assertEqual(self.draw()["reason"],"waiting_for_new_goal_plan")
        self.node._nav2_goal_start_ns=9*10**9
        self.node._nav2_path.poses=[]
        self.assertFalse(self.draw()["drawn"])

    def test_wrong_camera_cannot_receive_projected_route(self):
        self.header.frame_id="different_camera"
        self.assertEqual(self.draw()["reason"],"camera_frame_mismatch")
        self.node._route_tf.lookup_transform.assert_not_called()

    def test_display_rate_cap_does_not_stop_inference_or_nav2_packets(self):
        from test_ros_adapter import CallbackTests,image_message
        from road_navigation import DetectorConfig,RoadDetector
        CallbackTests.setUpClass()
        fixture=CallbackTests();fixture.setUp()
        node=fixture.node
        node._detector=RoadDetector(DetectorConfig(use_temporal=False,generate_reference_path=False))
        node._segmentation_pub=mock.Mock()
        node._segmentation_pub.get_subscription_count.return_value=1
        node._overlay_max_fps=2.
        node._last_overlay_at=1000.
        with mock.patch.object(fixture.module.time,"perf_counter",return_value=1000.1), \
             mock.patch.object(fixture.module,"draw_result") as draw:
            for _ in range(2):
                node._latest.put(image_message());node._process_latest()
        draw.assert_not_called()
        node._overlay_pub.publish.assert_not_called()
        self.assertEqual(node._predictor.predict_probability.call_count,2)
        self.assertEqual(node._segmentation_pub.publish.call_count,2)

    def test_nav2_mask_is_published_before_optional_display_work(self):
        from test_ros_adapter import CallbackTests,image_message
        CallbackTests.setUpClass();fixture=CallbackTests();fixture.setUp()
        node=fixture.node;events=[]
        node._segmentation_pub=mock.Mock()
        node._segmentation_pub.get_subscription_count.return_value=1
        node._segmentation_pub.publish.side_effect=lambda _:events.append("mask")
        original=fixture.module.draw_result
        def display(*args,**kwargs):
            events.append("display")
            return original(*args,**kwargs)
        node._latest.put(image_message())
        with mock.patch.object(fixture.module,"draw_result",side_effect=display):node._process_latest()
        self.assertEqual(events,["mask","display"])
