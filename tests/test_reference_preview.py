"""Display overload may skip frames, but cannot queue old work or alter pixels."""
import time
import unittest
from unittest import mock

import numpy as np

from road_navigation import DetectorConfig, RoadDetector
from husky_drivable_area.reference_preview import ReferencePreview


class ReferenceWorkerTests(unittest.TestCase):
    def setUp(self):
        self.config=DetectorConfig(use_temporal=False,max_width=160,analysis_width=160,clearance_px=2)
        self.frame=np.full((120,160,3),120,np.uint8)
        self.probability=np.ones((120,160),np.float32)
        self.worker=ReferencePreview(self.config,max_fps=10)
        self.addCleanup(self.worker.close)

    def wait_result(self,key,seconds=4):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            answer=self.worker.take()
            if answer is not None and answer[1]==key:
                return answer
            time.sleep(.01)
        self.fail("display worker did not return "+str(key)+"; "+str(self.worker.stats()))

    def test_display_result_matches_unchanged_detector_and_original_image(self):
        key=("camera",10,123)
        self.worker.submit(key,self.frame,self.probability,17.)
        _,actual_key,result,error=self.wait_result(key)
        self.assertIsNone(error)
        expected=RoadDetector(self.config).process(self.frame,drivable_probability=self.probability)
        np.testing.assert_array_equal(result.frame_bgr,expected.frame_bgr)
        np.testing.assert_array_equal(result.road_mask,expected.road_mask)
        np.testing.assert_array_equal(result.path,expected.path)
        self.assertEqual(result.reason,expected.reason)
        self.assertEqual(actual_key,key)

    def test_overload_replaces_waiting_jobs_instead_of_accumulating_a_backlog(self):
        started=time.monotonic()
        for n in range(40):
            self.worker.submit(("camera",10,n),self.frame,self.probability,0.)
        # Submission includes one spawn but never waits for pixel path planning.
        self.assertLess(time.monotonic()-started,1.)
        self.wait_result(("camera",10,39))
        self.assertGreater(self.worker.stats()["replaced"],20)
        self.assertLess(self.worker.stats()["completed"],10)

    def test_reset_rejects_in_flight_old_camera_generation_and_recovers(self):
        self.worker.submit(("old_camera",10,0),self.frame,self.probability,0.)
        self.worker.reset()
        self.worker.submit(("new_camera",11,0),self.frame,self.probability,0.)
        answer=self.wait_result(("new_camera",11,0))
        self.assertIsNone(answer[3])
        self.assertEqual(answer[0],1)

    def test_worker_failure_is_reported_and_next_valid_frame_can_recover(self):
        self.worker.submit(("camera",10,0),self.frame,np.zeros((16,16)),0.)
        answer=self.wait_result(("camera",10,0))
        self.assertIsNone(answer[2])
        self.assertIn("ValueError",answer[3])
        self.worker.submit(("camera",11,0),self.frame,self.probability,0.)
        self.assertIsNone(self.wait_result(("camera",11,0))[3])

    def test_unused_worker_is_lazy_and_close_stops_the_owned_process(self):
        self.assertIsNone(self.worker._process)
        self.worker.submit(("camera",10,0),self.frame,self.probability,0.)
        self.worker.close()
        self.assertFalse(self.worker._process.is_alive())


class AsyncPreviewAdapterTests(unittest.TestCase):
    def fixture(self):
        from test_ros_adapter import CallbackTests,image_message
        CallbackTests.setUpClass();fixture=CallbackTests();fixture.setUp()
        node=fixture.node
        node._async_reference_preview=True
        node._detector=RoadDetector(DetectorConfig(use_temporal=False,generate_reference_path=False))
        node._reference_worker=mock.Mock()
        node._reference_worker.stats.return_value={}
        node._segmentation_pub=mock.Mock()
        node._segmentation_pub.get_subscription_count.return_value=1
        node._latest.put(image_message())
        return fixture

    def test_nav2_packet_does_not_wait_for_display_and_main_never_runs_pixel_planning(self):
        fixture=self.fixture();node=fixture.node;events=[]
        node._segmentation_pub.publish.side_effect=lambda _:events.append("segmentation")
        node._reference_worker.submit.side_effect=lambda *args:events.append("display_submit")
        with mock.patch("road_navigation.detector.plan_path",side_effect=AssertionError("pixel planning in inference")):
            node._process_latest()
        self.assertEqual(events,["segmentation","display_submit"])
        node._overlay_pub.publish.assert_not_called()
        node._reference_worker.submit.assert_called_once()

    def test_no_viewer_means_no_display_job_but_segmentation_still_publishes(self):
        fixture=self.fixture();node=fixture.node
        node._overlay_pub.get_subscription_count.return_value=0
        node._process_latest()
        node._reference_worker.submit.assert_not_called()
        node._segmentation_pub.publish.assert_called_once()

    def test_camera_change_discards_in_flight_display_generation(self):
        from test_ros_adapter import header
        fixture=self.fixture();node=fixture.node
        node._last_processed_header=header(frame_id="previous_camera")
        node._process_latest()
        node._reference_worker.reset.assert_called_once()
        node._reference_worker.submit.assert_called_once()

    def test_slow_inference_can_update_display_but_never_creates_a_fresh_nav2_packet(self):
        import json
        fixture=self.fixture();node=fixture.node
        with mock.patch.object(fixture.module,"frame_age_seconds",side_effect=[.1,.8,.8]):
            node._process_latest()
        packet=json.loads(node._segmentation_pub.publish.call_args.args[0].data)
        self.assertFalse(packet["valid"])
        self.assertEqual(packet["reason"],"stale_segmentation_before_preview")
        node._reference_worker.submit.assert_called_once()

    def test_rendered_yellow_reference_is_not_published_as_a_control_reference(self):
        from test_ros_adapter import header
        fixture=self.fixture();node=fixture.node;node._axis_geometry=None
        result=RoadDetector(DetectorConfig(use_temporal=False)).process(
            np.full((240,320,3),120,np.uint8),drivable_probability=np.ones((240,320),np.float32))
        self.assertTrue(result.valid_image_path)
        node._reference_worker.take.return_value=(0,("camera_optical_frame",10,0),result,None)
        node._publish_reference_preview()
        self.assertEqual(node._overlay_pub.publish.call_args.args[0].header.stamp.sec,10)
        node._reference_pub.publish.assert_not_called()
        node._segmentation_pub.publish.assert_not_called()
