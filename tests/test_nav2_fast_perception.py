"""Nav2's full veto mask must not depend on the diagnostic image path."""
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import cv2
import numpy as np

from road_navigation import DetectorConfig, RoadDetector
from husky_drivable_area.vision_grid import encode_segmentation, decode_segmentation


class Nav2FastPerceptionTests(unittest.TestCase):
    def config(self, fast):
        return DetectorConfig(use_temporal=False,generate_reference_path=not fast,reject_green_vegetation=True)

    def scene(self):
        frame = np.full((240,320,3),140,np.uint8)
        frame[:,:60] = (30,120,40);frame[:,260:] = (30,120,40)
        probability = np.ones(frame.shape[:2],np.float32)
        probability[:,100:110] = 0  # disconnected visible branch must survive
        return frame, probability

    def test_same_full_segmentation_and_veto_with_and_without_pixel_planning(self):
        frame, probability = self.scene()
        normal = RoadDetector(self.config(False)).process(frame,drivable_probability=probability)
        fast = RoadDetector(self.config(True)).process(frame,drivable_probability=probability)
        np.testing.assert_array_equal(fast.raw_mask,normal.raw_mask)
        self.assertTrue(fast.raw_mask[150,80])
        self.assertFalse(fast.raw_mask[150,30])
        self.assertFalse(fast.raw_mask[150,105])
        self.assertFalse(fast.valid_image_path)
        self.assertFalse(fast.navigable_mask.any())
        self.assertFalse(fast.metadata()["image_reference_enabled"])
        self.assertEqual(fast.reason,"nav2_segmentation_only")

    def test_fast_profile_skips_dp_and_optical_flow_without_changing_image_size(self):
        profile=Path(__file__).resolve().parents[1]/"config/outdoor-nav2-fast.json"
        detector=RoadDetector(DetectorConfig(**json.loads(profile.read_text())))
        frame=np.full((480,640,3),140,np.uint8);probability=np.ones(frame.shape[:2])
        with mock.patch("road_navigation.detector.plan_path",side_effect=AssertionError("pixel planner called")), \
             mock.patch.object(cv2,"calcOpticalFlowFarneback",side_effect=AssertionError("flow called")):
            for _ in range(2):result=detector.process(frame,drivable_probability=probability)
        self.assertEqual(result.frame_bgr.shape,(480,640,3))
        self.assertEqual(result.raw_mask.shape,(480,640))
        self.assertTrue(result.vegetation_veto_enabled)

    def test_fast_packet_matches_normal_packet_and_keeps_original_stamp(self):
        frame, probability = self.scene()
        header=NS(frame_id="camera",stamp=NS(sec=10,nanosec=456))
        labels=[]
        for fast in (False,True):
            result=RoadDetector(self.config(fast)).process(frame,drivable_probability=probability)
            packet, decoded=decode_segmentation(encode_segmentation(result,header,.3))
            labels.append(decoded)
            self.assertEqual(packet["source_header"]["stamp"],dict(sec=10,nanosec=456))
        np.testing.assert_array_equal(*labels)

    def test_current_obstacle_and_grass_win_on_following_frame(self):
        detector=RoadDetector(self.config(True))
        frame=np.full((240,320,3),140,np.uint8);probability=np.ones(frame.shape[:2])
        detector.process(frame,drivable_probability=probability)
        frame[:,:80] = (30,120,40)
        blocked=np.zeros(frame.shape[:2],np.uint8);blocked[150:160,100:200]=255
        result=detector.process(frame,drivable_probability=probability,blocked_mask=blocked)
        self.assertFalse(result.raw_mask[:,10:70].any())
        self.assertFalse(result.raw_mask[150:160,100:200].any())

    def test_image_reference_default_is_preserved_and_invalid_flag_rejected(self):
        self.assertTrue(DetectorConfig().generate_reference_path)
        with self.assertRaises(ValueError):DetectorConfig(generate_reference_path=1)
