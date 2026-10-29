"""Camera offset changes the geometric start, never moves it around a veto."""
from dataclasses import replace
import unittest

import numpy as np

from road_navigation import DetectorConfig,RoadDetector
from husky_drivable_area.ground_geometry import CameraCalibration,CameraMount
from husky_drivable_area.ground_overlay import reference_seed
from test_ground_geometry import camera_mapping,mount_mapping


class CalibratedReferenceSeedTests(unittest.TestCase):
    def setUp(self):
        self.camera=CameraCalibration.from_mapping(camera_mapping())
        self.mount=CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,
            camera_x_m=-.26,camera_y_m=.21))
        self.config=DetectorConfig(use_temporal=False)

    def seed_config(self,config=None):
        config=config or self.config
        info=reference_seed(self.camera,self.mount,config)
        return info,replace(config,seed_x_fraction=info["seed_x_fraction"])

    def test_left_camera_uses_right_shifted_robot_axis_at_same_start_row(self):
        info,config=self.seed_config()
        self.assertGreater(info["seed_xy"][0],370)
        self.assertEqual(info["seed_xy"][1],round(479*self.config.seed_y_fraction))
        self.assertEqual(config.seed_y_fraction,self.config.seed_y_fraction)
        self.assertEqual(config.clearance_px,self.config.clearance_px)
        self.assertEqual(info["source"],"calibrated_robot_axis")

    def test_veto_at_camera_center_does_not_block_a_clear_actual_robot_axis(self):
        info,config=self.seed_config()
        probability=np.ones((480,640),np.float32)
        probability[435:467,306:334]=0.
        frame=np.full((480,640,3),120,np.uint8)
        old=RoadDetector(self.config).process(frame,drivable_probability=probability)
        actual=RoadDetector(config).process(frame,drivable_probability=probability)
        self.assertEqual(old.reason,"start_blocked_or_too_narrow")
        self.assertTrue(actual.valid_image_path,actual.reason)
        self.assertEqual(list(actual.path[0]),info["seed_xy"])
        self.assertFalse(actual.road_mask[435:467,306:334].any())
        self.assertTrue((actual.navigable_mask[actual.path[:,1],actual.path[:,0]]>0).all())

    def test_veto_at_true_robot_axis_still_rejects_instead_of_relocating_start(self):
        info,config=self.seed_config();x,y=info["seed_xy"]
        probability=np.ones((480,640),np.float32);probability[y-10:y+11,x-10:x+11]=0.
        actual=RoadDetector(config).process(np.full((480,640,3),120,np.uint8),drivable_probability=probability)
        self.assertEqual(actual.reason,"start_blocked_or_too_narrow")
        self.assertFalse(actual.valid_image_path)

    def test_projection_respects_processed_resolution_and_visibility(self):
        full,_=self.seed_config()
        half,_=self.seed_config(replace(self.config,max_width=320))
        self.assertAlmostEqual((full["seed_xy"][0]+.5)/2-.5,half["seed_xy"][0],delta=1.)
        self.mount=CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,camera_y_m=10.))
        with self.assertRaisesRegex(ValueError,"not_visible"):
            self.seed_config()
