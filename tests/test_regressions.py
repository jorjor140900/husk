import unittest

import cv2
import numpy as np

from road_navigation import DetectorConfig, RoadDetector
from road_navigation.planner import plan_path


class PerceptionRegressionTests(unittest.TestCase):
    def test_gray_column_with_visible_edges_is_not_filled_as_gray_floor(self):
        frame = np.full((360, 640, 3), 120, np.uint8)
        cv2.rectangle(frame, (290, 0), (350, 250), (30, 30, 30), -1)
        cv2.rectangle(frame, (294, 0), (346, 246), (130, 130, 130), -1)
        result = RoadDetector().process(frame)
        self.assertEqual(result.road_mask[200, 320], 0)
        self.assertGreater(result.road_mask[310, 200], 0)
        self.assertGreater(result.road_mask[310, 440], 0)
        if result.valid_image_path:
            x, y = result.path.T
            self.assertFalse(np.any((x >= 290) & (x <= 350) & (y <= 250)))

    def test_unresolved_full_width_edge_does_not_get_filled_to_make_a_route(self):
        frame = np.full((240, 320, 3), 120, np.uint8)
        frame[192:198] = 10
        result = RoadDetector().process(frame)
        self.assertEqual(result.road_mask[180, 160], 0)
        self.assertFalse(result.valid_image_path)

    def test_gradual_shadow_stays_connected_with_default_barriers(self):
        frame = np.full((240, 320, 3), 120, np.uint8)
        for y in range(100, 220):
            brightness = int(np.interp(y, [100, 140, 180, 219], [120, 40, 40, 120]))
            frame[y] = brightness
        result = RoadDetector().process(frame)
        self.assertGreater(result.road_mask[160, 160], 0)
        self.assertGreater(result.road_mask[90, 160], 0)
        self.assertTrue(result.valid_image_path, result.reason)

    def test_visible_column_edge_still_vetoes_previous_floor_history(self):
        detector = RoadDetector(DetectorConfig(temporal_alpha=.1))
        floor = np.full((360, 640, 3), 120, np.uint8)
        detector.process(floor)
        cv2.rectangle(floor, (290, 0), (350, 250), (30, 30, 30), -1)
        cv2.rectangle(floor, (294, 0), (346, 246), (130, 130, 130), -1)
        result = detector.process(floor)
        self.assertEqual(result.road_mask[200, 320], 0)
        self.assertEqual(result.probability[200, 320], 0)
        self.assertFalse(result.metadata()["quality_score_is_safety_confidence"])


class ReferenceRegressionTests(unittest.TestCase):
    def test_kinked_corridor_is_rejected_instead_of_cutting_its_corners(self):
        mask = np.zeros((240, 320), np.uint8)
        ys = np.arange(72, 240)
        centers = np.rint(np.interp(
            ys, [72, 95, 125, 155, 185, 225, 239],
            [160, 180, 135, 180, 135, 160, 160],
        )).astype(int)
        for y, center in zip(ys, centers):
            mask[y, center - 8:center + 9] = 255
        # There is a connected, row-by-row route, but following its repeated
        # sharp bends would be misleading reference guidance. Do not smooth
        # across the blocked corners just to display a path.
        self.assertTrue(np.all(np.abs(np.diff(centers)) <= 2))
        self.assertTrue(np.all(mask[ys, centers] > 0))
        plan = plan_path(mask, mask / 255., clearance_px=0)
        self.assertFalse(plan.valid)
        self.assertEqual(plan.reason, "no_smooth_reference")

    def test_reference_stays_centered_instead_of_chasing_a_high_color_score(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[72:, 30:290] = 255
        probability = np.full(mask.shape, .65, np.float32)
        probability[:, 215:270] = 1.0
        plan = plan_path(mask, probability)
        self.assertTrue(plan.valid)
        self.assertLessEqual(int(np.abs(plan.points[:, 0] - 160).max()), 2)

    def test_alternating_floor_color_scores_do_not_make_a_zigzag_reference(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[72:, 30:290] = 255
        probability = np.full(mask.shape, .65, np.float32)
        for y in range(72, 240):
            start = 60 if (y // 12) % 2 else 210
            probability[y, start:start + 45] = 1.0
        plan = plan_path(mask, probability)
        self.assertTrue(plan.valid)
        self.assertLessEqual(int(np.abs(plan.points[:, 0] - 160).max()), 2)


if __name__ == "__main__":
    unittest.main()
