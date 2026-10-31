import unittest

import numpy as np

from road_navigation import DetectorConfig, RoadDetector
from road_navigation.reference_geometry import build_reference_geometry, estimate_boundaries
from road_navigation.visualization import draw_result


def trapezoid():
    mask = np.zeros((240, 320), np.uint8)
    for y in range(72, 240):
        left = round(100 - 0.25 * (y - 72))
        right = round(220 + 0.25 * (y - 72))
        mask[y, left:right + 1] = 255
    return mask


class BoundaryTests(unittest.TestCase):
    def test_straight_edges_follow_a_perspective_corridor(self):
        left, right = estimate_boundaries(trapezoid())
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        for y in (110, 170, 230):
            self.assertAlmostEqual(left.x_at(y), 100 - .25 * (y - 72), delta=1)
            self.assertAlmostEqual(right.x_at(y), 220 + .25 * (y - 72), delta=1)

    def test_internal_obstacle_does_not_become_an_outer_road_edge(self):
        mask = trapezoid()
        expected = estimate_boundaries(mask)
        mask[120:180, 140:180] = 0
        before = mask.copy()
        observed = estimate_boundaries(mask)
        for a, b in zip(expected, observed):
            self.assertAlmostEqual(a.slope, b.slope)
            self.assertAlmostEqual(a.intercept, b.intercept)
        np.testing.assert_array_equal(mask, before)

    def test_side_outside_image_remains_unknown(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[72:, :250] = 255
        left, right = estimate_boundaries(mask)
        self.assertIsNone(left)
        self.assertIsNotNone(right)
        self.assertAlmostEqual(right.x_at(180), 249)

    def test_full_width_area_does_not_create_two_fake_border_edges(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[72:] = 255
        self.assertEqual(estimate_boundaries(mask), (None, None))

    def test_short_mask_leak_does_not_pull_the_fitted_edge(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[72:, 60:260] = 255
        mask[130:140, 10:60] = 255
        left, right = estimate_boundaries(mask)
        self.assertAlmostEqual(left.x_at(180), 60, delta=1)
        self.assertAlmostEqual(right.x_at(180), 259, delta=1)


class RobotReferenceTests(unittest.TestCase):
    def test_signed_offset_compares_reference_to_robot_at_the_same_row(self):
        mask = trapezoid()
        path = np.array([(210, y) for y in range(225, 71, -1)], np.int32)
        reference = build_reference_geometry(mask, path)
        self.assertEqual(reference.target_x, 210)
        self.assertEqual(reference.robot_x, 160)
        self.assertEqual(reference.lateral_error_px, 50)
        shifted_axis = build_reference_geometry(mask, path, robot_center_x_fraction=.75)
        self.assertLess(shifted_axis.lateral_error_px, 0)

    def test_short_path_does_not_extrapolate_a_lookahead_target(self):
        path = np.array([(160, y) for y in range(225, 179, -1)], np.int32)
        reference = build_reference_geometry(trapezoid(), path)
        self.assertIsNone(reference.target_x)
        self.assertIsNone(reference.lateral_error_px)

    def test_robot_axis_is_independent_of_seed_and_visible_without_a_path(self):
        cfg = DetectorConfig(seed_x_fraction=.4, robot_center_x_fraction=.6)
        frame = np.full((240, 320, 3), 120, np.uint8)
        result = RoadDetector(cfg).process(frame, drivable_probability=np.zeros((240, 320)))
        self.assertFalse(result.valid_image_path)
        self.assertEqual(result.reference_geometry.robot_x, 191)
        self.assertIsNone(result.reference_geometry.lateral_error_px)
        image = draw_result(result)
        blue, green, red = image[230, 191]
        self.assertGreater(int(blue), int(green) + 100)
        self.assertGreater(int(blue), int(red) + 100)
        metadata = result.metadata()["reference_geometry"]
        self.assertEqual(metadata["robot_x"], 191)
        self.assertIsNone(metadata["left"])

    def test_drawing_guides_does_not_modify_path_or_masks(self):
        frame = np.full((240, 320, 3), 120, np.uint8)
        mask = trapezoid()
        mask[120:175, 140:180] = 0
        result = RoadDetector().process(frame, drivable_probability=mask / 255)
        fields = ("frame_bgr", "road_mask", "navigable_mask", "probability", "path")
        before = {name: getattr(result, name).copy() for name in fields}
        draw_result(result)
        for name, values in before.items():
            np.testing.assert_array_equal(getattr(result, name), values)

    def test_guide_config_rejects_coordinates_outside_the_image(self):
        for values in ({"robot_center_x_fraction": -0.1}, {"lookahead_y_fraction": 1.1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                DetectorConfig(**values)


if __name__ == "__main__":
    unittest.main()
