"""Physical geometry checks for the experimental scan stop filter."""

import math
import unittest

import numpy as np

from husky_drivable_area.collision import CollisionConfig, check_sweep, circle_scan, scan_points, stop_sweep


class CollisionTests(unittest.TestCase):
    def setUp(self):
        self.config = CollisionConfig()

    def scan(self, ranges, **changes):
        fields = dict(angle_min=-math.pi, angle_max=math.pi-2*math.pi/720, angle_increment=2*math.pi/720,
                      range_min=0.05, range_max=8.0, time_increment=0.0)
        fields.update(changes)
        return scan_points(ranges, **fields)

    def check(self, points, requested=(0.2, 0), measured=(0, 0), **kwargs):
        return check_sweep(points, requested, measured, self.config, **kwargs)[0]

    def test_stop_distance_matches_straight_braking_integral(self):
        poses = stop_sweep((0.2, 0), self.config)
        self.assertAlmostEqual(poses[-1, 0], 0.2*0.3 + 0.2**2/(2*0.4))
        np.testing.assert_array_equal(poses[:, 1:], 0)
        self.assertEqual(self.check([[0.68, 0]]), "obstacle_in_sweep")
        self.assertEqual(self.check([[0.8, 0]]), "clear")

    def test_actual_motion_is_checked_even_when_request_is_zero(self):
        self.assertEqual(self.check([[0.75, 0]], requested=(0, 0), measured=(0.25, 0)), "obstacle_in_sweep")
        self.assertEqual(self.check([[0.75, 0]], requested=(0, 0)), "clear")
        self.assertEqual(self.check([[0.75, 0]], requested=(0, 0),
                         additional_velocities=((0.25, 0),)), "obstacle_in_sweep")

    def test_rotation_sweeps_a_side_obstacle_that_straight_motion_misses(self):
        obstacle = [[0.30, 0.53]]
        self.assertEqual(self.check(obstacle), "clear")
        self.assertEqual(self.check(obstacle, requested=(0, 0.6)), "obstacle_in_sweep")
        self.assertEqual(self.check([[0.30, -0.53]], requested=(0, -0.6)), "obstacle_in_sweep")
        self.assertEqual(self.check([[-0.55, 0]], requested=(0, 0)), "obstacle_in_sweep")

    def test_age_expands_the_modeled_stop_corridor(self):
        self.assertEqual(self.check([[0.75, 0]]), "clear")
        self.assertEqual(self.check([[0.75, 0]], extra_reaction_sec=0.3), "obstacle_in_sweep")

    def test_eight_cm_side_margin_preserves_body_and_front_rear_margin(self):
        config=CollisionConfig(front_m=.5,rear_m=.56,half_width_m=.33,margin_m=.15,lateral_margin_m=.08)
        check=lambda points:check_sweep(points,(0,0),(0,0),config)[0]
        self.assertEqual(check([[0,.44]]),"clear")
        self.assertEqual(check([[0,.40]]),"obstacle_in_sweep")
        self.assertEqual(check([[0,.32]]),"obstacle_in_sweep")
        self.assertEqual(check([[.63,0]]),"obstacle_in_sweep")
        self.assertEqual(check([[-.68,0]]),"obstacle_in_sweep")
        np.testing.assert_allclose(config.padded_corners(),[[.66,.42],[.66,-.42],[-.72,-.42],[-.72,.42]])

    def test_old_margin_default_and_invalid_side_margin(self):
        self.assertEqual(self.config.side_margin_m,self.config.margin_m)
        for value in (-.01,float("nan"),float("inf"),True,None):
            with self.assertRaises(ValueError):CollisionConfig(lateral_margin_m=value)

    def test_sensor_range_limits_apply_to_whole_footprint(self):
        self.assertEqual(self.check([], sensor_range_m=0.5), "sweep_outside_scan_range")
        self.assertEqual(self.check([], sensor_range_m=8.0), "clear")
        self.assertEqual(self.check([], sensor_origin_base=(-8, 0)), "sweep_outside_scan_range")

    def test_infinity_is_no_return_but_unknown_ranges_reject_scan(self):
        self.assertEqual(self.scan(np.full(720, np.inf)).shape, (0, 2))
        for invalid in (float("nan"), float("-inf"), 0.0, -1.0, 9.0):
            ranges = np.full(720, np.inf)
            ranges[5] = invalid
            with self.assertRaisesRegex(ValueError, "invalid_scan_ranges"):
                self.scan(ranges)
        with self.assertRaisesRegex(ValueError, "scan_requires_deskew"):
            self.scan(np.full(720, np.inf), time_increment=0.0001)
        with self.assertRaises(ValueError):
            self.scan(np.full(720, np.inf), angle_increment=math.pi/720)

    def test_ray_circle_returns_recover_known_near_surface(self):
        ranges = circle_scan((0, 0, 0), [(3, 0, 0.3)])
        self.assertAlmostEqual(ranges[360], 2.7)
        points = self.scan(ranges)
        self.assertTrue(np.isfinite(points).all())
        self.assertEqual(self.check(points), "clear")
        moved = circle_scan((2.1, 0, 0), [(3, 0, 0.3)])
        self.assertEqual(self.check(self.scan(moved)), "obstacle_in_sweep")
        rotated = circle_scan((0, 0, math.pi/2), [(0, 3, 0.3)])
        self.assertAlmostEqual(rotated[360], 2.7)

    def test_invalid_or_reverse_motion_and_invalid_config_stop(self):
        for velocity in ((-0.1, 0), (0.3, 0), (0, 0.7), (float("nan"), 0)):
            with self.assertRaises(ValueError):
                self.check([], requested=velocity)
        with self.assertRaises(ValueError):
            CollisionConfig(front_m=0)
        with self.assertRaises(ValueError):
            CollisionConfig(sweep_step_m=0.2)


if __name__ == "__main__":
    unittest.main()
