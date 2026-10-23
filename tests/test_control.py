"""Closed-loop kinematic tracking and rejection cases for metric control."""

from dataclasses import replace
import math
import unittest

import numpy as np

from husky_drivable_area.control import CommandLimiter, ControllerConfig, Demand, fresh_stamp, pursuit, quaternion_matrix, validate_path


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.config = ControllerConfig()
        self.straight = np.column_stack([np.linspace(0, 5, 51), np.zeros(51)])

    def test_straight_center_and_turn_direction(self):
        demand = pursuit(self.straight, [0, 0, 0], self.config)
        self.assertEqual(demand.reason, "tracking")
        self.assertAlmostEqual(demand.linear, 0.2)
        self.assertAlmostEqual(demand.angular, 0)
        self.assertLess(pursuit(self.straight, [0, 0.4, 0], self.config).angular, 0)
        self.assertGreater(pursuit(self.straight, [0, -0.4, 0], self.config).angular, 0)

    def test_fixed_frame_rotation_preserves_body_commands(self):
        pose = np.array([0.3, -0.2, 0.2])
        baseline = pursuit(self.straight, pose)
        angle = 0.9
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        offset = np.array([3, -2])
        path = self.straight @ rotation.T + offset
        transformed_pose = np.r_[pose[:2] @ rotation.T + offset, pose[2] + angle]
        result = pursuit(path, transformed_pose)
        np.testing.assert_allclose([result.linear, result.angular], [baseline.linear, baseline.angular], atol=1e-12)

    def test_faults_goal_and_reverse_target_produce_zero(self):
        for path, pose, reason in (([], [0, 0, 0], "invalid_path"),
                                  (self.straight, [0, 2, 0], "too_far_from_path"),
                                  (self.straight, [5, 0, 0], "goal_reached"),
                                  (self.straight, [0, 0, math.pi], "target_not_ahead"),
                                  (self.straight, [0, float("nan"), 0], "invalid_pose"),
                                  (self.straight, [1e308, 1e308, 0], "invalid_pose")):
            demand = pursuit(path, pose)
            self.assertEqual(demand.reason, reason)
            self.assertEqual((demand.linear, demand.angular), (0, 0))
        limiter = CommandLimiter(self.config)
        for _ in range(50):
            limiter.step(Demand(0.2, 0.4, "tracking"), 0.05)
        self.assertEqual(limiter.step(Demand(reason="stale_path"), 0.05), (0, 0))

    def test_limits_keep_curvature_and_acceleration_bounded(self):
        config = replace(self.config, lookahead_m=0.2)
        demand = pursuit(self.straight, [0, 0.5, 0], config)
        self.assertLessEqual(abs(demand.angular), config.max_angular_speed_rps + 1e-12)
        self.assertLess(demand.linear, config.cruise_speed_mps)
        limiter = CommandLimiter(config)
        previous = (0, 0)
        for _ in range(50):
            v, w = limiter.step(demand, 0.05)
            self.assertLessEqual(v - previous[0], config.max_acceleration_mps2 * 0.05 + 1e-12)
            self.assertLessEqual(abs(w - previous[1]), config.max_angular_acceleration_rps2 * 0.05 + 1e-12)
            previous = (v, w)
        self.assertEqual(limiter.step(Demand(float("nan"), 1, "tracking"), 0.05), (0, 0))

    def test_freshness_requires_source_and_receipt_clock(self):
        self.assertTrue(fresh_stamp(1_000_000_000, 10, 1_100_000_000, 10.1, 0.3))
        self.assertFalse(fresh_stamp(0, 10, 1_100_000_000, 10.1, 0.3))
        self.assertFalse(fresh_stamp(1_000_000_000, 10, 1_400_000_000, 10.1, 0.3))
        self.assertFalse(fresh_stamp(1_000_000_000, 10, 1_100_000_000, 10.4, 0.3))
        self.assertFalse(fresh_stamp(2_000_000_000, 10, 1_100_000_000, 10.1, 0.3))

    def test_paths_and_quaternions_reject_invalid_values(self):
        for points in ([[0, 0], [1, 0]], [[0, 0], [float("inf"), 0]], [[0, 0], [0, 0]], [[-1e308, 0], [1e308, 0]]):
            with self.assertRaises(ValueError):
                validate_path(points, 0.5)
        for quaternion in ((0, 0, 0, 0), (float("nan"), 0, 0, 1), (0, 0, 0, 2)):
            with self.assertRaises(ValueError):
                quaternion_matrix(*quaternion)
        rotation = quaternion_matrix(0, 0, math.sin(0.3), math.cos(0.3))
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
        with self.assertRaises(ValueError):
            ControllerConfig(cruise_speed_mps=-1)

    def test_closed_loop_curve_converges_and_stops_at_goal(self):
        x = np.linspace(0, 8, 81)
        path = np.column_stack([x, 0.7 * np.sin(x * 0.6)])
        pose = np.array([0.0, 0.45, 0.0])
        limiter = CommandLimiter(self.config)
        reached = False
        history = []
        for _ in range(2200):  # 110 virtual seconds; no wall-clock sleeps.
            demand = pursuit(path, pose, self.config)
            self.assertIn(demand.reason, ("tracking", "goal_reached"))
            v, w = limiter.step(demand, 0.05)
            history.append(demand.cross_track_m)
            pose[0] += v * math.cos(pose[2] + w * 0.025) * 0.05
            pose[1] += v * math.sin(pose[2] + w * 0.025) * 0.05
            pose[2] += w * 0.05
            if demand.reason == "goal_reached":
                self.assertEqual((v, w), (0, 0))
                reached = True
                break
        self.assertTrue(reached)
        self.assertLess(np.linalg.norm(pose[:2] - path[-1]), self.config.goal_tolerance_m)
        self.assertLess(max(history[300:]), 0.12)


if __name__ == "__main__":
    unittest.main()
