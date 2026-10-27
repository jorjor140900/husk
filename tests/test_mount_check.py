import unittest

import numpy as np

from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, ground_to_image
from husky_drivable_area.mount_check import check_floor_points
from test_ground_geometry import camera_mapping, mount_mapping


class MountCheckTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration.from_mapping(camera_mapping())
        self.mount = CameraMount.from_mapping(mount_mapping(height_m=.99,
            pitch_down_deg=18.16, roll_deg=-.47, yaw_deg=1.53,
            camera_x_m=-.26, camera_y_m=.21))
        self.world = np.array([[1., 0.], [1.5, 0.], [1.5, .2]])

    def test_recovers_independent_ground_distances_and_side_offset(self):
        pixels, _ = ground_to_image(self.world, self.camera, self.mount)
        report = check_floor_points(self.camera, self.mount, pixels, self.world)
        np.testing.assert_allclose(report["estimated_xy_m"], self.world, atol=1e-9)
        np.testing.assert_allclose(report["error_xy_cm"], 0, atol=1e-7)
        self.assertFalse(report["mount_changed"])
        self.assertFalse(report["ready_for_control"])

    def test_reports_measured_errors_without_refitting(self):
        actual = self.world + [0.08, -.02]
        pixels, _ = ground_to_image(actual, self.camera, self.mount)
        report = check_floor_points(self.camera, self.mount, pixels, self.world)
        np.testing.assert_allclose(report["error_xy_cm"], [[8, -2]]*3, atol=1e-7)

    def test_rejects_bad_points_and_horizon(self):
        for pixels in ([[np.nan, 1]]*3, [[-1, 20]]*3, [[1000, 1000]]*3, [[1, 2]]):
            with self.assertRaises(ValueError):
                check_floor_points(self.camera, self.mount, pixels, self.world)
        level = CameraMount.from_mapping(mount_mapping(pitch_down_deg=0))
        with self.assertRaisesRegex(ValueError, "intersect"):
            check_floor_points(self.camera, level, [[319.5, 239.5]], [[1., 0.]])
