import unittest

import numpy as np

from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, ground_to_image
from husky_drivable_area.mount_fit import floor_rectangle, fit_mount
from test_ground_geometry import camera_mapping, mount_mapping


class MountFitTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration.from_mapping(camera_mapping())
        self.mount = CameraMount.from_mapping(mount_mapping(height_m=.99,
            pitch_down_deg=20, yaw_deg=2, roll_deg=-1,
            camera_x_m=-.28, camera_y_m=.21))
        self.world = floor_rectangle(1.27, 1.87, .375, -.225)
        self.pixels, visible = ground_to_image(self.world[:, :2], self.camera, self.mount)
        self.assertTrue(visible.all())

    def test_known_pose_recovered_from_four_measured_points(self):
        old = CameraMount.from_mapping(mount_mapping(pitch_down_deg=67.5))
        report = fit_mount(self.camera, old, self.world, self.pixels)
        candidate = CameraMount.from_mapping(report["candidates"][0]["mount"])
        np.testing.assert_allclose(candidate.origin, self.mount.origin, atol=1e-7)
        np.testing.assert_allclose(candidate.rotation, self.mount.rotation, atol=1e-7)
        self.assertLess(report["candidates"][0]["rmse_px"], 1e-6)
        self.assertFalse(report["verified_geometry"])
        self.assertFalse(report["ready_for_control"])

    def test_distortion_and_noisy_clicks_report_residuals(self):
        mapping = camera_mapping()
        mapping["distortion_coefficients"]["data"] = [.04, -.01, .002, -.003, 0]
        camera = CameraCalibration.from_mapping(mapping)
        pixels, _ = ground_to_image(self.world[:, :2], camera, self.mount)
        pixels += [[.3, -.4], [-.3, .2], [.2, .3], [-.4, -.2]]
        report = fit_mount(camera, self.mount, self.world, pixels)
        self.assertGreater(report["candidates"][0]["rmse_px"], 0)
        self.assertLess(report["candidates"][0]["rmse_px"], 1)
        self.assertEqual(len(report["candidates"][0]["residuals_px"]), 4)

    def test_fixed_origin_recovers_angles_and_preserves_measured_translation(self):
        from dataclasses import asdict
        old = CameraMount.from_mapping({**asdict(self.mount), "pitch_down_deg": 67.5, "yaw_deg": 0., "roll_deg": 0.})
        report = fit_mount(self.camera, old, self.world, self.pixels, fix_origin=True)
        candidate = CameraMount.from_mapping(report["candidates"][0]["mount"])
        np.testing.assert_array_equal(candidate.origin, self.mount.origin)
        np.testing.assert_allclose(candidate.rotation, self.mount.rotation, atol=1e-7)
        self.assertTrue(report["origin_fixed_to_measurements"])
        self.assertLess(report["candidates"][0]["rmse_px"], 1e-6)

    def test_bad_click_order_and_degenerate_points_are_rejected(self):
        for pixels in (self.pixels[[0, 3, 2, 1]], np.zeros((4, 2)),
                       np.full((4, 2), np.nan), self.pixels+1000):
            with self.assertRaises(ValueError):
                fit_mount(self.camera, self.mount, self.world, pixels)
        with self.assertRaises(ValueError):
            fit_mount(self.camera, self.mount, np.zeros((4, 3)), self.pixels)
        with self.assertRaises(ValueError):
            floor_rectangle(2, 1, .375, -.225)


if __name__ == "__main__":
    unittest.main()
