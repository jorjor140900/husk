"""Check floor-marker projection independently of AI and robot dynamics."""

import unittest

import numpy as np

from husky_drivable_area.ground_check import BANNER_HEIGHT, GroundCheckOverlay
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, ground_to_image, project_path


def camera(distortion=None):
    return CameraCalibration.from_mapping({"image_width": 640, "image_height": 480,
        "distortion_model": "plumb_bob", "camera_matrix": {"data": [756.8, 0, 316.6, 0, 755.0, 230.0, 0, 0, 1]},
        "distortion_coefficients": {"data": distortion or [0, 0, 0, 0, 0]}})


def mount(**kwargs):
    return CameraMount.from_mapping({"height_m": 0.775, "pitch_down_deg": 70.0, "roll_deg": 0.0, "yaw_deg": 0.0,
        "camera_x_m": 0.0, "camera_y_m": 0.0, "ground_frame": "vision_ground", "camera_frame": "camera_optical_frame", **kwargs})


class GroundCheckTests(unittest.TestCase):
    def test_downward_webcam_markers_match_analytic_camera_rays(self):
        grid = GroundCheckOverlay(camera(), mount())
        center = [t for t in grid.targets if t["relative_xy_m"][1] == 0]
        self.assertEqual([t["id"] for t in center], ["C010", "C025", "C050"])
        angle, height = np.deg2rad(70), 0.775
        for target in center:
            x = target["relative_xy_m"][0]
            v = 230.0 + 755.0*(height*np.cos(angle)-x*np.sin(angle))/(x*np.cos(angle)+height*np.sin(angle))
            np.testing.assert_allclose(target["raw_pixel_uv"], [316.6, v], atol=1e-9)
        for target in grid.targets:
            if target["id"].startswith("L"):
                self.assertLess(target["raw_pixel_uv"][0], 316.6)
            if target["id"].startswith("R"):
                self.assertGreater(target["raw_pixel_uv"][0], 316.6)

    def test_distorted_rotated_projection_recovers_measured_world_points(self):
        calibration = camera([0.08, -0.02, 0.001, -0.002, 0.003])
        rig = mount(height_m=0.6, pitch_down_deg=20, roll_deg=3, yaw_deg=8, camera_x_m=0.15, camera_y_m=-0.04)
        ground = np.array([[1.3, 0.12], [1.45, 0.14], [1.6, 0.16]])
        pixels, visible = ground_to_image(ground, calibration, rig)
        self.assertTrue(visible.all())
        recovered, reason = project_path(pixels, [480, 640], [480, 640], calibration, rig)
        self.assertEqual(reason, "ok")
        np.testing.assert_allclose(recovered[:, :2], ground, atol=1e-7)

    def test_points_behind_camera_and_nonfinite_points_are_rejected(self):
        _, visible = ground_to_image([[-1, 0], [4, 0]], camera(), mount(pitch_down_deg=0))
        self.assertEqual(visible.tolist(), [False, True])
        for value in ([], [[1, 2, 0]], [[float("nan"), 0]], [[0, float("inf")]]):
            with self.assertRaises(ValueError):
                ground_to_image(value, camera(), mount())

    def test_offsets_keep_grid_origin_below_lens(self):
        original = GroundCheckOverlay(camera(), mount())
        shifted = GroundCheckOverlay(camera(), mount(camera_x_m=0.30, camera_y_m=-0.10))
        self.assertEqual(len(original.targets), len(shifted.targets))
        for first, second in zip(original.targets, shifted.targets):
            np.testing.assert_allclose(first["raw_pixel_uv"], second["raw_pixel_uv"], atol=1e-9)
            np.testing.assert_allclose(np.array(first["ground_xy_m"])+[0.30, -0.10], second["ground_xy_m"])

    def test_overlay_preserves_source_and_labels_expected_cross_locations(self):
        grid = GroundCheckOverlay(camera(), mount())
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        result = grid.draw(image)
        self.assertFalse(image.any())
        self.assertEqual(result.shape, (480+BANNER_HEIGHT, 640, 3))
        target = next(t for t in grid.targets if t["id"] == "C025")
        u, v = np.rint(target["raw_pixel_uv"]).astype(int)
        np.testing.assert_array_equal(result[v+BANNER_HEIGHT, u], [0, 230, 255])
        self.assertFalse(grid.metadata()["ready_for_control"])
        self.assertFalse(grid.metadata()["physical_geometry_verified"])

    def test_overlay_refuses_resolution_changes(self):
        grid = GroundCheckOverlay(camera(), mount())
        with self.assertRaisesRegex(ValueError, "resolution"):
            grid.draw(np.zeros((480, 800, 3), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
