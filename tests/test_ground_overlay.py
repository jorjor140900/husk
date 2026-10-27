import unittest

import numpy as np

from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, project_path
from husky_drivable_area.ground_overlay import robot_axis_pixels
from road_navigation import RoadDetector
from road_navigation.visualization import axis_x_at_row, draw_result
from test_ground_geometry import camera_mapping, mount_mapping


class GroundOverlayTests(unittest.TestCase):
    def setUp(self):
        self.calibration = CameraCalibration.from_mapping(camera_mapping())
        self.mount = CameraMount.from_mapping(mount_mapping(
            height_m=0.99, pitch_down_deg=67.5,
            camera_x_m=-0.28, camera_y_m=0.21))

    def axis(self, shape=(480, 640), mount=None):
        return robot_axis_pixels(self.calibration, mount or self.mount,
                                 shape, (480, 640), "camera_optical_frame")

    def test_left_camera_places_centerline_right_of_image_center_and_reprojects_y_zero(self):
        pixels = self.axis()
        self.assertGreater(axis_x_at_row(pixels, 260), 319.5)
        visible = pixels[np.isfinite(pixels).all(axis=1)]
        ground, reason = project_path(visible, [480, 640], [480, 640],
                                     self.calibration, self.mount, max_distance_m=10)
        self.assertEqual(reason, "ok")
        self.assertGreater(len(ground), 20)
        np.testing.assert_allclose(ground[:, 1], 0, atol=1e-10)

    def test_offset_direction_and_half_pixel_resize(self):
        right = CameraMount.from_mapping(mount_mapping(height_m=0.99,
            pitch_down_deg=67.5, camera_x_m=-0.28, camera_y_m=-0.21))
        self.assertLess(axis_x_at_row(self.axis(mount=right), 260), 319.5)
        full, small = self.axis(), self.axis((240, 320))
        # Compare the same physical row after resizing, including pixel centers.
        full_x = axis_x_at_row(full, 260)
        small_x = axis_x_at_row(small, (260+0.5)/2-0.5)
        self.assertAlmostEqual(small_x, (full_x+0.5)/2-0.5, places=8)

    def test_rear_mount_centerline_reaches_bottom_edge_at_both_resolutions(self):
        for shape in ((480, 640), (240, 320)):
            with self.subTest(shape=shape):
                pixels = self.axis(shape)
                visible = pixels[np.isfinite(pixels).all(axis=1)]
                self.assertGreater(visible[:, 1].max(), shape[0]-2)
                self.assertIsNotNone(axis_x_at_row(pixels, shape[0]-2))
                frame = np.full((*shape, 3), 120, np.uint8)
                result = RoadDetector().process(frame, drivable_probability=np.zeros(shape))
                image = draw_result(result, robot_axis=pixels)
                blue = image[-1, :, 0].astype(int)-image[-1, :, 1].astype(int)
                self.assertTrue((blue > 100).any())

    def test_frame_or_raw_resolution_mismatch_rejected(self):
        for size, frame in (((480, 640), "other_camera"), ((720, 1280), "camera_optical_frame")):
            with self.assertRaises(ValueError):
                robot_axis_pixels(self.calibration, self.mount, (240, 320), size, frame)

    def test_distorted_yawed_rolled_camera_still_recovers_robot_centerline(self):
        mapping = camera_mapping()
        mapping["distortion_coefficients"]["data"] = [0.08, -0.02, 0.001, -0.002, 0.003]
        calibration = CameraCalibration.from_mapping(mapping)
        mount = CameraMount.from_mapping(mount_mapping(height_m=0.99,
            pitch_down_deg=67.5, camera_x_m=-0.28, camera_y_m=0.21,
            roll_deg=3, yaw_deg=8))
        pixels = robot_axis_pixels(calibration, mount, (240, 320), (480, 640),
                                   "camera_optical_frame")
        visible = pixels[np.isfinite(pixels).all(axis=1)]
        ground, reason = project_path(visible, [240, 320], [480, 640],
                                     calibration, mount, max_distance_m=10)
        self.assertEqual(reason, "ok")
        self.assertGreater(len(ground), 20)
        np.testing.assert_allclose(ground[:, 1], 0, atol=1e-5)

    def test_missing_sections_do_not_make_a_false_row_crossing(self):
        self.assertIsNone(axis_x_at_row([[100, 220], [np.nan, np.nan], [200, 100]], 180))
        self.assertIsNone(axis_x_at_row([], 180))

    def test_calibrated_overlay_hides_old_axis_and_empty_axis_has_no_fallback(self):
        frame = np.full((240, 320, 3), 120, np.uint8)
        result = RoadDetector().process(frame, drivable_probability=np.zeros((240, 320)))
        axis = self.axis((240, 320))
        x = int(round(axis_x_at_row(axis, 130)))
        image = draw_result(result, robot_axis=axis)
        self.assertGreater(int(image[130, x, 0]), int(image[130, x, 1])+100)
        np.testing.assert_array_equal(image[130, 160], frame[130, 160])
        empty = draw_result(result, robot_axis=[])
        np.testing.assert_array_equal(empty[100:], frame[100:])


if __name__ == "__main__":
    unittest.main()
