"""Check metric geometry against known rays/world points, not perception output."""

import io
from pathlib import Path
import tarfile
import tempfile
import unittest

import cv2
import numpy as np
import yaml

from husky_drivable_area.calibration import import_calibration
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, project_path


def camera_mapping():
    return {"image_width": 640, "image_height": 480, "distortion_model": "plumb_bob",
            "camera_matrix": {"data": [500, 0, 319.5, 0, 500, 239.5, 0, 0, 1]},
            "distortion_coefficients": {"data": [0, 0, 0, 0, 0]}}


def mount_mapping(**overrides):
    return {"height_m": 1.0, "pitch_down_deg": 30.0, "roll_deg": 0.0, "yaw_deg": 0.0,
            "camera_x_m": 0.0, "camera_y_m": 0.0, "ground_frame": "vision_ground",
            "camera_frame": "camera_optical_frame", **overrides}


class GroundGeometryTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration.from_mapping(camera_mapping())
        self.mount = CameraMount.from_mapping(mount_mapping())

    def project(self, pixels, **kwargs):
        return project_path(pixels, [480, 640], [480, 640], self.camera, self.mount, **kwargs)

    def test_center_ray_and_left_right_signs(self):
        points, reason = self.project([[319.5, 400], [319.5, 239.5]])
        self.assertEqual(reason, "insufficient_ground_path:segment_gap_clipped")
        self.assertEqual(len(points), 0)
        points, reason = self.project([[319.5, 400], [319.5, 239.5]], max_segment_m=5)
        self.assertEqual(reason, "ok")
        np.testing.assert_allclose(points[-1], [np.sqrt(3), 0, 0], atol=1e-9)
        left, _ = self.project([[219.5, 400], [219.5, 390]])
        right, _ = self.project([[419.5, 400], [419.5, 390]])
        self.assertTrue((left[:, 1] > 0).all())
        np.testing.assert_allclose(left[:, 0], right[:, 0])
        np.testing.assert_allclose(left[:, 1], -right[:, 1])

    def test_resize_half_pixel_centers_preserve_metric_locations(self):
        full = np.array([[319.5, 400], [319.5, 390]])
        expected, _ = self.project(full)
        resized = (full + 0.5) / 2 - 0.5
        actual, _ = project_path(resized, [240, 320], [480, 640], self.camera, self.mount)
        np.testing.assert_allclose(actual, expected, atol=1e-10)

    def test_distortion_pose_and_offsets_recover_known_ground_points(self):
        mapping = camera_mapping()
        mapping["distortion_coefficients"]["data"] = [0.08, -0.02, 0.001, -0.002, 0.003]
        camera = CameraCalibration.from_mapping(mapping)
        mount = CameraMount.from_mapping(mount_mapping(height_m=0.6, pitch_down_deg=20,
            roll_deg=3, yaw_deg=8, camera_x_m=0.15, camera_y_m=-0.04))
        ground = np.array([[1.1, 0.1, 0], [1.3, 0.12, 0], [1.6, 0.16, 0]])
        camera_points = (ground - mount.origin) @ mount.rotation
        image, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3), camera.matrix, camera.distortion)
        actual, reason = project_path(image.reshape(-1, 2), [480, 640], [480, 640], camera, mount)
        self.assertEqual(reason, "ok")
        np.testing.assert_allclose(actual, ground, atol=1e-7)

    def test_tf_quaternion_matches_projection_rotation(self):
        for angles in ((0, 0, 0), (4, 23, -34), (-10, 60, 177)):
            mount = CameraMount.from_mapping(mount_mapping(roll_deg=angles[0], pitch_down_deg=angles[1], yaw_deg=angles[2]))
            x, y, z, w = mount.quaternion_xyzw
            self.assertAlmostEqual(x*x+y*y+z*z+w*w, 1)
            matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                               [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                               [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            np.testing.assert_allclose(matrix, mount.rotation, atol=1e-12)

    def test_rear_camera_preserves_contiguous_path_across_robot_origin(self):
        mount = CameraMount.from_mapping(mount_mapping(
            height_m=0.99, pitch_down_deg=67.5,
            camera_x_m=-0.28, camera_y_m=0.21))
        ground = np.array([[-0.15, 0.21, 0], [-0.05, 0.21, 0],
                           [0.05, 0.21, 0], [0.20, 0.21, 0]])
        camera_points = (ground - mount.origin) @ mount.rotation
        image, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3),
                                     self.camera.matrix, self.camera.distortion)
        actual, reason = project_path(image.reshape(-1, 2), [480, 640],
                                     [480, 640], self.camera, mount)
        self.assertEqual(reason, "ok")
        np.testing.assert_allclose(actual, ground, atol=1e-9)

    def test_rear_camera_does_not_bridge_a_metric_gap(self):
        mount = CameraMount.from_mapping(mount_mapping(
            height_m=0.99, pitch_down_deg=67.5,
            camera_x_m=-0.28, camera_y_m=0.21))
        ground = np.array([[-0.15, 0.21, 0], [-0.05, 0.21, 0],
                           [0.55, 0.21, 0]])
        camera_points = (ground - mount.origin) @ mount.rotation
        image, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3),
                                     self.camera.matrix, self.camera.distortion)
        actual, reason = project_path(image.reshape(-1, 2), [480, 640],
                                     [480, 640], self.camera, mount)
        self.assertEqual(reason, "segment_gap_clipped")
        np.testing.assert_allclose(actual, ground[:2], atol=1e-9)

    def test_horizon_and_range_keep_only_contiguous_prefix(self):
        path = [[319.5, v] for v in range(450, 0, -1)]
        points, reason = self.project(path, max_distance_m=2)
        self.assertEqual(reason, "range_clipped")
        self.assertGreater(len(points), 2)
        self.assertTrue((np.linalg.norm(points[:, :2], axis=1) <= 2).all())
        # An invalid first ray must not be skipped to invent a later path.
        level = CameraMount.from_mapping(mount_mapping(pitch_down_deg=0))
        points, reason = project_path([[319.5, 239.5], [319.5, 230]], [480, 640], [480, 640], self.camera, level)
        self.assertEqual(len(points), 0)
        self.assertIn("horizon_clipped", reason)

    def test_resolution_coordinates_and_nonfinite_values_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "resolution"):
            project_path([[10, 20], [10, 18]], [240, 320], [720, 1280], self.camera, self.mount)
        for pixels in ([[320, 400], [320, 410]], [[640, 400], [320, 390]], [[320, float("nan")], [320, 390]]):
            with self.assertRaises(ValueError):
                self.project(pixels)
        with self.assertRaises(ValueError):
            self.project([[320, 400], [320, 390]], max_distance_m=float("inf"))

    def test_placeholder_mount_and_unsupported_lens_are_rejected(self):
        for overrides in ({"height_m": None}, {"height_m": 0}, {"pitch_down_deg": 90},
                          {"pitch_down_deg": float("nan")}, {"camera_frame": "vision_ground"}):
            with self.assertRaises(ValueError):
                CameraMount.from_mapping(mount_mapping(**overrides))
        camera = camera_mapping()
        camera["distortion_model"] = "equidistant"
        with self.assertRaisesRegex(ValueError, "pinhole"):
            CameraCalibration.from_mapping(camera)

    def test_archive_import_validates_and_preserves_previous_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            archive = root / "calibration.tar.gz"
            content = yaml.safe_dump(camera_mapping()).encode()
            with tarfile.open(archive, "w:gz") as target:
                member = tarfile.TarInfo("ost.yaml")
                member.size = len(content)
                target.addfile(member, io.BytesIO(content))
                # Extraction must not follow arbitrary extra archive paths.
                member = tarfile.TarInfo("../../unwanted")
                member.size = 1
                target.addfile(member, io.BytesIO(b"x"))
            output = root / "calibration/camera.yaml"
            result = import_calibration(archive, output)
            self.assertEqual((result.width, result.height), (640, 480))
            self.assertEqual(output.read_bytes(), content)
            output.write_text("prior camera calibration")
            with self.assertRaises(FileExistsError):
                import_calibration(archive, output)
            self.assertEqual(output.read_text(), "prior camera calibration")


if __name__ == "__main__":
    unittest.main()
