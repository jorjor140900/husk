"""Camera metadata to stationary command preview; no ROS or fabricated odometry."""

import copy
import unittest

from husky_drivable_area.control import ControllerConfig
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
from husky_drivable_area.vision_preview import preview_detection


class VisionPreviewTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration.from_mapping({"image_width": 640, "image_height": 480,
            "distortion_model": "plumb_bob", "camera_matrix": {"data": [500, 0, 319.5, 0, 500, 239.5, 0, 0, 1]},
            "distortion_coefficients": {"data": [0, 0, 0, 0, 0]}})
        self.mount = CameraMount.from_mapping({"height_m": 0.6, "pitch_down_deg": 30.0, "roll_deg": 0.0,
            "yaw_deg": 0.0, "camera_x_m": 0.0, "camera_y_m": 0.0,
            "ground_frame": "vision_ground", "camera_frame": "camera_optical_frame"})
        self.data = {"valid_image_path": True, "reason": "ok",
            "source_header": {"frame_id": "camera_optical_frame", "stamp": {"sec": 3, "nanosec": 0}},
            "path_coordinate_system": "processed_image_pixels_xy_near_to_far",
            "path": [[319.5, v] for v in range(450, 199, -1)],
            "processed_shape_hw": [480, 640], "input_shape_hw": [480, 640]}

    def evaluate(self, data=None, now=3_100_000_000):
        return preview_detection(data if data is not None else self.data, self.camera, self.mount, ControllerConfig(), now)

    def test_fresh_center_line_has_forward_candidate_with_explicit_unverified_state(self):
        original = copy.deepcopy(self.data)
        result = self.evaluate()
        self.assertEqual(result.reason, "candidate_available")
        self.assertAlmostEqual(result.demand.linear, 0.2)
        self.assertAlmostEqual(result.demand.angular, 0)
        self.assertGreater(len(result.path), 2)
        self.assertTrue(result.metadata()["preview_only"])
        self.assertFalse(result.metadata()["ready_for_control"])
        self.assertFalse(result.metadata()["footprint_checked"])
        self.assertEqual(original, self.data)

    def test_steering_sign_follows_left_right_projected_references(self):
        self.data["path"] = [[279.5, v] for v in range(450, 199, -1)]
        self.assertGreater(self.evaluate().demand.angular, 0)
        self.data["path"] = [[359.5, v] for v in range(450, 199, -1)]
        self.assertLess(self.evaluate().demand.angular, 0)

    def test_old_and_future_images_never_reuse_candidate_or_path(self):
        for now in (3_600_000_000, 2_900_000_000):
            result = self.evaluate(now=now)
            self.assertEqual(result.reason, "stale_or_invalid_source_timestamp")
            self.assertEqual(len(result.path), 0)
            self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])

    def test_wrong_frame_resolution_and_missing_path_are_explained(self):
        self.data["source_header"]["frame_id"] = "other_camera"
        self.assertEqual(self.evaluate().reason, "source_frame_mismatch")
        self.data["source_header"]["frame_id"] = "camera_optical_frame"
        self.data["input_shape_hw"] = [720, 1280]
        result = self.evaluate()
        self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])
        self.assertIn("resolution", result.detail)
        result = self.evaluate({"valid_image_path": False, "reason": "start_blocked"})
        self.assertEqual(result.reason, "image_path_invalid:start_blocked")

    def test_blind_gap_is_visible_without_relaxing_controller_contract(self):
        self.data["path"] = [[319.5, v] for v in range(250, 199, -1)]
        result = self.evaluate()
        self.assertEqual(result.reason, "controller:too_far_from_path")
        self.assertGreater(len(result.path), 2)
        self.assertGreater(result.metadata()["distance_to_reference_m"], 0.8)
        self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])


if __name__ == "__main__":
    unittest.main()
