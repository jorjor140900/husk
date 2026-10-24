"""Camera metadata to stationary command preview; no ROS or fabricated odometry."""

import copy
import unittest

from husky_drivable_area.control import ControllerConfig
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
from husky_drivable_area.vision_preview import age_preview, preview_detection


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

    def test_slow_cpu_results_keep_diagnostic_geometry_without_a_candidate(self):
        for age_ms in (655, 1005, 1330, 1671):
            result = self.evaluate(now=3_000_000_000 + age_ms*1_000_000)
            self.assertEqual(result.reason, "stale_for_control")
            self.assertGreater(len(result.path), 2)
            self.assertEqual(result.demand.reason, "tracking")
            self.assertTrue(result.metadata()["valid_image_path"])
            self.assertEqual(result.metadata()["display_state"], "stale_for_control")
            self.assertEqual(result.metadata()["frame_age_ms"], age_ms)
            self.assertFalse(result.metadata()["source_fresh_for_control"])
            self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])

    def test_watchdog_expires_candidate_then_display_with_original_source_stamp(self):
        original = self.evaluate()
        result = age_preview(original, 3_600_000_000)
        self.assertEqual(result.reason, "stale_for_control")
        self.assertEqual(result.source_ns, original.source_ns)
        self.assertGreater(len(result.path), 2)
        self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])
        result = age_preview(original, 6_100_000_000)
        self.assertEqual(result.reason, "display_expired")
        self.assertEqual(result.source_ns, original.source_ns)
        self.assertEqual(len(result.path), 0)
        self.assertIsNone(result.demand)
        self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])

    def test_expired_and_future_images_do_not_produce_geometry(self):
        for now, reason in ((6_100_000_000, "display_expired"), (2_900_000_000, "stale_or_invalid_source_timestamp")):
            result = self.evaluate(now=now)
            self.assertEqual(result.reason, reason)
            self.assertEqual(len(result.path), 0)
            self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])

    def test_watchdog_preserves_actual_detection_rejection_before_display_expiry(self):
        invalid = self.evaluate({"valid_image_path": False, "reason": "start_blocked"})
        result = age_preview(invalid, 4_000_000_000, receipt_age_sec=0.8)
        self.assertEqual(result.reason, "image_path_invalid:start_blocked")
        self.assertFalse(result.metadata()["valid_image_path"])
        self.assertEqual(result.metadata()["detection_reason"], "start_blocked")
        self.assertEqual(len(result.path), 0)
        self.assertEqual(age_preview(invalid, 7_000_000_000, receipt_age_sec=3.1).reason, "display_expired")

    def test_display_timeout_cannot_weaken_controller_freshness(self):
        result = preview_detection(self.data, self.camera, self.mount, ControllerConfig(), 3_600_000_000,
                                   display_max_age_sec=20.0)
        self.assertIsNone(result.metadata()["candidate_before_acceleration_limits"])
        with self.assertRaises(ValueError):
            preview_detection(self.data, self.camera, self.mount, ControllerConfig(), 3_100_000_000,
                              display_max_age_sec=0.2)

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

    def test_current_downward_mount_changes_projection_without_changing_intrinsics(self):
        # A level mount entered for a camera actually pointing down rejects
        # even a centered image reference as too distant from the ground origin.
        values = {"height_m": 0.77, "pitch_down_deg": 0.0, "roll_deg": 0.0,
                  "yaw_deg": 0.0, "camera_x_m": 0.0, "camera_y_m": 0.0,
                  "ground_frame": "vision_ground", "camera_frame": "camera_optical_frame"}
        # Use the webcam's narrower field of view: the wider test fixture
        # would see behind the ground origin at its bottom edge at 70 degrees.
        camera = CameraCalibration.from_mapping({"image_width": 640, "image_height": 480,
            "distortion_model": "plumb_bob", "camera_matrix": {"data": [756.8, 0, 316.6, 0, 755.0, 230.0, 0, 0, 1]},
            "distortion_coefficients": {"data": [0, 0, 0, 0, 0]}})
        level = preview_detection(self.data, camera, CameraMount.from_mapping(values), ControllerConfig(), 3_100_000_000)
        self.assertEqual(level.reason, "controller:too_far_from_path")
        values["pitch_down_deg"] = 70.0
        down = preview_detection(self.data, camera, CameraMount.from_mapping(values), ControllerConfig(), 3_100_000_000)
        self.assertEqual(down.reason, "candidate_available")
        self.assertLess(down.metadata()["distance_to_reference_m"], 0.1)
        self.assertLess(down.metadata()["path_start_ground_xy_m"][0], 0.1)


if __name__ == "__main__":
    unittest.main()
