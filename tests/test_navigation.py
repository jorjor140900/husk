import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np

from road_navigation import DetectorConfig, RoadDetector
from road_navigation.planner import plan_path


ROOT = Path(__file__).resolve().parents[1]


def corridor():
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[72:, 30:290] = 255
    return mask


def outdoor_frame():
    frame = np.full((240, 320, 3), (35, 125, 35), dtype=np.uint8)
    frame[72:, 30:290] = (120, 120, 120)
    return frame


class PlannerTests(unittest.TestCase):
    def assert_path_clear(self, plan, mask):
        self.assertTrue(plan.valid, plan.reason)
        raster = np.zeros_like(mask)
        cv2.polylines(raster, [plan.points], False, 255, 1)
        self.assertFalse(np.any((raster > 0) & (mask == 0)))
        self.assertFalse(np.any((raster > 0) & (plan.navigable_mask == 0)))
        self.assertTrue(np.all(np.diff(plan.points[:, 1]) == -1))

    def test_reference_detours_around_central_obstacle(self):
        mask = corridor()
        mask[108:174, 135:185] = 0
        plan = plan_path(mask, mask / 255.0, clearance_px=5)
        self.assert_path_clear(plan, mask)
        self.assertGreater(np.max(np.abs(plan.points[:, 0] - 160)), 25)
        self.assertEqual(plan.reason, "ok")

    def test_blocked_start_does_not_teleport_to_another_component(self):
        mask = corridor()
        mask[200:, 140:180] = 0
        plan = plan_path(mask, mask / 255.0)
        self.assertFalse(plan.valid)
        self.assertEqual(plan.reason, "start_blocked_or_too_narrow")

    def test_clearance_rejects_narrow_corridor(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[72:, 156:165] = 255
        self.assertFalse(plan_path(mask, mask / 255.0, clearance_px=8).valid)

    def test_path_stops_before_a_full_width_barrier(self):
        mask = corridor()
        mask[140:150] = 0
        plan = plan_path(mask, mask / 255.0, clearance_px=5)
        self.assert_path_clear(plan, mask)
        self.assertGreater(int(plan.points[:, 1].min()), 150)
        self.assertEqual(plan.reason, "limited_lookahead")

    def test_nearby_barrier_rejects_insufficient_lookahead(self):
        mask = corridor()
        mask[200:210] = 0
        self.assertFalse(plan_path(mask, mask / 255.0, clearance_px=5).valid)

    def test_diagonal_contact_cannot_cut_a_blocked_corner(self):
        mask = np.zeros((5, 5), dtype=np.uint8)
        mask[4, 2] = mask[3, 1] = mask[2, 0] = 255
        plan = plan_path(mask, mask / 255.0, seed_y=1, horizon=0, clearance_px=0, max_lateral_step=1, min_path_height=0.2)
        self.assertFalse(plan.valid)

    def test_two_pixel_transition_cannot_jump_a_hole(self):
        mask = np.zeros((5, 7), np.uint8)
        mask[3:5, 3] = 255
        mask[0:4, 5] = 255
        plan = plan_path(mask, mask / 255.0, seed_y=1, horizon=0, clearance_px=0, max_lateral_step=2, min_path_height=0.5)
        self.assertFalse(plan.valid)


class DetectorTests(unittest.TestCase):
    def test_shadow_does_not_become_a_hard_edge(self):
        frame = outdoor_frame()
        frame[110:160, 30:290] = (40, 40, 40)
        result = RoadDetector(DetectorConfig(use_temporal=False)).process(frame)
        self.assertTrue(result.valid_image_path, result.reason)
        self.assertEqual(result.road_mask[130, 160], 255)
        self.assertEqual(result.road_mask[130, 10], 0)
        self.assertEqual(result.road_mask[20, 160], 0)

    def test_appearance_preserves_a_different_colored_obstacle(self):
        frame = outdoor_frame()
        frame[108:174, 135:185] = (180, 25, 25)
        result = RoadDetector().process(frame)
        self.assertEqual(result.road_mask[140, 160], 0)
        self.assertTrue(result.valid_image_path, result.reason)
        x, y = result.path.T
        self.assertTrue(np.all(result.road_mask[y, x] > 0))

    def test_current_obstacle_clears_temporal_history_immediately(self):
        frame = outdoor_frame()
        detector = RoadDetector(DetectorConfig(temporal_alpha=0.1))
        probability = corridor().astype(np.float32) / 255
        self.assertTrue(detector.process(frame, drivable_probability=probability).valid_image_path)
        probability[108:174, 135:185] = 0
        result = detector.process(frame, drivable_probability=probability)
        self.assertEqual(result.road_mask[140, 160], 0)
        self.assertEqual(result.probability[140, 160], 0)
        x, y = result.path.T
        self.assertTrue(result.valid_image_path)
        self.assertTrue(np.all(probability[y, x] > 0))

    def test_no_evidence_returns_no_stale_path(self):
        frame = outdoor_frame()
        detector = RoadDetector()
        detector.process(frame, drivable_probability=corridor() / 255)
        result = detector.process(frame, drivable_probability=np.zeros(frame.shape[:2]))
        self.assertFalse(result.valid_image_path)
        self.assertFalse(result.road_mask.any())
        self.assertEqual(result.quality_score, 0)

    def test_history_follows_image_motion_instead_of_staying_at_old_pixels(self):
        rng = np.random.default_rng(17)
        texture = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), 0)
        first_frame = cv2.cvtColor(texture, cv2.COLOR_GRAY2BGR)
        first_probability = np.ones(texture.shape, dtype=np.float32)
        first_probability[90:180, 90:140] = 0
        detector = RoadDetector(DetectorConfig(temporal_alpha=0.1))
        detector.process(first_frame, drivable_probability=first_probability)
        shifted_frame = cv2.warpAffine(first_frame, np.float32([[1, 0, 12], [0, 1, 0]]), (320, 240))
        uncertain_probability = np.ones(texture.shape, dtype=np.float32)
        uncertain_probability[90:180, 102:152] = 0.55
        result = detector.process(shifted_frame, drivable_probability=uncertain_probability)
        # This strip was outside the old hole, but is inside its shifted position.
        self.assertLess(float(result.probability[110:160, 144:150].mean()), 0.3)

    def test_subpixel_blocked_mask_survives_2k_downsampling(self):
        frame = np.full((1152, 2048, 3), 120, np.uint8)
        blocked = np.zeros(frame.shape[:2], np.uint8)
        cfg = DetectorConfig()
        sy, sx = round(359 * cfg.seed_y_fraction), round(639 * cfg.seed_x_fraction)
        original_y, original_x = int(sy * 1152 / 360), int(sx * 2048 / 640)
        blocked[original_y, original_x] = 255
        result = RoadDetector(cfg).process(frame, drivable_probability=np.ones(frame.shape[:2]), blocked_mask=blocked)
        self.assertEqual(result.frame_bgr.shape[:2], (360, 640))
        self.assertEqual(result.road_mask[sy, sx], 0)
        self.assertFalse(result.valid_image_path)

    def test_repeated_frame_and_reset_are_deterministic(self):
        detector = RoadDetector()
        frame = outdoor_frame()
        first = detector.process(frame)
        second = detector.process(frame)
        np.testing.assert_array_equal(first.path, second.path)
        detector.reset()
        third = detector.process(frame)
        np.testing.assert_array_equal(first.road_mask, third.road_mask)
        np.testing.assert_array_equal(first.path, third.path)

    def test_external_probability_validation(self):
        detector = RoadDetector()
        frame = outdoor_frame()
        for invalid in (np.zeros((3, 4)), np.full(frame.shape[:2], np.nan), np.full(frame.shape[:2], 255)):
            with self.subTest(shape=invalid.shape):
                with self.assertRaises(ValueError):
                    detector.process(frame, drivable_probability=invalid)

    def test_configuration_validation(self):
        for values in ({"temporal_alpha": 0}, {"chroma_tolerance": float("nan")}, {"horizon_fraction": 0.99}, {"clearance_px": -1}, {"max_width": 640.5}):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    DetectorConfig(**values)


class RunnerTests(unittest.TestCase):
    def run_cli(self, *args):
        return subprocess.run([sys.executable, str(ROOT / "RoadDetection.py"), *map(str, args)], cwd=ROOT, capture_output=True, text=True)

    def test_headless_image_writes_outputs_and_preserves_obstacle(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            image = folder / "input.png"
            probability_file = folder / "probability.npy"
            mask = corridor().astype(np.float32) / 255
            mask[108:174, 135:185] = 0
            self.assertTrue(cv2.imwrite(str(image), outdoor_frame()))
            np.save(probability_file, mask)
            output = folder / "result"
            command = ("--image", image, "--probability", probability_file, "--output", output, "--config", ROOT / "config.example.json")
            run = self.run_cli(*command)
            self.assertEqual(run.returncode, 0, run.stderr)
            data = json.loads((output / "result.json").read_text())
            self.assertTrue(data["valid_image_path"])
            saved_mask = cv2.imread(str(output / "mask.png"), cv2.IMREAD_GRAYSCALE)
            self.assertEqual(saved_mask[140, 160], 0)
            self.assertTrue((output / "overlay.png").is_file())
            original_json = (output / "result.json").read_bytes()
            repeat = self.run_cli(*command)
            self.assertNotEqual(repeat.returncode, 0)
            self.assertEqual((output / "result.json").read_bytes(), original_json)

    def test_video_outputs_decode_and_match_actual_frame_count(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            video = folder / "input.avi"
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 10, (320, 240))
            self.assertTrue(writer.isOpened(), "test requires MJPEG encoding")
            try:
                for _ in range(6):
                    writer.write(outdoor_frame())
            finally:
                writer.release()
            output = folder / "result"
            run = self.run_cli("--video", video, "--output", output, "--save-masks", "--max-frames", 4)
            self.assertEqual(run.returncode, 0, run.stderr)
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["frames"], 4)
            self.assertEqual(summary["valid_image_paths"], 4)
            self.assertEqual(len((output / "frames.jsonl").read_text().splitlines()), 4)
            self.assertEqual(len(list((output / "masks").glob("*.png"))), 4)
            capture = cv2.VideoCapture(str(output / "overlay.avi"))
            decoded = 0
            try:
                while capture.read()[0]:
                    decoded += 1
            finally:
                capture.release()
            self.assertEqual(decoded, 4)

    def test_missing_source_fails_without_claiming_success(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "result"
            run = self.run_cli("--image", Path(folder) / "missing.png", "--output", output)
            self.assertNotEqual(run.returncode, 0)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
