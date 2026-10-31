import tempfile
from pathlib import Path
import unittest

import cv2
import numpy as np

from husky_drivable_area.charuco import CharucoSession, save_calibration
from husky_drivable_area.ground_geometry import CameraCalibration


class CharucoTests(unittest.TestCase):
    def test_printed_board_detection_duplicate_rejection_and_readiness(self):
        session = CharucoSession()
        image = cv2.cvtColor(session.board.generateImage((640, 480), marginSize=25), cv2.COLOR_GRAY2BGR)
        corners, ids = session.detect(image)
        self.assertEqual(len(ids), 24)
        self.assertEqual(session.add(image, corners, ids), "captured")
        self.assertEqual(session.add(image, corners, ids), "move_or_tilt_board_for_new_view")
        self.assertFalse(session.progress()["ready_to_calibrate"])
        with self.assertRaisesRegex(ValueError, "views"):
            session.calibrate()
        with self.assertRaisesRegex(ValueError, "resolution"):
            session.add(image[:240], corners, ids)

    def test_wrong_dictionary_and_blur_are_not_captured(self):
        session = CharucoSession()
        wrong = CharucoSession(dictionary="DICT_4X4_50")
        image = cv2.cvtColor(session.board.generateImage((640, 480), marginSize=25), cv2.COLOR_GRAY2BGR)
        corners, ids = wrong.detect(image)
        self.assertEqual(wrong.add(image, corners, ids), "need_at_least_12_charuco_corners")
        corners, ids = session.detect(image)
        blank = np.full_like(image, 120)
        self.assertEqual(session.add(blank, corners, ids), "hold_board_still_or_improve_focus")

    def test_recovers_known_intrinsics_from_diverse_synthetic_views_and_keeps_previous_output(self):
        session = CharucoSession()
        matrix = np.array([[530., 0, 319.5], [0, 535., 239.5], [0, 0, 1]])
        distortion = np.array([.02, -.01, .001, -.002, .005])
        random = np.random.default_rng(42)
        frame = random.integers(0, 255, (480, 640, 3), dtype=np.uint8)
        center = np.array([.084, .060, 0.])
        for _ in range(200):
            rvec = random.uniform([-.6, -.6, -.5], [.6, .6, .5])
            rotation, _ = cv2.Rodrigues(rvec)
            z = random.uniform(.25, .55)
            u, v = random.uniform([90, 80], [550, 400])
            tvec = np.array([(u-319.5)*z/530, (v-239.5)*z/535, z])-rotation@center
            pixels, _ = cv2.projectPoints(session.object_corners, rvec, tvec, matrix, distortion)
            pixels = pixels.reshape(-1, 2)
            valid = ((pixels[:, 0] > 5) & (pixels[:, 0] < 635) & (pixels[:, 1] > 5) & (pixels[:, 1] < 475))
            if valid.sum() < 12:
                continue
            pixels += random.normal(0, .04, pixels.shape)
            session.add(frame, pixels[valid].reshape(-1, 1, 2), np.flatnonzero(valid).reshape(-1, 1).astype(np.int32))
            if len(session.views) >= 30 and session.progress()["ready_to_calibrate"]:
                break
        self.assertTrue(session.progress()["ready_to_calibrate"], session.progress())
        mapping, report = session.calibrate()
        camera = CameraCalibration.from_mapping(mapping)
        np.testing.assert_allclose(camera.matrix[:2, :2], matrix[:2, :2], rtol=.02, atol=1)
        np.testing.assert_allclose(camera.matrix[:2, 2], matrix[:2, 2], atol=2)
        self.assertLess(report["training_rms_px"], .15)
        self.assertLess(max(report["held_out_rms_px"]), .15)
        self.assertEqual(report["held_out_views"], 6)
        self.assertFalse(report["ready_for_control"])
        with tempfile.TemporaryDirectory() as folder:
            save_calibration(folder, mapping, report)
            loaded = CameraCalibration.load(Path(folder)/"camera.yaml")
            np.testing.assert_allclose(loaded.matrix, camera.matrix)
            before = (Path(folder)/"camera.yaml").read_bytes()
            with self.assertRaises(FileExistsError):
                save_calibration(folder, mapping, report)
            self.assertEqual((Path(folder)/"camera.yaml").read_bytes(), before)
