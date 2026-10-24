"""Remote camera compression preserves image coordinates and source metadata."""
import base64
import copy
import unittest

import cv2
import numpy as np

from husky_drivable_area.robot_camera import decode_camera


class RobotCameraTests(unittest.TestCase):
    def packet(self, image, extension=".png"):
        ok, data = cv2.imencode(extension, image)
        self.assertTrue(ok)
        return {"header": {"seq": 7, "stamp": {"secs": 3, "nsecs": 100}, "frame_id": "camera_optical_frame"},
                "format": extension, "data_b64": base64.b64encode(data).decode("ascii")}

    def test_rgb_pixels_and_raw_size_preserved_without_touching_header(self):
        image = np.full((48, 64, 3), [30, 100, 200], dtype=np.uint8)
        packet = self.packet(image)
        original = copy.deepcopy(packet)
        np.testing.assert_array_equal(decode_camera(packet, 64, 48), image)
        self.assertEqual(packet, original)

    def test_jpeg_camera_is_bgr_not_swapped(self):
        image = np.full((48, 64, 3), [30, 100, 200], dtype=np.uint8)
        actual = decode_camera(self.packet(image, ".jpg"), 64, 48)
        np.testing.assert_allclose(actual[20, 20], image[20, 20], atol=4)

    def test_wrong_resolution_rejects_instead_of_resizing_calibrated_camera(self):
        packet = self.packet(np.zeros((48, 64, 3), dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "resolution"):
            decode_camera(packet, 640, 480)

    def test_depth_grayscale_malformed_and_oversized_input_rejected(self):
        for image in (np.zeros((48, 64), dtype=np.uint16), np.zeros((48, 64), dtype=np.uint8)):
            with self.assertRaisesRegex(ValueError, "RGB"):
                decode_camera(self.packet(image))
        for text in ("not-base64!", "", "a"*1_400_001):
            with self.assertRaises(ValueError):
                decode_camera({"header": {}, "format": "jpeg", "data_b64": text})


if __name__ == "__main__":
    unittest.main()
