import json
from pathlib import Path
import tempfile
import unittest
import zipfile

import cv2
import numpy as np

from tools.prepare_cvat_dataset import prepare_dataset


class CvatDatasetTests(unittest.TestCase):
    def make_inputs(self, root, frame_number=1, shape=(48, 64), unknown_color=False):
        video = root / "clip.avi"
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 10, (64, 48))
        self.assertTrue(writer.isOpened())
        try:
            for brightness in (30, 100, 180):
                writer.write(np.full((48, 64, 3), brightness, np.uint8))
        finally:
            writer.release()
        color = np.zeros((*shape, 3), np.uint8)
        color[:, :20] = (114, 183, 71)  # BGR, labelmap below is RGB.
        color[:, 20:40] = (236, 234, 146)
        if unknown_color:
            color[0, 0] = (1, 2, 3)
        ok, png = cv2.imencode(".png", color)
        self.assertTrue(ok)
        archive = root / "annotations.zip"
        with zipfile.ZipFile(archive, "w") as output:
            output.writestr("labelmap.txt", "# label:color_rgb:parts:actions\nbackground:0,0,0::\ndrivable:71,183,114::\nnon-drivable:146,234,236::\n")
            output.writestr(f"SegmentationClass/frame_{frame_number:06d}.png", png.tobytes())
            # An instance mask is not a semantic mask and must not be used.
            output.writestr(f"SegmentationObject/frame_{frame_number:06d}.png", b"not a class PNG")
        return archive, video

    def test_rgb_mapping_ignore_pixels_and_exact_source_frame(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            annotations, video = self.make_inputs(root)
            out = root / "paired"
            summary = prepare_dataset(annotations, video, out, "session-01")
            self.assertEqual(summary["samples"], 1)
            self.assertEqual(summary["source_frame_indices_zero_based"], [1])
            mask = cv2.imread(str(out / "masks/frame_000001.png"), cv2.IMREAD_UNCHANGED)
            self.assertEqual(mask.ndim, 2)
            self.assertTrue(np.all(mask[:, :20] == 1))
            self.assertTrue(np.all(mask[:, 20:40] == 0))
            self.assertTrue(np.all(mask[:, 40:] == 255))
            source = cv2.VideoCapture(str(video))
            try:
                source.read()
                ok, expected = source.read()
            finally:
                source.release()
            self.assertTrue(ok)
            image = cv2.imread(str(out / "images/frame_000001.png"))
            np.testing.assert_array_equal(image, expected)
            manifest = json.loads((out / "manifest.json").read_text())
            self.assertEqual(manifest[0]["recording_group"], "session-01")
            self.assertEqual(manifest[0]["split"], "unassigned")

    def test_unrecognized_color_does_not_silently_become_ground_or_blocked(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            annotations, video = self.make_inputs(root, unknown_color=True)
            with self.assertRaisesRegex(ValueError, "colors missing"):
                prepare_dataset(annotations, video, root / "paired", "session-01")
            self.assertFalse((root / "paired").exists())

    def test_dimension_mismatch_is_not_hidden_by_resizing(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            annotations, video = self.make_inputs(root, shape=(24, 32))
            with self.assertRaisesRegex(ValueError, "differs from mask"):
                prepare_dataset(annotations, video, root / "paired", "session-01")
            self.assertFalse((root / "paired").exists())

    def test_source_frame_beyond_video_does_not_leave_a_completed_dataset(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            annotations, video = self.make_inputs(root, frame_number=5)
            with self.assertRaisesRegex(ValueError, "ends before"):
                prepare_dataset(annotations, video, root / "paired", "session-01")
            self.assertFalse((root / "paired").exists())

    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            annotations, video = self.make_inputs(root)
            out = root / "paired"
            out.mkdir()
            sentinel = out / "keep.txt"
            sentinel.write_text("existing experiment")
            with self.assertRaises(FileExistsError):
                prepare_dataset(annotations, video, out, "session-01")
            self.assertEqual(sentinel.read_text(), "existing experiment")


if __name__ == "__main__":
    unittest.main()
