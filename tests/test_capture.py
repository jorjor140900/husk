import queue
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

import cv2
import numpy as np

from road_navigation.capture import LatestFrameCapture
from road_navigation import RoadDetector
from road_navigation.cli import _run_video


class FakeCamera:
    def __init__(self):
        self.frames = queue.Queue()
        self.released = threading.Event()
        self.read_count = 0

    def isOpened(self):
        return True

    def set(self, key, value):
        return False  # Simulate a driver ignoring the buffer-size request.

    def get(self, key):
        return self.read_count * 33 if key == cv2.CAP_PROP_POS_MSEC else 30

    def read(self):
        value = self.frames.get(timeout=2)
        if value is None:
            return False, None
        self.read_count += 1
        return True, np.full((16, 16, 3), value, np.uint8)

    def release(self):
        self.released.set()


class LatestFrameTests(unittest.TestCase):
    def test_camera_runner_writes_timing_and_capture_sequence_metadata(self):
        source = FakeCamera()
        source.frames.put(120)
        source.frames.put(None)
        args = SimpleNamespace(camera=2, video=None, save_masks=False, show=False, max_frames=1)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "result"
            with mock.patch("road_navigation.cli.cv2.VideoCapture", return_value=source):
                summary = _run_video(args, RoadDetector(), output)
            self.assertEqual(summary["frames"], 1)
            row = json.loads((output / "frames.jsonl").read_text())
            self.assertEqual(row["capture_frame_index"], 0)
            self.assertEqual(row["camera_frames_skipped"], 0)
            self.assertGreaterEqual(row["host_frame_age_ms"], 0)
            self.assertTrue(source.released.is_set())

    def test_slow_consumer_receives_latest_frame_even_when_driver_ignores_buffer_size(self):
        source = FakeCamera()
        capture = LatestFrameCapture(source)
        try:
            source.frames.put(10)
            ok, first = capture.read()
            self.assertTrue(ok)
            self.assertEqual(int(first[0, 0, 0]), 10)
            # Simulate camera frames arriving while inference is busy.
            for value in (20, 30, 40, None):
                source.frames.put(value)
            self.assertTrue(source.released.wait(timeout=2))
            ok, latest = capture.read()
            self.assertTrue(ok)
            self.assertEqual(int(latest[0, 0, 0]), 40)
            self.assertEqual(capture.last_frame_index, 3)
            self.assertEqual(capture.dropped_frames, 2)
            self.assertGreater(capture.last_frame_timestamp, 0)
            self.assertEqual(capture.get(cv2.CAP_PROP_POS_MSEC), 132)
            self.assertFalse(capture.read()[0])  # No stale frame on disconnect.
        finally:
            capture.release()

    def test_failed_camera_read_is_reported_and_released(self):
        class BrokenCamera(FakeCamera):
            def read(self):
                raise RuntimeError("camera unplugged")

        source = BrokenCamera()
        capture = LatestFrameCapture(source)
        try:
            with self.assertRaisesRegex(OSError, "capture failed"):
                capture.read()
            self.assertTrue(source.released.wait(timeout=2))
        finally:
            capture.release()


if __name__ == "__main__":
    unittest.main()
