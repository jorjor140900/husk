"""Distinguish a blocked publish from source-capture starvation."""
import json
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import numpy as np

from test_ros_adapter import load_node_callbacks


class WebcamDeliveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_callbacks()

    def setUp(self):
        self.node = object.__new__(self.module.WebcamNode)
        self.node._capture = mock.Mock(last_frame_timestamp=10.0)
        self.frame = np.full((480, 640, 3), (12, 34, 56), np.uint8)
        self.node._capture.read.return_value = (True, self.frame)
        self.node._capture.diagnostics.return_value = dict(captured_frames=30, capture_age_ms=1,
            capture_gap_max_ms=67, device_read_max_ms=67, skipped_capture_frames=0)
        self.node._frame_id = "calibrated_camera"
        self.node._publisher = mock.Mock()
        self.node._capture_status_pub = mock.Mock()
        self.node._capture_report_at = 10.0
        self.node._capture_report_frames = 0
        self.node._publish_last_at = None
        self.node._delivery_window = {}
        self.source_stamp = NS(sec=100, nanosec=0)
        ros_now = mock.Mock(nanoseconds=100_000_000_000)
        ros_now.__sub__ = mock.Mock(return_value=NS(to_msg=lambda: self.source_stamp))
        self.node.get_clock = lambda: NS(now=lambda: ros_now)

    def publish(self):
        # Includes a two-second block inside publish; the USB frame remains fresh
        # when the callback starts. All numbers use the monotonic clock.
        with mock.patch.object(self.module, "Duration", side_effect=lambda **kw: NS(**kw)), \
             mock.patch.object(self.module, "Header", NS), \
             mock.patch.object(self.module.time, "perf_counter", side_effect=[10.0, 10.001, 10.002, 10.003, 12.003]):
            self.node._publish_frame()

    def test_publish_stall_is_separate_from_usb_read_and_preserves_raw_frame(self):
        self.publish()
        image = self.node._publisher.publish.call_args.args[0]
        self.assertIs(image.header.stamp, self.source_stamp)
        self.assertEqual(image.header.frame_id, "calibrated_camera")
        self.assertEqual((image.width, image.height), (640, 480))
        np.testing.assert_array_equal(np.frombuffer(image.data, np.uint8).reshape(480, 640, 3), self.frame)
        with mock.patch.object(self.module.time, "perf_counter", return_value=12.01):
            self.node._publish_capture_status()
        result = json.loads(self.node._capture_status_pub.publish.call_args.args[0].data)
        self.assertAlmostEqual(result["delivery"]["dds_publish_max_ms"], 2000)
        self.assertAlmostEqual(result["delivery"]["slot_wait_max_ms"], 1)
        self.assertAlmostEqual(result["capture_fps"], 30 / 2.01)
        self.assertEqual(result["capture"]["capture_gap_max_ms"], 67)
        self.assertEqual(self.node._delivery_window, {})

    def test_empty_report_window_does_not_repeat_previous_publish_count(self):
        self.publish()
        with mock.patch.object(self.module.time, "perf_counter", side_effect=[12.01, 13.01]):
            self.node._publish_capture_status()
            self.node._publish_capture_status()
        result = json.loads(self.node._capture_status_pub.publish.call_args.args[0].data)
        self.assertEqual(result["published_fps"], 0)
        self.assertEqual(result["capture_fps"], 0)

    def test_disconnected_camera_never_publishes_previous_frame(self):
        self.node._capture.read.return_value = (False, None)
        with self.assertRaisesRegex(OSError, "stopped delivering"):
            self.node._publish_frame()
        self.node._publisher.publish.assert_not_called()


if __name__ == "__main__":
    unittest.main()
