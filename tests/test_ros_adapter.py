"""Transport and callback unit tests; ROS middleware is not simulated here."""

import importlib.util
from array import array
import json
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import cv2

from husky_drivable_area.transport import (
    InputFrame, LatestInput, fill_compressed_image, fill_image, frame_age_seconds, image_to_bgr, status_payload,
)
from road_navigation import DetectorConfig, RoadDetector


def header(sec=10, nanosec=0, frame_id="camera_optical_frame"):
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id=frame_id)


def image_message(image=None):
    if image is None:
        image = np.full((240, 320, 3), 120, np.uint8)
    return fill_image(SimpleNamespace(), image, header())


class TransportTests(unittest.TestCase):
    def test_padded_rgb_rows_preserve_color_and_do_not_alias_dds_buffer(self):
        rgb = np.zeros((16, 17, 3), np.uint8)
        rgb[:] = (20, 40, 80)
        padded = np.full((16, 17 * 3 + 5), 255, np.uint8)
        padded[:, :51] = rgb.reshape(16, 51)
        message = SimpleNamespace(height=16, width=17, step=56, encoding="rgb8", data=bytearray(padded.tobytes()))
        bgr = image_to_bgr(message)
        self.assertEqual(bgr.shape, (16, 17, 3))
        self.assertTrue(np.all(bgr == (80, 40, 20)))
        message.data[:] = bytes(len(message.data))
        self.assertTrue(np.all(bgr == (80, 40, 20)))

    def test_mono_roundtrip_and_non_contiguous_output(self):
        mono = np.arange(32 * 32, dtype=np.uint8).reshape(32, 32)[:, ::-1]
        message = fill_image(SimpleNamespace(), mono, header())
        self.assertEqual(message.encoding, "mono8")
        self.assertEqual(message.step, 32)
        self.assertIsInstance(message.data, array)
        self.assertEqual(message.data.typecode, "B")
        decoded = image_to_bgr(message)
        for channel in range(3):
            np.testing.assert_array_equal(decoded[:, :, channel], mono)

    def test_bad_encodings_steps_and_lengths_are_rejected(self):
        for field, value in (("encoding", "16UC1"), ("step", 1), ("data", b"")):
            message = image_message()
            setattr(message, field, value)
            with self.subTest(field=field), self.assertRaises(ValueError):
                image_to_bgr(message)

    def test_jpeg_preview_preserves_header_dimensions_and_does_not_change_input(self):
        frame = np.full((240, 320, 3), (80, 120, 160), np.uint8)
        original = frame.copy()
        source_header = header()
        message = fill_compressed_image(SimpleNamespace(), frame, source_header)
        self.assertIs(message.header, source_header)
        self.assertEqual(message.format, "bgr8; jpeg compressed bgr8")
        self.assertIsInstance(message.data, array)
        self.assertEqual(message.data.typecode, "B")
        decoded = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, frame.shape)
        self.assertLess(np.abs(decoded.astype(float) - frame).mean(), 3)
        self.assertLess(len(message.data), frame.nbytes / 10)
        np.testing.assert_array_equal(frame, original)
        for quality in (0, 101, True):
            with self.subTest(quality=quality), self.assertRaises(ValueError):
                fill_compressed_image(SimpleNamespace(), frame, source_header, quality)

    def test_latest_slot_replaces_waiting_frames_without_repeating_consumed_input(self):
        slot = LatestInput()
        for index in range(10):
            message = image_message()
            message.header.stamp.sec = index
            slot.put(message)
        pending = slot.take()
        self.assertEqual(pending.message.header.stamp.sec, 9)
        self.assertEqual(slot.state()[2], 9)
        self.assertIsNone(slot.take())
        # A new callback can enqueue while inference holds the previous item.
        slot.put(image_message())
        self.assertIsNotNone(slot.take())

    def test_age_includes_source_delay_and_inference_even_with_zero_stamp(self):
        frame = InputFrame(image_message(), 100.0)
        with mock.patch("husky_drivable_area.transport.time.perf_counter", return_value=100.1):
            self.assertAlmostEqual(frame_age_seconds(frame, 13_000_000_000), 3.0)
        frame.message.header.stamp.sec = 0
        with mock.patch("husky_drivable_area.transport.time.perf_counter", return_value=102.5):
            self.assertAlmostEqual(frame_age_seconds(frame, 13_000_000_000), 2.5)

    def test_resized_mask_and_reference_use_the_same_pixel_coordinates(self):
        frame = np.full((120, 240, 3), 120, np.uint8)
        result = RoadDetector(DetectorConfig(max_width=160, analysis_width=160, clearance_px=2)).process(
            frame, drivable_probability=np.ones((120, 240), np.float32),
        )
        self.assertTrue(result.valid_image_path, result.reason)
        payload = status_payload(result, header(), age_seconds=0.2)
        mask = fill_image(SimpleNamespace(), result.road_mask, header())
        self.assertEqual(payload["processed_shape_hw"], [mask.height, mask.width])
        self.assertEqual((mask.height, mask.width), (80, 160))
        for x, y in payload["path"]:
            self.assertTrue(0 <= x < mask.width and 0 <= y < mask.height)
            self.assertEqual(result.navigable_mask[y, x], 255)
        self.assertFalse(payload["verified_traversability"])
        self.assertEqual(payload["source_header"]["stamp"]["sec"], 10)
        json.dumps(payload, allow_nan=False)

    def test_invalid_status_carries_no_old_geometry(self):
        payload = status_payload(None, None, reason="camera_timeout")
        self.assertFalse(payload["valid_image_path"])
        self.assertEqual(payload["path"], [])
        self.assertIsNone(payload["reference_geometry"])
        self.assertIsNone(payload["source_header"])


def load_node_callbacks():
    """Import real callbacks with message/Node stand-ins, not a fake ROS runtime."""
    class Polygon:
        def __init__(self):
            self.header = None
            self.polygon = SimpleNamespace(points=[])

    modules = {
        "rclpy": SimpleNamespace(),
        "rclpy.callback_groups": SimpleNamespace(MutuallyExclusiveCallbackGroup=object),
        "rclpy.duration": SimpleNamespace(Duration=object),
        "rclpy.executors": SimpleNamespace(ExternalShutdownException=RuntimeError, MultiThreadedExecutor=object),
        "rclpy.node": SimpleNamespace(Node=object),
        "rclpy.qos": SimpleNamespace(HistoryPolicy=object, QoSProfile=object, ReliabilityPolicy=object),
        "geometry_msgs": SimpleNamespace(),
        "geometry_msgs.msg": SimpleNamespace(Point32=SimpleNamespace, PolygonStamped=Polygon),
        "sensor_msgs": SimpleNamespace(),
        "sensor_msgs.msg": SimpleNamespace(Image=SimpleNamespace, CompressedImage=SimpleNamespace),
        "std_msgs": SimpleNamespace(),
        "std_msgs.msg": SimpleNamespace(Header=header, String=SimpleNamespace),
    }
    path = Path(__file__).resolve().parents[1] / "husky_drivable_area/nodes.py"
    spec = importlib.util.spec_from_file_location("husky_drivable_area._callbacks_under_test", path)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict("sys.modules", modules):
        spec.loader.exec_module(module)
    return module


class CallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_node_callbacks()

    def setUp(self):
        self.node = object.__new__(self.module.DrivableAreaNode)
        self.node._detector = RoadDetector(DetectorConfig(use_temporal=False))
        self.node._predictor = mock.Mock()
        self.node._predictor.predict_probability.side_effect = lambda frame: np.ones(frame.shape[:2], np.float32)
        self.node._latest = LatestInput()
        self.node._max_age = 2.0
        self.node._last_invalid_at = float("-inf")
        self.node._last_processed_header = None
        self.node._jpeg_quality = 80
        for name in ("_overlay_pub", "_compressed_pub", "_mask_pub", "_clearance_pub", "_reference_pub", "_status_pub"):
            setattr(self.node, name, mock.Mock())
        self.node._compressed_pub.get_subscription_count.return_value = 0
        for publisher in (self.node._overlay_pub, self.node._mask_pub, self.node._clearance_pub):
            publisher.get_subscription_count.return_value = 1
        now = SimpleNamespace(nanoseconds=10_000_000_000, to_msg=lambda: header().stamp)
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: now)
        self.node.get_logger = mock.Mock()

    def status(self):
        return json.loads(self.node._status_pub.publish.call_args.args[0].data)

    def test_calibrated_axis_cache_invalidates_on_frame_change_without_center_fallback(self):
        from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
        from test_ground_geometry import camera_mapping, mount_mapping
        self.node._axis_geometry = (
            CameraCalibration.from_mapping(camera_mapping()),
            CameraMount.from_mapping(mount_mapping(camera_y_m=0.21)))
        self.node._axis_cache = None
        result = SimpleNamespace(frame_bgr=np.zeros((240, 320, 3), np.uint8), input_shape=(480, 640))
        with mock.patch.object(self.module, "robot_axis_pixels", wraps=self.module.robot_axis_pixels) as project:
            first = self.node._overlay_axis(result, header())
            self.assertIs(first, self.node._overlay_axis(result, header()))
            self.assertEqual(project.call_count, 1)
            self.assertEqual(self.node._overlay_axis(result, header(frame_id="wrong_camera")), [])
            self.assertEqual(project.call_count, 2)

    def test_success_draws_calibrated_axis_with_processed_dimensions(self):
        from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
        from test_ground_geometry import camera_mapping, mount_mapping
        self.node._axis_geometry = (
            CameraCalibration.from_mapping(camera_mapping()),
            CameraMount.from_mapping(mount_mapping(camera_y_m=0.21)))
        self.node._axis_cache = None
        self.node._detector = RoadDetector(DetectorConfig(max_width=320, use_temporal=False))
        self.node._latest.put(image_message(np.full((480, 640, 3), 120, np.uint8)))
        with mock.patch.object(self.module, "draw_result", wraps=self.module.draw_result) as draw:
            self.node._process_latest()
        axis = draw.call_args.kwargs["robot_axis"]
        self.assertEqual(axis.shape, (512, 2))
        self.assertTrue(np.isfinite(axis).any())
        self.assertEqual(self.status()["processed_shape_hw"], [240, 320])

    def test_success_preserves_header_and_stale_result_clears_previous_path(self):
        message = image_message()
        self.node._latest.put(message)
        self.node._process_latest()
        self.assertTrue(self.status()["valid_image_path"])
        reference = self.node._reference_pub.publish.call_args.args[0]
        self.assertIs(reference.header, message.header)
        self.assertTrue(reference.polygon.points)
        self.assertEqual(self.status()["backend"], "segformer")
        for field in ("input_age_ms", "inference_ms", "detector_ms", "image_output_ms"):
            self.assertGreaterEqual(self.status()[field], 0)
        self.node._overlay_pub.reset_mock()
        self.node._latest.put(message)
        with mock.patch.object(self.module, "frame_age_seconds", side_effect=[0.1, 3.0]):
            self.node._process_latest()
        self.assertEqual(self.status()["reason"], "stale_result")
        self.assertGreaterEqual(self.status()["inference_ms"], 0)
        self.assertFalse(self.status()["valid_image_path"])
        self.assertEqual(self.node._reference_pub.publish.call_args.args[0].polygon.points, [])
        self.node._overlay_pub.publish.assert_not_called()

    def test_no_image_subscriber_skips_rendering_and_conversion_but_keeps_reference(self):
        for publisher in (self.node._overlay_pub, self.node._mask_pub, self.node._clearance_pub):
            publisher.get_subscription_count.return_value = 0
        self.node._latest.put(image_message())
        with mock.patch.object(self.module, "draw_result") as draw, mock.patch.object(self.module, "fill_image") as fill:
            self.node._process_latest()
        draw.assert_not_called()
        fill.assert_not_called()
        self.assertTrue(self.status()["valid_image_path"])
        self.node._mask_pub.publish.assert_not_called()

    def test_compressed_only_viewer_receives_preview_when_no_raw_viewer_is_connected(self):
        self.node._overlay_pub.get_subscription_count.return_value = 0
        self.node._compressed_pub.get_subscription_count.return_value = 1
        self.node._latest.put(image_message())
        self.node._process_latest()
        self.node._overlay_pub.publish.assert_not_called()
        self.node._compressed_pub.publish.assert_called_once()
        message = self.node._compressed_pub.publish.call_args.args[0]
        decoded = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (240, 320, 3))
        self.assertTrue(self.status()["valid_image_path"])
        self.assertEqual(self.status()["overlay_raw_subscribers"], 0)
        self.assertEqual(self.status()["overlay_compressed_subscribers"], 1)

    def test_slow_image_output_cannot_leave_a_valid_reference(self):
        self.node._latest.put(image_message())
        with mock.patch.object(self.module, "frame_age_seconds", side_effect=[0.1, 0.5, 3.0]):
            self.node._process_latest()
        self.assertEqual(self.status()["reason"], "stale_output")
        self.assertFalse(self.status()["valid_image_path"])
        self.assertEqual(self.node._reference_pub.publish.call_args.args[0].polygon.points, [])

    def test_camera_silence_invalidates_reference_and_resets_history(self):
        self.node._latest.put(image_message())
        self.node._process_latest()
        self.node._detector.reset = mock.Mock(wraps=self.node._detector.reset)
        self.node._latest.state = lambda: (time.perf_counter() - 3, header(), 0)
        self.node._process_latest()
        self.assertEqual(self.status()["reason"], "camera_timeout")
        self.assertEqual(self.status()["path"], [])
        self.node._detector.reset.assert_called_once()

    def test_input_already_stale_does_not_run_model(self):
        message = image_message()
        message.header.stamp.sec = 5
        self.node._latest.put(message)
        self.node._process_latest()
        self.node._predictor.predict_probability.assert_not_called()
        self.assertEqual(self.status()["reason"], "stale_input")

    def test_inference_error_and_unsupported_input_publish_empty_reference(self):
        self.node._predictor.predict_probability.side_effect = RuntimeError("model failed")
        for encoding in ("bgr8", "16UC1"):
            message = image_message()
            message.encoding = encoding
            self.node._latest.put(message)
            self.node._process_latest()
            self.assertEqual(self.status()["reason"], "processing_error")
            self.assertFalse(self.status()["valid_image_path"])
            self.assertEqual(self.node._reference_pub.publish.call_args.args[0].polygon.points, [])

    def test_blocked_seed_does_not_republish_previous_valid_reference(self):
        self.node._latest.put(image_message())
        self.node._process_latest()
        self.assertTrue(self.status()["valid_image_path"])
        self.node._predictor.predict_probability.side_effect = lambda frame: np.zeros(frame.shape[:2], np.float32)
        self.node._latest.put(image_message())
        self.node._process_latest()
        self.assertFalse(self.status()["valid_image_path"])
        self.assertEqual(self.node._reference_pub.publish.call_args.args[0].polygon.points, [])


if __name__ == "__main__":
    unittest.main()
