"""ROS 2 Jazzy/Humble adapters for the existing detector and local SegFormer."""

import json
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point32, PolygonStamped
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String

from road_navigation import DetectorConfig, RoadDetector
from road_navigation.capture import LatestFrameCapture
from road_navigation.visualization import draw_result
from .transport import LatestInput, fill_compressed_image, fill_image, frame_age_seconds, image_to_bgr, status_payload
from .ground_geometry import CameraCalibration, CameraMount
from .ground_overlay import robot_axis_pixels


def image_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT)


class DrivableAreaNode(Node):
    def __init__(self):
        super().__init__("drivable_area")
        defaults = {
            "image_topic": "/camera/image_raw", "model_path": "", "device": "cpu",
            "model_width": 448, "model_height": 256, "config_file": "",
            "max_frame_age_sec": 2.0, "cpu_threads": 2, "overlay_jpeg_quality": 80,
            "camera_calibration": "", "camera_mount": "",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self._axis_geometry = None
        self._axis_cache = None
        if bool(value("camera_calibration")) != bool(value("camera_mount")):
            raise ValueError("overlay requires both camera_calibration and camera_mount")
        if value("camera_calibration"):
            import inspect
            if "robot_axis" not in inspect.signature(draw_result).parameters:
                raise ValueError("calibrated overlay requires visualization.py update: export with --update-code --update-visualization")
            self._axis_geometry = (CameraCalibration.load(value("camera_calibration")),
                                   CameraMount.load(value("camera_mount")))
        self._max_age = float(value("max_frame_age_sec"))
        if not 0 < self._max_age < float("inf"):
            raise ValueError("max_frame_age_sec must be finite and positive")
        model_path = value("model_path")
        if not model_path:
            raise ValueError("model_path is required: pass the extracted husky-segformer directory")
        cpu_threads = value("cpu_threads")
        if type(cpu_threads) is not int or cpu_threads < 1:
            raise ValueError("cpu_threads must be a positive integer")
        self._jpeg_quality = value("overlay_jpeg_quality")
        if type(self._jpeg_quality) is not int or not 1 <= self._jpeg_quality <= 100:
            raise ValueError("overlay_jpeg_quality must be an integer from 1 to 100")
        overrides = json.loads(Path(value("config_file")).expanduser().read_text()) if value("config_file") else {}
        if not isinstance(overrides, dict):
            raise ValueError("config_file must contain a DetectorConfig JSON object")
        self._detector = RoadDetector(DetectorConfig(**overrides))
        from road_navigation.segmentation import SegmentationPredictor
        import torch
        # Leave CPU time for image transport, the ROS executor, and the viewer.
        # Large thread pools can be slower than a small pool for this input size.
        torch.set_num_threads(cpu_threads)
        cv2.setNumThreads(1)
        self.get_logger().info("Loading local SegFormer model...")
        self._predictor = SegmentationPredictor(
            Path(model_path).expanduser(), value("device"),
            input_size=(value("model_width"), value("model_height")),
        )
        self._latest = LatestInput()
        self._last_invalid_at = float("-inf")
        self._last_processed_header = None
        self._overlay_pub = self.create_publisher(Image, "drivable_area/overlay", image_qos())
        self._compressed_pub = self.create_publisher(CompressedImage, "drivable_area/overlay/compressed", image_qos())
        self._mask_pub = self.create_publisher(Image, "drivable_area/mask", image_qos())
        self._clearance_pub = self.create_publisher(Image, "drivable_area/navigable_mask", image_qos())
        self._reference_pub = self.create_publisher(PolygonStamped, "drivable_area/reference_pixels", 1)
        self._status_pub = self.create_publisher(String, "drivable_area/status", 1)
        # Inference and incoming images use separate callback groups/threads.
        # DDS depth=1 AND the application slot=1 prevent an inference backlog.
        self._input_group = MutuallyExclusiveCallbackGroup()
        self._processing_group = MutuallyExclusiveCallbackGroup()
        self._subscription = self.create_subscription(
            Image, value("image_topic"), self._latest.put, image_qos(),
            callback_group=self._input_group,
        )
        self._timer = self.create_timer(0.02, self._process_latest, callback_group=self._processing_group)
        self.get_logger().info(
            f"Ready: {value('image_topic')} -> /drivable_area/*; "
            f"{self._predictor.device}, model input {self._predictor.input_size}, "
            f"torch threads {torch.get_num_threads()}. Pixel references only."
        )

    def _publish_reference(self, header, points, payload):
        reference = PolygonStamped()
        reference.header = header or Header()
        # Ordered open polyline, NOT a closed polygon or a metric ROS path.
        reference.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in points]
        self._reference_pub.publish(reference)
        published = self.get_clock().now().to_msg()
        payload["published_stamp"] = {
            "sec": published.sec, "nanosec": published.nanosec,
        }
        self._status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def _overlay_axis(self, result, header):
        if getattr(self, "_axis_geometry", None) is None:
            return None
        key = (tuple(result.frame_bgr.shape[:2]), tuple(result.input_shape), header.frame_id)
        if self._axis_cache is not None and self._axis_cache[0] == key:
            return self._axis_cache[1]
        try:
            points = robot_axis_pixels(*self._axis_geometry, *key)
        except ValueError as exc:
            self.get_logger().warning(f"Robot axis hidden: {exc}", throttle_duration_sec=2.0)
            # Configured calibration must never fall back to a centered axis.
            return []
        self._axis_cache = (key, points)
        return points

    def _invalidate(self, reason, header=None, age=None, timings=None):
        self._detector.reset()
        _, _, skipped = self._latest.state()
        payload = status_payload(
            None, header, age_seconds=age, skipped=skipped, reason=reason,
        )
        payload.update(timings or {})
        self._publish_reference(header, [], payload)
        detail = f", age {age * 1000:.0f}ms" if age is not None else ""
        if timings and "inference_ms" in timings:
            detail += f", inference {timings['inference_ms']:.0f}ms"
        self.get_logger().warning(
            f"No fresh reference: {reason}{detail}", throttle_duration_sec=2.0,
        )
        self._last_processed_header = None
        self._last_invalid_at = time.perf_counter()

    def _process_latest(self):
        pending = self._latest.take()
        if pending is None:
            received_at, header, _ = self._latest.state()
            now = time.perf_counter()
            if now - received_at > self._max_age and now - self._last_invalid_at >= 1.0:
                self._invalidate("camera_timeout", header, now - received_at)
            return
        header = pending.message.header
        timings = {}
        age = None
        try:
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            timings["input_age_ms"] = age * 1000
            if age > self._max_age:
                self._invalidate("stale_input", header, age, timings)
                return
            previous = self._last_processed_header
            # New camera/frame or backward replay timestamps start new history.
            if previous is not None and (
                header.frame_id != previous.frame_id or
                (header.stamp.sec, header.stamp.nanosec) < (previous.stamp.sec, previous.stamp.nanosec)
            ):
                self._detector.reset()
            frame = image_to_bgr(pending.message)
            started = time.perf_counter()
            probability = self._predictor.predict_probability(frame)
            inference_finished = time.perf_counter()
            timings["inference_ms"] = (inference_finished - started) * 1000
            result = self._detector.process(frame, drivable_probability=probability)
            result.backend = "segformer"
            processed_at = time.perf_counter()
            result.processing_ms = (processed_at - started) * 1000
            timings["detector_ms"] = (processed_at - inference_finished) * 1000
            timings["processing_ms"] = result.processing_ms
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            if age > self._max_age:
                self._invalidate("stale_result", header, age, timings)
                return
            _, _, skipped = self._latest.state()
            raw_subscribers = self._overlay_pub.get_subscription_count()
            jpeg_subscribers = self._compressed_pub.get_subscription_count()
            timings["overlay_raw_subscribers"] = raw_subscribers
            timings["overlay_compressed_subscribers"] = jpeg_subscribers
            if raw_subscribers > 0 or jpeg_subscribers > 0:
                axis_options = ({"robot_axis": self._overlay_axis(result, header)}
                                if getattr(self, "_axis_geometry", None) is not None else {})
                overlay = draw_result(result, telemetry={
                    "processing_ms": result.processing_ms, "host_frame_age_ms": age * 1000,
                }, **axis_options)
                if raw_subscribers > 0:
                    self._overlay_pub.publish(fill_image(Image(), overlay, header))
                if jpeg_subscribers > 0:
                    self._compressed_pub.publish(fill_compressed_image(
                        CompressedImage(), overlay, header, self._jpeg_quality,
                    ))
            for publisher, mask in ((self._mask_pub, result.road_mask),
                                    (self._clearance_pub, result.navigable_mask)):
                if publisher.get_subscription_count() > 0:
                    publisher.publish(fill_image(Image(), mask, header))
            timings["image_output_ms"] = (time.perf_counter() - processed_at) * 1000
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            if age > self._max_age:
                self._invalidate("stale_output", header, age, timings)
                return
            payload = status_payload(result, header, age_seconds=age, skipped=skipped)
            payload.update(timings)
            self._publish_reference(header, result.path, payload)
            self._last_processed_header = header
            self.get_logger().info(
                f"Frame age {age * 1000:.0f}ms | inference {timings['inference_ms']:.0f}ms | "
                f"detector {timings['detector_ms']:.0f}ms | {result.reason} | "
                f"preview subscribers raw={raw_subscribers}, compressed={jpeg_subscribers}",
                throttle_duration_sec=5.0,
            )
        except Exception as exc:
            self._invalidate("processing_error", header, age, timings)
            self.get_logger().error(f"Frame rejected: {type(exc).__name__}: {exc}")


class WebcamNode(Node):
    def __init__(self):
        super().__init__("webcam")
        for name, value in {
            "camera_index": 2, "width": 640, "height": 480, "fps": 15.0,
            "frame_id": "camera_optical_frame", "image_topic": "/camera/image_raw",
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        fps = float(value("fps"))
        if not 0 < fps <= 120 or value("width") < 16 or value("height") < 16 or value("camera_index") < 0:
            raise ValueError("camera index >= 0, dimensions >= 16, and 0 < fps <= 120 are required")
        cv2.setNumThreads(1)
        camera = cv2.VideoCapture(value("camera_index"), cv2.CAP_V4L2)
        if not camera.isOpened():
            camera.release()
            raise OSError(f"cannot open /dev/video{value('camera_index')}; close the standalone camera program")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, value("width"))
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, value("height"))
        camera.set(cv2.CAP_PROP_FPS, fps)
        self._capture = LatestFrameCapture(camera)
        self._frame_id = value("frame_id")
        self._publisher = self.create_publisher(Image, value("image_topic"), image_qos())
        self._timer = self.create_timer(1.0 / fps, self._publish_frame)
        self.get_logger().info(
            f"Camera {value('camera_index')}: "
            f"{int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}, requested {fps:g} fps"
        )

    def _publish_frame(self):
        ok, frame = self._capture.read()
        if not ok:
            raise OSError("webcam stopped delivering frames")
        # Approximate HOST read completion, not hardware exposure time.
        delay_ns = int(max(0.0, time.perf_counter() - self._capture.last_frame_timestamp) * 1e9)
        now = self.get_clock().now()
        stamp = now - Duration(nanoseconds=min(delay_ns, max(0, now.nanoseconds)))
        header = Header(stamp=stamp.to_msg(), frame_id=self._frame_id)
        self._publisher.publish(fill_image(Image(), frame, header))

    def destroy_node(self):
        self._capture.release()
        return super().destroy_node()


def _run(node_type, args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = node_type()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def detector_main(args=None):
    _run(DrivableAreaNode, args)


def camera_main(args=None):
    _run(WebcamNode, args)
