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
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from road_navigation import DetectorConfig, RoadDetector
from road_navigation.capture import LatestFrameCapture
from road_navigation.visualization import draw_result
from .transport import LatestInput, fill_image, frame_age_seconds, image_to_bgr, status_payload


def image_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT)


class DrivableAreaNode(Node):
    def __init__(self):
        super().__init__("drivable_area")
        defaults = {
            "image_topic": "/camera/image_raw", "model_path": "", "device": "cpu",
            "model_width": 448, "model_height": 256, "config_file": "",
            "max_frame_age_sec": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self._max_age = float(value("max_frame_age_sec"))
        if not 0 < self._max_age < float("inf"):
            raise ValueError("max_frame_age_sec must be finite and positive")
        model_path = value("model_path")
        if not model_path:
            raise ValueError("model_path is required: pass the extracted husky-segformer directory")
        overrides = json.loads(Path(value("config_file")).expanduser().read_text()) if value("config_file") else {}
        if not isinstance(overrides, dict):
            raise ValueError("config_file must contain a DetectorConfig JSON object")
        self._detector = RoadDetector(DetectorConfig(**overrides))
        from road_navigation.segmentation import SegmentationPredictor
        self.get_logger().info("Loading local SegFormer model...")
        self._predictor = SegmentationPredictor(
            Path(model_path).expanduser(), value("device"),
            input_size=(value("model_width"), value("model_height")),
        )
        self._latest = LatestInput()
        self._last_invalid_at = float("-inf")
        self._last_processed_header = None
        self._overlay_pub = self.create_publisher(Image, "drivable_area/overlay", image_qos())
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
            f"{self._predictor.device}, model input {self._predictor.input_size}. Pixel references only."
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

    def _invalidate(self, reason, header=None, age=None):
        self._detector.reset()
        _, _, skipped = self._latest.state()
        self._publish_reference(header, [], status_payload(
            None, header, age_seconds=age, skipped=skipped, reason=reason,
        ))
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
        try:
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            if age > self._max_age:
                self._invalidate("stale_input", header, age)
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
            result = self._detector.process(frame, drivable_probability=probability)
            result.backend = "segformer"
            result.processing_ms = (time.perf_counter() - started) * 1000
            age = frame_age_seconds(pending, self.get_clock().now().nanoseconds)
            if age > self._max_age:
                self._invalidate("stale_result", header, age)
                return
            _, _, skipped = self._latest.state()
            overlay = draw_result(result, telemetry={
                "processing_ms": result.processing_ms, "host_frame_age_ms": age * 1000,
            })
            self._overlay_pub.publish(fill_image(Image(), overlay, header))
            self._mask_pub.publish(fill_image(Image(), result.road_mask, header))
            self._clearance_pub.publish(fill_image(Image(), result.navigable_mask, header))
            self._publish_reference(header, result.path, status_payload(
                result, header, age_seconds=age, skipped=skipped,
            ))
            self._last_processed_header = header
        except Exception as exc:
            self._invalidate("processing_error", header)
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
