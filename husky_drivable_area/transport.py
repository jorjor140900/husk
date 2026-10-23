"""ROS-independent image transport and bounded latest-input state.

Only byte-based Image encodings are accepted; no cv_bridge/NumPy ABI dependency.
The functions use sensor_msgs/Image attributes without importing ROS at import.
"""

from array import array
from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np


def image_to_bgr(message):
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(message.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding {message.encoding!r}; use bgr8, rgb8 or mono8")
    height, width, step = message.height, message.width, message.step
    if height < 16 or width < 16 or step < width * channels:
        raise ValueError("invalid image dimensions/step (minimum 16x16)")
    data = np.frombuffer(message.data, dtype=np.uint8)
    if len(data) != height * step:
        raise ValueError("image data length does not match height * step")
    # Respect padded rows, and own the resulting array independently of DDS.
    image = data.reshape(height, step)[:, :width * channels]
    image = image.reshape(height, width, channels).copy()
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if message.encoding == "mono8":
        return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
    return image


def fill_image(message, image, header):
    if image.dtype != np.uint8 or not (image.ndim == 2 or (image.ndim == 3 and image.shape[2] == 3)):
        raise ValueError("output must be uint8 mono or BGR")
    contiguous = np.ascontiguousarray(image)
    message.header = header
    message.height, message.width = image.shape[:2]
    message.encoding = "mono8" if image.ndim == 2 else "bgr8"
    message.is_bigendian = 0
    message.step = image.shape[1] * (1 if image.ndim == 2 else 3)
    # Generated ROS setters recognize array('B') directly. Assigning bytes runs
    # Python type/range assertions for every pixel byte (also on camera output).
    message.data = array("B", contiguous.tobytes())
    return message


@dataclass(frozen=True)
class InputFrame:
    message: object
    received_at: float


class LatestInput:
    """Subscription callbacks replace one slot while inference runs elsewhere."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = None
        self.last_received_at = time.perf_counter()
        self.last_header = None
        self.skipped = 0

    def put(self, message):
        with self._lock:
            if self._pending is not None:
                self.skipped += 1
            self.last_received_at = time.perf_counter()
            self.last_header = message.header
            self._pending = InputFrame(message, self.last_received_at)

    def take(self):
        with self._lock:
            pending, self._pending = self._pending, None
            return pending

    def state(self):
        with self._lock:
            return self.last_received_at, self.last_header, self.skipped


def frame_age_seconds(frame, now_ros_ns):
    """Include inference and DDS delay when the source supplies a ROS stamp.

    Receipt age also works for zero stamps. Camera publishers on other machines
    must use synchronized clocks; a future stamp cannot measure source latency.
    """
    receipt_age = max(0.0, time.perf_counter() - frame.received_at)
    stamp = frame.message.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    source_age = (now_ros_ns - stamp_ns) / 1e9 if stamp_ns else 0.0
    return max(receipt_age, source_age)


def status_payload(result, header, *, age_seconds=None, skipped=0, reason=None):
    if result is None:
        payload = {
            "valid_image_path": False, "reason": reason,
            "path": [], "reference_geometry": None,
            "path_coordinate_system": "processed_image_pixels_xy_near_to_far",
            "processed_shape_hw": None,
        }
    else:
        payload = result.metadata()
    payload.update({
        "source_header": {
            "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec},
            "frame_id": header.frame_id,
        } if header is not None else None,
        "frame_age_ms": age_seconds * 1000 if age_seconds is not None else None,
        "subscription_frames_skipped": skipped,
        "verified_traversability": False,
    })
    return payload
