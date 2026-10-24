"""Bounded decoding for an optional ROS1 compressed RGB camera."""
import base64
import binascii
import io

import cv2
import numpy as np
from PIL import Image


def decode_camera(payload, expected_width=0, expected_height=0):
    if not isinstance(payload, dict) or set(payload) != {"header", "format", "data_b64"}:
        raise ValueError("invalid_camera_packet")
    text = payload["data_b64"]
    if not isinstance(text, str) or len(text) > 1_400_000:
        raise ValueError("camera_payload_too_large")
    try:
        data = base64.b64decode(text, validate=True)
        with Image.open(io.BytesIO(data)) as probe:
            width, height = probe.size
            if probe.format not in ("JPEG", "PNG") or probe.mode != "RGB":
                raise ValueError("camera_requires_compressed_RGB")
            if not 16 <= width <= 4096 or not 16 <= height <= 2160:
                raise ValueError("unsupported_camera_dimensions")
            if (expected_width and width != expected_width) or (expected_height and height != expected_height):
                raise ValueError("camera_resolution_differs_from_calibration")
        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.shape != (height, width, 3):
            raise ValueError("camera_decode_failed")
        return frame
    except (OSError, TypeError, binascii.Error, cv2.error, Image.DecompressionBombError) as exc:
        raise ValueError("invalid_compressed_camera_data") from exc
