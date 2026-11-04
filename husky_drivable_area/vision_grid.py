"""Encode/decode current segmentation labels: 1 drivable, 0 blocked, 255 unknown."""
import base64
import json
import cv2
import numpy as np

def encode_segmentation(result, header, horizon_fraction):
    labels = (result.raw_mask > 0).astype(np.uint8)
    labels[:int(labels.shape[0]*horizon_fraction)] = 255  # not analyzed, not blocked
    ok, encoded = cv2.imencode(".png", labels)
    if not ok:
        raise ValueError("segmentation_png_failed")
    return json.dumps(dict(protocol=1, valid=True, source_header=dict(frame_id=header.frame_id,
        stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec)),
        raw_shape_hw=list(result.input_shape), shape_hw=list(labels.shape),
        png_b64=base64.b64encode(encoded).decode("ascii")), allow_nan=False)


def decode_segmentation(text):
    if not isinstance(text, str) or len(text) > 2_000_000:
        raise ValueError("oversized_segmentation_packet")
    packet = json.loads(text)
    if not isinstance(packet, dict) or packet.get("protocol") != 1 or packet.get("valid") is not True:
        raise ValueError("invalid_segmentation_packet")
    for key in ("raw_shape_hw", "shape_hw"):
        size = packet[key]
        if (not isinstance(size, list) or len(size) != 2 or
                any(type(v) is not int or not 16 <= v <= 4096 for v in size) or np.prod(size) > 1_000_000):
            raise ValueError("invalid_segmentation_dimensions")
    encoded = base64.b64decode(packet["png_b64"], validate=True)
    if len(encoded) < 24 or encoded[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("segmentation_requires_png")
    width, height = int.from_bytes(encoded[16:20], "big"), int.from_bytes(encoded[20:24], "big")
    if [height, width] != packet["shape_hw"]:
        raise ValueError("segmentation_png_dimensions_differ")
    labels = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_UNCHANGED)
    if labels is None or labels.ndim != 2 or labels.dtype != np.uint8 or not np.isin(labels, [0, 1, 255]).all():
        raise ValueError("invalid_segmentation_labels")
    return packet, labels
