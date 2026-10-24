"""Drawing helpers; importing this module never opens a window."""

import cv2
import numpy as np

from .detector import DetectionResult
from .reference_geometry import build_reference_geometry


def _draw_boundary(image, line):
    if line is None:
        return
    h, w = image.shape[:2]
    top = (int(round(line.x_at(line.y_top))), line.y_top)
    bottom = (int(round(line.x_at(line.y_bottom))), line.y_bottom)
    visible, start, end = cv2.clipLine((0, 0, w, h), top, bottom)
    if visible:
        cv2.line(image, start, end, (0, 0, 255), 2, cv2.LINE_AA)


def draw_result(result: DetectionResult, telemetry=None) -> np.ndarray:
    image = result.frame_bgr.copy()
    green = np.full_like(image, (0, 200, 0))
    road = result.road_mask > 0
    image[road] = cv2.addWeighted(image, 0.65, green, 0.35, 0)[road]
    contours, _ = cv2.findContours(result.navigable_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (255, 220, 0), 1)
    reference = result.reference_geometry or build_reference_geometry(result.road_mask, result.path)
    _draw_boundary(image, reference.left)
    _draw_boundary(image, reference.right)
    # Blue is an assumed robot/camera axis, even when perception has no path.
    # Draw it wider than the yellow reference so alignment remains visible.
    cv2.line(
        image, (reference.robot_x, reference.y_bottom),
        (reference.robot_x, reference.y_top), (255, 0, 0), 3, cv2.LINE_AA,
    )
    if reference.target_x is not None:
        target = (int(round(reference.target_x)), reference.lookahead_y)
        axis = (reference.robot_x, reference.lookahead_y)
        cv2.line(image, axis, target, (240, 240, 240), 1, cv2.LINE_AA)
        cv2.circle(image, axis, 6, (255, 0, 0), 2, cv2.LINE_AA)
    if result.valid_image_path:
        cv2.polylines(image, [result.path], False, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(image, tuple(result.path[-1]), 5, (0, 255, 255), -1)
        if reference.target_x is not None:
            cv2.circle(image, target, 4, (0, 255, 255), -1, cv2.LINE_AA)
    state = "PATH IN MASK" if result.valid_image_path else "NO PATH"
    cv2.rectangle(image, (0, 0), (image.shape[1], 87 if telemetry else 66), (20, 20, 20), -1)
    cv2.putText(image, f"{state}: {result.reason}", (7, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(image, f"{result.backend} | RGB mask is NOT verified traversability", (7, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    offset = f"dx {reference.lateral_error_px:+.1f}px" if reference.lateral_error_px is not None else "dx n/a"
    legend = f"Yellow=reference  Blue=robot  Red=edges | {offset}"
    scale = min(0.42, max(0.25, image.shape[1] / 1500))
    cv2.putText(image, legend, (7, 58), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1)
    if telemetry:
        elapsed = telemetry["processing_ms"]
        age = telemetry["host_frame_age_ms"]
        timing = f"Compute {elapsed:.0f}ms | {1000 / max(elapsed, .001):.1f} proc FPS | host age {age:.0f}ms"
        cv2.putText(image, timing, (7, 79), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
    return image

