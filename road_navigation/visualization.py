"""Drawing helpers; importing this module never opens a window."""

import cv2
import numpy as np

from .detector import DetectionResult


def draw_result(result: DetectionResult) -> np.ndarray:
    image = result.frame_bgr.copy()
    green = np.full_like(image, (0, 200, 0))
    road = result.road_mask > 0
    image[road] = cv2.addWeighted(image, 0.65, green, 0.35, 0)[road]
    contours, _ = cv2.findContours(result.navigable_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (255, 220, 0), 1)
    if result.valid_image_path:
        cv2.polylines(image, [result.path], False, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(image, tuple(result.path[-1]), 5, (0, 255, 255), -1)
    state = "IMAGE PATH" if result.valid_image_path else "NO PATH"
    cv2.rectangle(image, (0, 0), (image.shape[1], 47), (20, 20, 20), -1)
    cv2.putText(image, f"{state}: {result.reason}", (7, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(image, f"{result.backend} | quality {result.quality_score:.2f} (heuristic)", (7, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    return image

