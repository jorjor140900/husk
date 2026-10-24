"""Read-only perception-to-Pure-Pursuit preview at an assumed stationary origin.

There is no odometry, collision clearance, permission, or motor command here.
The reference frame's origin is the bench reference, not a measured robot pose.
"""

from dataclasses import dataclass, replace
import math

import numpy as np

from .control import Demand, pursuit
from .ground_geometry import project_path


@dataclass(frozen=True)
class VisionPreview:
    reason: str
    path: np.ndarray
    demand: Demand | None = None
    source_ns: int | None = None
    age_ms: float | None = None
    projection_reason: str | None = None
    detail: str | None = None
    control_fresh: bool = False
    image_path_valid: bool | None = None
    detection_reason: str | None = None

    def metadata(self):
        candidate = None
        if self.control_fresh and self.demand is not None and self.demand.reason in ("tracking", "goal_reached"):
            candidate = {"linear_x_mps": self.demand.linear, "angular_z_rps": self.demand.angular}
        return {"preview_only": True, "ready_for_control": False, "footprint_checked": False,
                "assumption": "stationary_origin_in_calibrated_ground_frame; no_odometry",
                "reason": self.reason, "projection_reason": self.projection_reason,
                "detail": self.detail,
                "valid_image_path": self.image_path_valid, "detection_reason": self.detection_reason,
                "source_fresh_for_control": self.control_fresh,
                "display_state": ("fresh" if self.control_fresh else "stale_for_control") if len(self.path)
                                 else "expired" if self.reason == "display_expired" else "no_ground_path",
                "path_start_ground_xy_m": self.path[0, :2].tolist() if len(self.path) else None,
                "controller_reason": self.demand.reason if self.demand is not None else None,
                "distance_to_reference_m": self.demand.cross_track_m if self.demand is not None else None,
                "target_in_assumed_frame_xy_m": self.demand.target_robot_xy if self.demand is not None else None,
                "candidate_before_acceleration_limits": candidate, "frame_age_ms": self.age_ms,
                "source_stamp_ns": self.source_ns, "ground_path_points": len(self.path)}


def _validate_timeouts(max_age_sec, display_max_age_sec):
    if (not math.isfinite(max_age_sec) or max_age_sec <= 0 or
            not math.isfinite(display_max_age_sec) or display_max_age_sec < max_age_sec):
        raise ValueError("display age limit must be finite and >= the positive controller age limit")


def age_preview(result, now_ros_ns, max_age_sec=0.5, display_max_age_sec=3.0, *, receipt_age_sec=0.0):
    """Expire a command candidate promptly, keeping bounded diagnostic geometry.

    This is a visualization policy only. A delayed image must never become a
    candidate merely because its projected path is still useful to inspect.
    Invalid new detections replace the previous result; no path is filled in.
    """
    _validate_timeouts(max_age_sec, display_max_age_sec)
    age_ms = (now_ros_ns-result.source_ns)/1e6 if result.source_ns is not None else None
    if receipt_age_sec > display_max_age_sec or (age_ms is not None and age_ms > display_max_age_sec*1000):
        return replace(result, reason="display_expired", path=np.empty((0, 3)), demand=None,
                       age_ms=age_ms, control_fresh=False)
    if result.source_ns is None:
        return replace(result, control_fresh=False)
    if result.source_ns <= 0 or age_ms < -50:
        return replace(result, reason="stale_or_invalid_source_timestamp", path=np.empty((0, 3)), demand=None,
                       age_ms=age_ms, control_fresh=False)
    fresh = age_ms <= max_age_sec*1000 and receipt_age_sec <= max_age_sec
    reason = result.reason
    if len(result.path) and result.demand is not None:
        reason = ("candidate_available" if result.demand.reason == "tracking" else "controller:" + result.demand.reason) if fresh else "stale_for_control"
    return replace(result, reason=reason, age_ms=age_ms, control_fresh=fresh)


def preview_detection(data, calibration, mount, config, now_ros_ns, max_age_sec=0.5, display_max_age_sec=3.0):
    empty = np.empty((0, 3))
    _validate_timeouts(max_age_sec, display_max_age_sec)
    if not isinstance(data, dict):
        return VisionPreview("invalid_detection_message", empty)
    context = {"image_path_valid": data.get("valid_image_path") is True, "detection_reason": str(data.get("reason"))}
    if data.get("valid_image_path") is not True:
        return VisionPreview("image_path_invalid:" + str(data.get("reason")), empty, **context)
    source_ns = age_ms = None
    try:
        source = data["source_header"]
        if source["frame_id"] != mount.camera_frame:
            return VisionPreview("source_frame_mismatch", empty, **context)
        sec, ns = source["stamp"]["sec"], source["stamp"]["nanosec"]
        if type(sec) is not int or type(ns) is not int or not 0 <= ns < 1_000_000_000:
            return VisionPreview("invalid_source_timestamp", empty, **context)
        source_ns = sec * 1_000_000_000 + ns
        age_ms = (now_ros_ns-source_ns)/1e6
        if source_ns <= 0 or age_ms < -50 or age_ms > display_max_age_sec*1000:
            reason = "display_expired" if source_ns > 0 and age_ms > display_max_age_sec*1000 else "stale_or_invalid_source_timestamp"
            return VisionPreview(reason, empty, source_ns=source_ns, age_ms=age_ms, **context)
        if data.get("path_coordinate_system") != "processed_image_pixels_xy_near_to_far":
            return VisionPreview("unsupported_pixel_coordinates", empty, source_ns=source_ns, age_ms=age_ms, **context)
        points, projection_reason = project_path(data["path"], data["processed_shape_hw"], data["input_shape_hw"],
            calibration, mount, max_distance_m=5.0, max_segment_m=config.max_segment_m)
        if len(points) < 2:
            return VisionPreview("no_ground_path", empty, source_ns=source_ns, age_ms=age_ms, projection_reason=projection_reason, **context)
        demand = pursuit(points[:, :2], [0.0, 0.0, 0.0], config)
        reason = "candidate_available" if demand.reason == "tracking" else "controller:" + demand.reason
        result = VisionPreview(reason, points, demand, source_ns, age_ms, projection_reason, **context)
        return age_preview(result, now_ros_ns, max_age_sec, display_max_age_sec)
    except (ValueError, KeyError, TypeError, OverflowError) as exc:
        return VisionPreview("invalid_detection_or_calibration_geometry", empty, source_ns=source_ns, age_ms=age_ms, detail=str(exc), **context)
