"""Raw image pixels to a measured flat ground plane; independent of ROS.

The output frame has x forward, y left, z up. Optical camera axes are x right,
y down, z forward. This is geometry, not a traversability/footprint check.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray

    @classmethod
    def from_mapping(cls, data):
        if not isinstance(data, dict):
            raise ValueError("camera YAML must contain a mapping")
        width, height = data.get("image_width"), data.get("image_height")
        if any(type(v) is not int or v < 16 for v in (width, height)):
            raise ValueError("camera YAML requires integer image_width/image_height >= 16")
        model = data.get("distortion_model")
        if model not in ("plumb_bob", "rational_polynomial"):
            raise ValueError("only pinhole plumb_bob/rational_polynomial calibration is supported")
        try:
            matrix = np.asarray(data["camera_matrix"]["data"], dtype=float).reshape(3, 3)
            distortion = np.asarray(data["distortion_coefficients"]["data"], dtype=float).reshape(-1)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid camera_matrix/distortion_coefficients in camera YAML") from exc
        if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or
                matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or
                not np.allclose(matrix[2], [0, 0, 1]) or
                not np.allclose([matrix[0, 1], matrix[1, 0]], 0) or
                not (0 <= matrix[0, 2] < width and 0 <= matrix[1, 2] < height)):
            raise ValueError("camera intrinsics must be finite, positive and centered inside the image")
        if len(distortion) not in ({4, 5} if model == "plumb_bob" else {8}):
            raise ValueError("expected 4/5 plumb_bob or 8 rational_polynomial distortion coefficients")
        return cls(width, height, matrix, distortion)

    @classmethod
    def load(cls, filename):
        import yaml
        return cls.from_mapping(yaml.safe_load(Path(filename).expanduser().read_text()))


@dataclass(frozen=True)
class CameraMount:
    height_m: float
    pitch_down_deg: float
    roll_deg: float
    yaw_deg: float
    camera_x_m: float
    camera_y_m: float
    ground_frame: str
    camera_frame: str

    @classmethod
    def from_mapping(cls, data):
        keys = set(cls.__dataclass_fields__)
        if not isinstance(data, dict) or set(data) != keys:
            raise ValueError("mount JSON requires exactly: " + ", ".join(sorted(keys)))
        numeric = keys - {"ground_frame", "camera_frame"}
        if any(type(data[k]) not in (int, float) or not math.isfinite(data[k]) for k in numeric):
            raise ValueError("measure the mount first: all distances/angles must be finite numbers (no nulls)")
        if data["height_m"] <= 0 or abs(data["pitch_down_deg"]) >= 90:
            raise ValueError("height_m must be positive; pitch_down_deg must be between -90 and 90")
        for key in ("ground_frame", "camera_frame"):
            if not isinstance(data[key], str) or not data[key] or any(c.isspace() for c in data[key]) or data[key].startswith("/"):
                raise ValueError(f"{key} must be a nonempty TF frame without spaces or leading slash")
        if data["ground_frame"] == data["camera_frame"]:
            raise ValueError("ground_frame and camera_frame must differ")
        return cls(**data)

    @classmethod
    def load(cls, filename):
        return cls.from_mapping(json.loads(Path(filename).expanduser().read_text()))

    @property
    def origin(self):
        return np.array([self.camera_x_m, self.camera_y_m, self.height_m], dtype=float)

    @property
    def rotation(self):
        roll, pitch, yaw = np.deg2rad([self.roll_deg, self.pitch_down_deg, self.yaw_deg])
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        optical_to_level = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        return rz @ ry @ rx @ optical_to_level

    @property
    def quaternion_xyzw(self):
        # Unit quaternion of Rz(yaw) Ry(pitch) Rx(roll) R_optical_to_level.
        roll, pitch, yaw = np.deg2rad([self.roll_deg, self.pitch_down_deg, self.yaw_deg]) / 2
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        x, y, z, w = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
        # Hamilton product with fixed optical quaternion (-.5, .5, -.5, .5).
        return np.array([-.5*w+.5*x-.5*y-.5*z, .5*w+.5*x+.5*y-.5*z,
                         -.5*w+.5*x+.5*y+.5*z, .5*w+.5*x-.5*y+.5*z])


def project_path(pixels, processed_shape_hw, input_shape_hw, calibration, mount,
                 *, max_distance_m=5.0, max_segment_m=0.5):
    """Project a contiguous near-to-far prefix; never join across invalid rays.

    Intrinsics refer to RAW camera input, not model tensors or the display.
    Resize uses OpenCV's half-pixel center convention. Wrong resolution, frame,
    age and non-flat terrain must not be silently treated as calibrated data.
    """
    for shape in (processed_shape_hw, input_shape_hw):
        if not isinstance(shape, (list, tuple)) or len(shape) != 2 or any(type(v) is not int or v < 16 for v in shape):
            raise ValueError("image shapes must be [height, width] integers >= 16")
    if list(input_shape_hw) != [calibration.height, calibration.width]:
        raise ValueError("camera resolution differs from calibration; use the calibrated capture size")
    if any(not math.isfinite(v) or v <= 0 for v in (max_distance_m, max_segment_m)):
        raise ValueError("projection limits must be finite and positive")
    points = np.asarray(pixels, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("path must contain at least two finite [u, v] points")
    h, w = processed_shape_hw
    if (points < 0).any() or (points[:, 0] > w - 1).any() or (points[:, 1] > h - 1).any():
        raise ValueError("pixel path lies outside processed image")
    if (np.diff(points[:, 1]) > 0).any():
        raise ValueError("pixel path must be ordered near-to-far (nonincreasing image v)")
    raw = (points + 0.5) * np.array([calibration.width / w, calibration.height / h]) - 0.5
    try:
        normalized = cv2.undistortPoints(raw.reshape(-1, 1, 2), calibration.matrix, calibration.distortion).reshape(-1, 2)
    except cv2.error as exc:
        raise ValueError("camera distortion could not be applied") from exc
    rays = np.column_stack([normalized, np.ones(len(points))]) @ mount.rotation.T
    projected = []
    reason = "ok"
    for ray in rays:
        if not np.isfinite(ray).all() or ray[2] >= -1e-6:
            reason = "horizon_clipped"
            break
        hit = mount.origin - (mount.height_m / ray[2]) * ray
        hit[2] = 0.0
        if hit[0] <= 0 or np.linalg.norm(hit[:2]) > max_distance_m:
            reason = "range_clipped"
            break
        if projected and np.linalg.norm(hit - projected[-1]) > max_segment_m:
            reason = "segment_gap_clipped"
            break
        if not projected or np.linalg.norm(hit - projected[-1]) > 1e-6:
            projected.append(hit)
    if len(projected) < 2:
        return np.empty((0, 3)), "insufficient_ground_path:" + reason
    return np.asarray(projected), reason


def ground_to_image(points_xy, calibration, mount):
    """Project known Z=0 ground points into RAW, distorted camera pixels.

    Return pixels and an in-image mask. Points behind the camera have NaN
    pixels and are never visible. This direction is for measured-floor checks,
    not an assertion that any projected surface is traversable.
    """
    points = np.asarray(points_xy, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or not 1 <= len(points) <= 10000 or
            not np.isfinite(points).all()):
        raise ValueError("ground points must contain 1..10000 finite [x, y] pairs")
    world = np.column_stack([points, np.zeros(len(points))])
    camera_points = (world - mount.origin) @ mount.rotation
    front = camera_points[:, 2] > 1e-6
    pixels = np.full((len(points), 2), np.nan)
    if front.any():
        try:
            projected, _ = cv2.projectPoints(camera_points[front], np.zeros(3), np.zeros(3),
                                             calibration.matrix, calibration.distortion)
        except cv2.error as exc:
            raise ValueError("ground points could not be projected through camera calibration") from exc
        pixels[front] = projected.reshape(-1, 2)
    visible = (front & np.isfinite(pixels).all(axis=1) & (pixels[:, 0] >= 0) &
               (pixels[:, 0] <= calibration.width-1) & (pixels[:, 1] >= 0) &
               (pixels[:, 1] <= calibration.height-1))
    return pixels, visible
