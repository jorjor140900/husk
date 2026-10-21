"""ROS-independent baseline with an optional external segmentation input.

RGB alone supplies appearance evidence. Similar-looking walls, curbs and ground
can still be confused. This module does not certify a surface as safe to drive.
"""

from dataclasses import asdict, dataclass
import math
import time

import cv2
import numpy as np

from .planner import plan_path
from .reference_geometry import ReferenceGeometry, build_reference_geometry


@dataclass(frozen=True)
class DetectorConfig:
    max_width: int = 640
    analysis_width: int = 320
    horizon_fraction: float = 0.30
    seed_x_fraction: float = 0.5
    seed_y_fraction: float = 0.94
    goal_x_fraction: float = 0.5
    robot_center_x_fraction: float = 0.5
    lookahead_y_fraction: float = 0.65
    chroma_tolerance: float = 14.0
    probability_threshold: float = 0.62
    current_probability_floor: float = 0.50
    temporal_alpha: float = 0.65
    use_temporal: bool = True
    clearance_px: int = 8
    max_lateral_step: int = 2
    min_path_height_fraction: float = 0.25

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not isinstance(value, (int, float, bool)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
        for name in ("max_width", "analysis_width", "clearance_px", "max_lateral_step"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if type(self.use_temporal) is not bool:
            raise ValueError("use_temporal must be boolean")
        if self.max_width < 64 or not 32 <= self.analysis_width <= self.max_width:
            raise ValueError("require max_width >= 64 and 32 <= analysis_width <= max_width")
        if not 0 <= self.horizon_fraction < self.seed_y_fraction <= 1:
            raise ValueError("require 0 <= horizon_fraction < seed_y_fraction <= 1")
        if not 0 <= self.seed_x_fraction <= 1 or not 0 <= self.goal_x_fraction <= 1:
            raise ValueError("seed_x_fraction and goal_x_fraction must be in [0, 1]")
        if not 0 <= self.robot_center_x_fraction <= 1 or not 0 <= self.lookahead_y_fraction <= 1:
            raise ValueError("robot_center_x_fraction and lookahead_y_fraction must be in [0, 1]")
        if self.chroma_tolerance <= 0:
            raise ValueError("chroma_tolerance must be positive")
        if not 0 < self.current_probability_floor <= self.probability_threshold < 1:
            raise ValueError("require 0 < current_probability_floor <= probability_threshold < 1")
        if not 0 < self.temporal_alpha <= 1:
            raise ValueError("temporal_alpha must be in (0, 1]")
        if self.clearance_px < 0 or not 1 <= self.max_lateral_step <= 10:
            raise ValueError("invalid clearance_px or max_lateral_step")
        if not 0 < self.min_path_height_fraction <= 1:
            raise ValueError("min_path_height_fraction must be in (0, 1]")


@dataclass
class DetectionResult:
    frame_bgr: np.ndarray
    probability: np.ndarray
    raw_mask: np.ndarray
    road_mask: np.ndarray
    navigable_mask: np.ndarray
    path: np.ndarray
    quality_score: float
    reason: str
    backend: str
    processing_ms: float
    input_shape: tuple[int, int]
    reference_geometry: ReferenceGeometry | None = None

    @property
    def valid_image_path(self) -> bool:
        return len(self.path) > 0

    def metadata(self) -> dict:
        return {
            "valid_image_path": self.valid_image_path,
            "reason": self.reason,
            "backend": self.backend,
            "quality_score": self.quality_score,
            "processing_ms": self.processing_ms,
            "input_shape_hw": list(self.input_shape),
            "processed_shape_hw": list(self.frame_bgr.shape[:2]),
            "path_coordinate_system": "processed_image_pixels_xy_near_to_far",
            "path": self.path.tolist(),
            "reference_geometry": self.reference_geometry.metadata() if self.reference_geometry else None,
        }


class RoadDetector:
    def __init__(self, config: DetectorConfig | None = None):
        self.config = config or DetectorConfig()
        self.reset()

    def reset(self):
        """Start a new independent clip; do not carry its history into another."""
        self._previous_gray = None
        self._previous_probability = None
        self._previous_path = None

    def _appearance_probability(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        scale = min(1.0, self.config.analysis_width / w)
        small = cv2.resize(frame, (max(1, round(w * scale)), max(1, round(h * scale))), interpolation=cv2.INTER_AREA)
        lab = cv2.cvtColor(cv2.bilateralFilter(small, 5, 30, 30), cv2.COLOR_BGR2LAB).astype(np.float32)
        sh, sw = lab.shape[:2]
        radius = max(1, round(sw * 0.025))
        sy = int(round((sh - 1) * self.config.seed_y_fraction))
        center = int(round((sw - 1) * self.config.seed_x_fraction))
        refs = []
        for dx in (0, -2 * radius, 2 * radius):
            sx = int(np.clip(center + dx, 0, sw - 1))
            patch = lab[max(0, sy - radius):min(sh, sy + radius + 1), max(0, sx - radius):min(sw, sx + radius + 1)]
            ref = np.median(patch.reshape(-1, 3), axis=0)
            # Do not automatically learn a different surface beside the center.
            if not refs or np.linalg.norm(ref[1:] - refs[0][1:]) <= self.config.chroma_tolerance:
                refs.append(ref)

        best_distance = np.full((sh, sw), np.inf, dtype=np.float32)
        for ref in refs:
            chroma_distance = np.sum(((lab[:, :, 1:] - ref[1:]) / self.config.chroma_tolerance) ** 2, axis=2)
            # A brightness change is weaker evidence than a material/color change.
            # It is deliberately not a hard Canny barrier across shadows/seams.
            light_distance = np.maximum(np.abs(lab[:, :, 0] - ref[0]) - 40.0, 0.0) / 90.0
            best_distance = np.minimum(best_distance, chroma_distance + light_distance ** 2)
        probability = np.exp(-0.5 * best_distance)
        return cv2.resize(probability, (w, h), interpolation=cv2.INTER_LINEAR)

    def _filter_history(self, probability: np.ndarray, gray: np.ndarray):
        preferred = None
        cfg = self.config
        if not cfg.use_temporal or self._previous_gray is None or self._previous_gray.shape != gray.shape:
            return probability.copy(), preferred
        # Backward flow maps each CURRENT pixel into the PREVIOUS image.
        flow = cv2.calcOpticalFlowFarneback(gray, self._previous_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        yy, xx = np.indices(gray.shape, dtype=np.float32)
        mx, my = xx + flow[:, :, 0], yy + flow[:, :, 1]
        inside = (mx >= 0) & (my >= 0) & (mx <= gray.shape[1] - 1) & (my <= gray.shape[0] - 1)
        old_gray = cv2.remap(self._previous_gray, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        consistent = inside & (np.abs(gray.astype(np.float32) - old_gray) < 25)
        # Discard history when correspondence is poor, including scene changes.
        if consistent.mean() < 0.55:
            return probability.copy(), preferred
        old_probability = cv2.remap(self._previous_probability, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        filtered = probability.copy()
        filtered[consistent] = (
            cfg.temporal_alpha * probability[consistent]
            + (1.0 - cfg.temporal_alpha) * old_probability[consistent]
        )
        # Current negative evidence wins immediately; old masks cannot reopen it.
        filtered[probability < cfg.current_probability_floor] = probability[probability < cfg.current_probability_floor]
        if self._previous_path is not None:
            preferred = cv2.remap(self._previous_path, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            preferred[~consistent] = 0
        return filtered, preferred

    def process(
        self,
        frame_bgr: np.ndarray,
        *,
        drivable_probability: np.ndarray | None = None,
        blocked_mask: np.ndarray | None = None,
    ) -> DetectionResult:
        """Process uint8 BGR without ROS, GUI windows or model downloads.

        Optional AI probabilities must be finite [0, 1] arrays matching the
        ORIGINAL frame's H,W. A blocked mask is an authoritative veto; any
        nonzero pixel blocks that location, including during downsampling.
        Results use a resized image (at most max_width), never metric units.
        """
        start = time.perf_counter()
        if not isinstance(frame_bgr, np.ndarray) or frame_bgr.dtype != np.uint8 or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 uint8 BGR image")
        original_shape = frame_bgr.shape[:2]
        if min(original_shape) < 16:
            raise ValueError("frame dimensions must each be at least 16 pixels")
        scale = min(1.0, self.config.max_width / original_shape[1])
        size = (max(1, round(original_shape[1] * scale)), max(1, round(original_shape[0] * scale)))
        frame = cv2.resize(frame_bgr, size, interpolation=cv2.INTER_AREA)
        h, w = frame.shape[:2]

        if drivable_probability is None:
            current = self._appearance_probability(frame)
            backend = "appearance_baseline"
        else:
            supplied = np.asarray(drivable_probability)
            if supplied.shape != original_shape or not np.isfinite(supplied).all() or np.any((supplied < 0) | (supplied > 1)):
                raise ValueError("drivable_probability must match original H,W and contain finite values in [0, 1]")
            current = cv2.resize(supplied.astype(np.float32), size, interpolation=cv2.INTER_AREA)
            backend = "external_probability"

        blocked = np.zeros((h, w), dtype=bool)
        if blocked_mask is not None:
            supplied = np.asarray(blocked_mask)
            if supplied.shape != original_shape or not np.isfinite(supplied).all():
                raise ValueError("blocked_mask must be finite and match original H,W")
            # Preserve even a subpixel blocked region when reducing a 2K image.
            blocked = cv2.resize((supplied != 0).astype(np.float32), size, interpolation=cv2.INTER_AREA) > 0
        y_top = int(h * self.config.horizon_fraction)
        current[:y_top] = 0
        current[blocked] = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        probability, preferred = self._filter_history(current, gray)
        probability[:y_top] = 0
        probability[blocked] = 0
        candidate = (probability >= self.config.probability_threshold) & (current >= self.config.current_probability_floor)
        raw_mask = candidate.astype(np.uint8) * 255
        sx = int(round((w - 1) * self.config.seed_x_fraction))
        sy = int(round((h - 1) * self.config.seed_y_fraction))
        road_mask = np.zeros((h, w), dtype=np.uint8)
        if candidate[sy, sx]:
            _, components = cv2.connectedComponents(candidate.astype(np.uint8), connectivity=4)
            road_mask[components == components[sy, sx]] = 255

        cfg = self.config
        plan = plan_path(
            road_mask, probability,
            seed_x=cfg.seed_x_fraction, seed_y=cfg.seed_y_fraction,
            horizon=cfg.horizon_fraction, goal_x=cfg.goal_x_fraction,
            clearance_px=cfg.clearance_px, max_lateral_step=cfg.max_lateral_step,
            min_path_height=cfg.min_path_height_fraction, preferred_path=preferred,
        )
        quality = 0.0
        if plan.valid:
            px, py = plan.points.T
            span = float(py[0] - py[-1]) / h
            quality = float(probability[py, px].mean() * min(1.0, span / 0.5))
        path_mask = np.zeros((h, w), dtype=np.uint8)
        if plan.valid:
            cv2.polylines(path_mask, [plan.points], False, 255, 3)
        self._previous_gray = gray
        self._previous_probability = probability.copy()
        self._previous_path = path_mask
        reference = build_reference_geometry(
            road_mask, plan.points,
            robot_center_x_fraction=cfg.robot_center_x_fraction,
            lookahead_y_fraction=cfg.lookahead_y_fraction,
            horizon_fraction=cfg.horizon_fraction,
        )
        return DetectionResult(
            frame, probability, raw_mask, road_mask, plan.navigable_mask,
            plan.points, quality, plan.reason, backend,
            (time.perf_counter() - start) * 1000.0, original_shape, reference,
        )
