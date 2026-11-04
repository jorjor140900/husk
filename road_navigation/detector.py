"""SegFormer probability filtering for camera steering assistance.

RGB/model labels are appearance evidence, not verified physical traversability.
"""

from dataclasses import asdict, dataclass
import math
import time

import cv2
import numpy as np

from .reference_geometry import ReferenceGeometry, build_reference_geometry
from .vegetation import green_vegetation_mask


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
    use_edge_barrier: bool = True
    edge_low_threshold: int = 35
    edge_high_threshold: int = 80
    region_step: int = 6
    probability_threshold: float = 0.62
    probability_release_threshold: float = 0.62
    current_probability_floor: float = 0.50
    enclosed_uncertain_max_pixels: int = 0
    enclosed_uncertain_radius_px: int = 2
    temporal_alpha: float = 0.65
    temporal_flow_width: int = 0
    use_temporal: bool = True
    generate_reference_path: bool = False
    clearance_px: int = 8
    max_lateral_step: int = 2
    min_path_height_fraction: float = 0.25
    reject_green_vegetation: bool = False
    vegetation_hue_min: int = 25
    vegetation_hue_max: int = 95
    vegetation_saturation_min: int = 45
    vegetation_value_min: int = 20
    vegetation_excess_green_min: float = 0.08
    vegetation_dilation_px: int = 2
    vegetation_support_radius_px: int = 0
    vegetation_support_fraction: float = 0.5

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not isinstance(value, (int, float, bool)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
        for name in ("max_width", "analysis_width", "clearance_px", "max_lateral_step", "edge_low_threshold", "edge_high_threshold", "region_step", "temporal_flow_width"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if (type(self.enclosed_uncertain_max_pixels) is not int or not 0<=self.enclosed_uncertain_max_pixels<=256 or
                type(self.enclosed_uncertain_radius_px) is not int or not 1<=self.enclosed_uncertain_radius_px<=3):
            raise ValueError("invalid enclosed uncertain-region limits")
        if type(self.use_temporal) is not bool:
            raise ValueError("use_temporal must be boolean")
        if self.temporal_flow_width!=0 and not 64<=self.temporal_flow_width<=self.max_width:
            raise ValueError('temporal_flow_width must be 0 or within 64..max_width')
        if type(self.generate_reference_path) is not bool:
            raise ValueError("generate_reference_path must be boolean")
        if type(self.use_edge_barrier) is not bool:
            raise ValueError("use_edge_barrier must be boolean")
        if type(self.reject_green_vegetation) is not bool:
            raise ValueError("reject_green_vegetation must be boolean")
        for name in ("vegetation_hue_min", "vegetation_hue_max", "vegetation_saturation_min",
                     "vegetation_value_min", "vegetation_dilation_px", "vegetation_support_radius_px"):
            if type(getattr(self, name)) is not int:
                raise ValueError(name+" must be an integer")
        if (not 0 <= self.vegetation_hue_min <= self.vegetation_hue_max <= 179 or
                not 0 <= self.vegetation_saturation_min <= 255 or not 0 <= self.vegetation_value_min <= 255 or
                not 0 <= self.vegetation_excess_green_min <= 1 or not 0 <= self.vegetation_dilation_px <= 12):
            raise ValueError("invalid vegetation colour-veto settings")
        if (not 0 <= self.vegetation_support_radius_px <= 12 or
                type(self.vegetation_support_fraction) not in (int, float) or
                not 0 < self.vegetation_support_fraction <= 1):
            raise ValueError("invalid vegetation spatial-support settings")
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
        if not 0 <= self.edge_low_threshold < self.edge_high_threshold <= 255:
            raise ValueError("require 0 <= edge_low_threshold < edge_high_threshold <= 255")
        if not 1 <= self.region_step <= 40:
            raise ValueError("region_step must be in 1..40")
        if not 0 < self.current_probability_floor <= self.probability_threshold < 1:
            raise ValueError("require 0 < current_probability_floor <= probability_threshold < 1")
        if not self.current_probability_floor <= self.probability_release_threshold <= self.probability_threshold:
            raise ValueError('probability_release_threshold must be within current floor..entry threshold')
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
    vegetation_mask: np.ndarray | None = None
    vegetation_veto_enabled: bool = False
    reference_path_enabled: bool = True
    vegetation_support_radius_px: int = 0
    vegetation_support_fraction: float = 0.5

    enclosed_uncertain_pixels: int = 0

    @property
    def valid_image_path(self) -> bool:
        return len(self.path) > 0

    def metadata(self) -> dict:
        return {
            "valid_image_path": self.valid_image_path,
            "drivable_pixels": int(np.count_nonzero(self.raw_mask)),
            "mask_pixels": int(self.raw_mask.size),
            "image_reference_enabled": self.reference_path_enabled,
            "enclosed_uncertain_pixels": self.enclosed_uncertain_pixels,
            "reason": self.reason,
            "backend": self.backend,
            "quality_score": self.quality_score,
            "quality_score_is_safety_confidence": False,
            "processing_ms": self.processing_ms,
            "input_shape_hw": list(self.input_shape),
            "processed_shape_hw": list(self.frame_bgr.shape[:2]),
            "path_coordinate_system": "processed_image_pixels_xy_near_to_far",
            "path": self.path.tolist(),
            "reference_geometry": self.reference_geometry.metadata() if self.reference_geometry else None,
            "vegetation_veto": {
                "enabled": self.vegetation_veto_enabled,
                "blocked_pixels": int(np.count_nonzero(self.vegetation_mask)) if self.vegetation_mask is not None else 0,
                "method": "raw_bgr_green_colour",
                "support_radius_px": self.vegetation_support_radius_px,
                "support_fraction": self.vegetation_support_fraction,
                "verified_grass_classification": False,
            },
        }


def connect_enclosed_uncertainty(candidate,current,filtered,blocked,config):
    """Connect only small enclosed threshold gaps still supported by the model.

    Never override confident negatives, explicit vetoes, vegetation, large
    holes, or regions connected to the image boundary. This is not a detector
    of floor markings: training remains necessary for confident model errors.
    """
    if not config.enclosed_uncertain_max_pixels:return candidate,0
    count,components,stats,_=cv2.connectedComponentsWithStats((~candidate).astype(np.uint8),connectivity=8)
    result=candidate.copy();added=0
    h,w=candidate.shape
    for index in range(1,count):
        x,y,width,height,area=stats[index]
        if area>config.enclosed_uncertain_max_pixels or x==0 or y==0 or x+width>=w or y+height>=h:continue
        # Work on a bounded crop: the accepted component must be entirely
        # surrounded by drivable pixels, not just adjacent to a green patch.
        region=components[y-1:y+height+1,x-1:x+width+1]==index
        pad=cv2.dilate(region.astype(np.uint8),np.ones((3,3),np.uint8))>0
        ring=pad&~region
        sl=np.s_[y-1:y+height+1,x-1:x+width+1]
        if not candidate[sl][ring].all():continue
        if blocked[sl][region].any():continue
        if np.any(current[sl][region]<config.current_probability_floor):continue
        if np.any(filtered[sl][region]<config.current_probability_floor):continue
        thickness=cv2.distanceTransform(region.astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
        if float(thickness.max())>config.enclosed_uncertain_radius_px:continue
        result[sl]|=region;added+=int(area)
    return result,added


class RoadDetector:
    def __init__(self, config: DetectorConfig | None = None):
        self.config = config or DetectorConfig()
        if self.config.generate_reference_path:
            raise ValueError("Use the metric camera-assist planner, not pixel path generation")
        self.reset()

    def reset(self):
        """Start a new independent clip; do not carry its history into another."""
        self._previous_gray = None
        self._previous_probability = None
        self._previous_path = None
        self._previous_mask = None
        self._aligned_previous_mask = None

    def _filter_history(self, probability: np.ndarray, gray: np.ndarray):
        preferred = None
        self._aligned_previous_mask = None
        cfg = self.config
        if not cfg.use_temporal or self._previous_gray is None or self._previous_gray.shape != gray.shape:
            return probability.copy(), preferred
        # Backward flow maps each CURRENT pixel into the PREVIOUS image.
        width=min(cfg.temporal_flow_width or gray.shape[1],gray.shape[1])
        if width<gray.shape[1]:
            size=(width,max(16,round(gray.shape[0]*width/gray.shape[1])))
            current_small=cv2.resize(gray,size,interpolation=cv2.INTER_AREA)
            previous_small=cv2.resize(self._previous_gray,size,interpolation=cv2.INTER_AREA)
            flow=cv2.calcOpticalFlowFarneback(current_small,previous_small,None,.5,3,15,3,5,1.2,0)
            flow=cv2.resize(flow,(gray.shape[1],gray.shape[0]),interpolation=cv2.INTER_LINEAR)
            flow*=np.array([gray.shape[1]/size[0],gray.shape[0]/size[1]],np.float32)
        else:
            flow=cv2.calcOpticalFlowFarneback(gray,self._previous_gray,None,.5,3,15,3,5,1.2,0)
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
        if self._previous_mask is not None:
            self._aligned_previous_mask = cv2.remap(self._previous_mask,mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)>0
            self._aligned_previous_mask[~consistent] = False
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
            raise ValueError("camera assist requires SegFormer probabilities")
        supplied = np.asarray(drivable_probability)
        if supplied.shape != original_shape or not np.isfinite(supplied).all() or np.any((supplied < 0) | (supplied > 1)):
            raise ValueError("drivable_probability must match original H,W and contain finite values in [0, 1]")
        current = cv2.resize(supplied.astype(np.float32), size, interpolation=cv2.INTER_AREA)
        backend = "external_probability"

        vegetation = np.zeros((h, w), dtype=bool)
        if self.config.reject_green_vegetation:
            cfg = self.config
            vegetation = green_vegetation_mask(frame_bgr, hue_min=cfg.vegetation_hue_min,
                hue_max=cfg.vegetation_hue_max, saturation_min=cfg.vegetation_saturation_min,
                value_min=cfg.vegetation_value_min, excess_green_min=cfg.vegetation_excess_green_min,
                dilation_px=cfg.vegetation_dilation_px,
                support_radius_px=cfg.vegetation_support_radius_px,
                support_fraction=cfg.vegetation_support_fraction)
            vegetation = cv2.resize(vegetation.astype(np.float32), size, interpolation=cv2.INTER_AREA) > 0
        blocked = vegetation.copy()
        if blocked_mask is not None:
            supplied = np.asarray(blocked_mask)
            if supplied.shape != original_shape or not np.isfinite(supplied).all():
                raise ValueError("blocked_mask must be finite and match original H,W")
            # Preserve even a subpixel blocked region when reducing a 2K image.
            blocked |= cv2.resize((supplied != 0).astype(np.float32), size, interpolation=cv2.INTER_AREA) > 0
        y_top = int(h * self.config.horizon_fraction)
        current[:y_top] = 0
        current[blocked] = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        probability, preferred = self._filter_history(current, gray)
        probability[:y_top] = 0
        probability[blocked] = 0
        candidate = (probability >= self.config.probability_threshold) & (current >= self.config.current_probability_floor)
        # Hysteresis applies only to a motion-aligned, previously accepted road
        # pixel. Strong current negatives, vegetation and explicit vetoes win.
        if self._aligned_previous_mask is not None:
            candidate |= (self._aligned_previous_mask &
                (probability >= self.config.probability_release_threshold) &
                (current >= self.config.current_probability_floor))
        candidate,recovered=connect_enclosed_uncertainty(candidate,current,probability,blocked,self.config)
        raw_mask = candidate.astype(np.uint8) * 255
        self._previous_mask = raw_mask.copy()
        if not self.config.generate_reference_path:
            # Camera assist consumes the full segmentation mask. Pixel reference
            # generation is not used in this branch.
            empty = np.empty((0, 2), dtype=np.int32)
            unknown_clearance = np.zeros((h, w), dtype=np.uint8)
            reference = build_reference_geometry(unknown_clearance, empty,
                robot_center_x_fraction=self.config.robot_center_x_fraction,
                lookahead_y_fraction=self.config.lookahead_y_fraction,
                horizon_fraction=self.config.horizon_fraction)
            self._previous_gray = gray
            self._previous_probability = probability.copy()
            self._previous_path = None
            return DetectionResult(frame, probability, raw_mask, raw_mask.copy(), unknown_clearance,
                empty, 0., "segmentation_only", backend, (time.perf_counter()-start)*1000.,
                original_shape, reference, vegetation.astype(np.uint8)*255,
                self.config.reject_green_vegetation, False,
                self.config.vegetation_support_radius_px, self.config.vegetation_support_fraction, recovered)
