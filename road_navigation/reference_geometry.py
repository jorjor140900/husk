"""Image-space guides for display and future robot integration.

Straight boundary fits describe the outer mask envelope only. They never fill
the mask or replace the collision-checked yellow path. The robot axis assumes
an aligned camera until calibration supplies its horizontal image position.
"""

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class BoundaryLine:
    slope: float
    intercept: float
    y_top: int
    y_bottom: int
    support_points: int

    def x_at(self, y: float) -> float:
        return self.slope * y + self.intercept


@dataclass(frozen=True)
class ReferenceGeometry:
    robot_x: int
    y_top: int
    y_bottom: int
    lookahead_y: int
    target_x: float | None
    lateral_error_px: float | None
    lateral_error_norm: float | None
    left: BoundaryLine | None
    right: BoundaryLine | None

    def metadata(self) -> dict:
        result = asdict(self)
        result["coordinate_system"] = "processed_image_pixels_xy"
        result["lateral_error_sign"] = "positive = reference to the right of robot axis"
        result["robot_axis_source"] = "configured camera alignment assumption"
        result["boundary_use"] = "display approximation, not planning constraints"
        return result


def _fit_boundary(points, image_width: int, min_span: float) -> BoundaryLine | None:
    if len(points) < 6:
        return None
    ys, xs = np.asarray(points, dtype=np.float64).T
    if np.ptp(ys) < min_span:
        return None
    # Median pairwise slopes give a deterministic robust starting line. Initial
    # least squares could otherwise let a few mask leaks dominate the fit.
    first, second = np.triu_indices(len(ys), 1)
    dy = ys[second] - ys[first]
    slopes = (xs[second] - xs[first]) / dy
    slope = float(np.median(slopes))
    intercept = float(np.median(xs - slope * ys))
    residuals = np.abs(xs - (slope * ys + intercept))
    mad = float(np.median(np.abs(residuals - np.median(residuals))))
    tolerance = min(image_width * 0.05, max(3.0, image_width * 0.015, 2.5 * mad))
    keep = residuals <= tolerance
    if keep.sum() < 6 or np.ptp(ys[keep]) < min_span:
        return None
    slope, intercept = np.polyfit(ys[keep], xs[keep], 1)
    if not np.isfinite([slope, intercept]).all() or abs(slope) > 4:
        return None
    return BoundaryLine(
        float(slope), float(intercept), int(ys[keep].min()),
        int(ys[keep].max()), int(keep.sum()),
    )


def estimate_boundaries(road_mask: np.ndarray) -> tuple[BoundaryLine | None, BoundaryLine | None]:
    """Fit observed left/right outer edges, excluding image-border cutoffs.

    Holes inside the area remain holes and do not become its outer boundary.
    A side outside the camera view stays unknown; no image-border fallback or
    stale previous-frame line is invented. Fits stop at their observed support.
    """
    if road_mask.ndim != 2:
        raise ValueError("road_mask must be a 2-D array")
    h, w = road_mask.shape
    rows = np.flatnonzero(np.any(road_mask > 0, axis=1))
    if not len(rows) or rows[-1] - rows[0] < max(20, h * 0.15):
        return None, None
    top = int(round(rows[0] + 0.15 * (rows[-1] - rows[0])))
    bottom = int(rows[-1])
    samples = np.unique(np.linspace(top, bottom, 48).astype(int))
    border = max(2, round(w * 0.005))
    left, right = [], []
    for y in samples:
        xs = np.flatnonzero(road_mask[y] > 0)
        if len(xs) < 3:
            continue
        if xs[0] > border:
            left.append((int(y), int(xs[0])))
        if xs[-1] < w - 1 - border:
            right.append((int(y), int(xs[-1])))
    min_span = max(12.0, (bottom - top) * 0.25)
    left_line = _fit_boundary(left, w, min_span)
    right_line = _fit_boundary(right, w, min_span)
    if left_line is not None and right_line is not None:
        common_top = max(left_line.y_top, right_line.y_top)
        common_bottom = min(left_line.y_bottom, right_line.y_bottom)
        if common_top <= common_bottom and any(
            left_line.x_at(y) >= right_line.x_at(y)
            for y in (common_top, common_bottom)
        ):
            return None, None
    return left_line, right_line


def build_reference_geometry(
    road_mask: np.ndarray,
    path: np.ndarray,
    *,
    robot_center_x_fraction: float = 0.5,
    lookahead_y_fraction: float = 0.65,
    horizon_fraction: float = 0.30,
) -> ReferenceGeometry:
    h, w = road_mask.shape
    robot_x = int(round((w - 1) * robot_center_x_fraction))
    lookahead_y = int(round((h - 1) * lookahead_y_fraction))
    left, right = estimate_boundaries(road_mask)
    target_x = error = normalized_error = None
    # No extrapolation to an unobserved part of the path or stale target.
    if len(path) >= 2 and path[:, 1].min() <= lookahead_y <= path[:, 1].max():
        order = np.argsort(path[:, 1])
        target_x = float(np.interp(lookahead_y, path[order, 1], path[order, 0]))
        error = target_x - robot_x
        normalized_error = error / max(1.0, w / 2.0)
    return ReferenceGeometry(
        robot_x, int(h * horizon_fraction), h - 1, lookahead_y,
        target_x, error, normalized_error, left, right,
    )
