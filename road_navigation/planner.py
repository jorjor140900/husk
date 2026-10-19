"""Plan a connected image-space reference without filling obstacle holes.

Pixel clearance is a development aid, not a calibrated robot footprint. The
returned path only claims consistency with the supplied mask, not real-world
traversability. No curve fitting or smoothing may move it outside that mask.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class PathPlan:
    points: np.ndarray
    navigable_mask: np.ndarray
    reason: str

    @property
    def valid(self) -> bool:
        return len(self.points) > 0


def _shift(values: np.ndarray, dx: int, fill: float | bool) -> np.ndarray:
    shifted = np.full_like(values, fill)
    if dx == 0:
        shifted[:] = values
    elif 0 < dx < len(values):
        shifted[dx:] = values[:-dx]
    elif -len(values) < dx < 0:
        shifted[:dx] = values[-dx:]
    return shifted


def plan_path(
    road_mask: np.ndarray,
    probability: np.ndarray,
    *,
    seed_x: float = 0.5,
    seed_y: float = 0.94,
    horizon: float = 0.30,
    goal_x: float = 0.5,
    clearance_px: int = 8,
    max_lateral_step: int = 2,
    min_path_height: float = 0.25,
    preferred_path: np.ndarray | None = None,
) -> PathPlan:
    """Find the lowest-cost upward path from the assumed robot contact point.

    Dynamic programming visits every image row. A transition checks the whole
    rectangle between its endpoints on both rows, so it cannot jump across a
    hole or cut a blocked corner. A blocked start is rejected, never relocated
    to a disconnected road fragment. Paths are ordered from near to far.
    """
    if road_mask.ndim != 2 or road_mask.shape != probability.shape:
        raise ValueError("road_mask and probability must be matching 2-D arrays")
    if clearance_px < 0 or not 1 <= max_lateral_step <= 10:
        raise ValueError("clearance_px must be nonnegative; lateral step must be 1..10")
    if not (0 <= horizon < seed_y <= 1 and 0 <= seed_x <= 1 and 0 <= goal_x <= 1):
        raise ValueError("invalid normalized start, horizon or goal")
    if not 0 < min_path_height <= 1:
        raise ValueError("min_path_height must be in (0, 1]")

    h, w = road_mask.shape
    # Padding makes the image border unknown/blocked rather than infinite road.
    padded = np.pad((road_mask > 0).astype(np.uint8), 1)
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    navigable = distance > clearance_px
    nav_u8 = navigable.astype(np.uint8) * 255
    empty = np.empty((0, 2), dtype=np.int32)
    start_x, start_y = int(round((w - 1) * seed_x)), int(round((h - 1) * seed_y))
    # Keep the same clearance from the artificial ROI cutoff as other edges.
    # Otherwise every open corridor would be reported as stopping prematurely.
    top = min(start_y, int(h * horizon) + clearance_px)
    if not navigable[start_y, start_x]:
        return PathPlan(empty, nav_u8, "start_blocked_or_too_narrow")

    xs = np.arange(w)
    goal_distance = np.abs(xs / max(w - 1, 1) - goal_x)
    local_cost = (
        0.45 * (1.0 - probability)
        + 0.25 / (distance + 1.0)
        + 0.025 * goal_distance[None, :]
    )
    if preferred_path is not None and preferred_path.shape == road_mask.shape and preferred_path.any():
        prior_distance = cv2.distanceTransform(
            (preferred_path == 0).astype(np.uint8), cv2.DIST_L2, 3
        )
        local_cost += 0.10 * np.minimum(prior_distance / max(w, 1), 0.25)

    costs = np.full(w, np.inf)
    costs[start_x] = 0.0
    parents = np.full((h, w), -1, dtype=np.int32)
    last_y = start_y
    # Prefer straight movement on equal costs, making symmetric cases repeatable.
    moves = [0] + [sign * step for step in range(1, max_lateral_step + 1) for sign in (-1, 1)]
    for y in range(start_y - 1, top - 1, -1):
        best = np.full(w, np.inf)
        both_rows = navigable[y] & navigable[y + 1]
        for dx in moves:
            allowed = both_rows.copy()
            direction = 1 if dx > 0 else -1
            for offset in range(1, abs(dx) + 1):
                allowed &= _shift(both_rows, direction * offset, False)
            candidate = _shift(costs, dx, np.inf) + 0.02 * abs(dx)
            better = allowed & (candidate < best)
            best[better] = candidate[better]
            parents[y, better] = xs[better] - dx
        if not np.isfinite(best).any():
            break
        costs = best + local_cost[y]
        last_y = y

    if start_y - last_y < h * min_path_height:
        return PathPlan(empty, nav_u8, "insufficient_lookahead")

    x = int(np.argmin(costs + 0.25 * goal_distance))
    points = [(x, last_y)]
    for y in range(last_y, start_y):
        x = int(parents[y, x])
        if x < 0:
            raise RuntimeError("path reconstruction lost a reachable predecessor")
        points.append((x, y + 1))
    path = np.array(points[::-1], dtype=np.int32)
    reason = "ok" if last_y == top else "limited_lookahead"
    return PathPlan(path, nav_u8, reason)

