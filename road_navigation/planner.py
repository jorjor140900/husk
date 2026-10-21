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


def _smooth_reference(path, navigable, local_cost, goal_x, max_step):
    """Search a small family of smooth references and check every transition.

    These are candidate curves, not a spline blindly applied after planning.
    Every pixel between samples must remain in the clearance mask. If the
    observed corridor supports only a kinked route around uncertain texture,
    report no smooth reference instead of presenting those kinks as guidance.
    """
    start_x, start_y = path[0]
    end_y = int(path[-1, 1])
    _, w = navigable.shape
    free = np.flatnonzero(navigable[end_y])
    if not len(free):
        return None
    endpoints = np.unique(np.rint(np.concatenate([
        np.linspace(free[0], free[-1], 33),
        [path[-1, 0], free[len(free) // 2], goal_x * (w - 1), start_x],
    ])).astype(int))
    endpoints = endpoints[(endpoints >= 0) & (endpoints < w)]
    endpoints = endpoints[navigable[end_y, endpoints]]
    offsets = np.linspace(-w * .25, w * .25, 17)
    targets = np.repeat(endpoints, len(offsets))
    bumps = np.tile(offsets, len(endpoints))
    ys = np.arange(start_y, end_y - 1, -1)
    t = np.linspace(0, 1, len(ys))
    blend = 3 * t ** 2 - 2 * t ** 3
    bump = 16 * t ** 2 * (1 - t) ** 2
    curves = start_x + (targets[:, None] - start_x) * blend + bumps[:, None] * bump
    xs = np.rint(curves).astype(int)
    inside = ((xs >= 0) & (xs < w)).all(axis=1)
    safe_x = np.clip(xs, 0, w - 1)
    allowed = inside & navigable[ys[None, :], safe_x].all(axis=1)
    allowed &= (np.abs(np.diff(xs, axis=1)) <= max_step).all(axis=1)
    # Supercover check, matching DP: no skipped hole or diagonal corner cut.
    left = np.minimum(safe_x[:, :-1], safe_x[:, 1:])
    right = np.maximum(safe_x[:, :-1], safe_x[:, 1:])
    for step in range(max_step + 1):
        x = np.minimum(left + step, right)
        allowed &= navigable[ys[None, :-1], x].all(axis=1)
        allowed &= navigable[ys[None, 1:], x].all(axis=1)
    if not allowed.any():
        return None
    scores = local_cost[ys[None, :], safe_x].mean(axis=1)
    scores += 0.05 * np.abs(bumps) / max(w, 1)
    scores += 0.10 * np.abs(targets / max(w - 1, 1) - goal_x)
    scores[~allowed] = np.inf
    best = int(np.argmin(scores))
    return np.column_stack([xs[best], ys]).astype(np.int32)


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
    """Find a centered, smooth reference from the assumed robot contact point.

    Dynamic programming visits every image row. A transition checks the whole
    rectangle between its endpoints on both rows, so it cannot jump across a
    hole or cut a blocked corner. A blocked start is rejected, never relocated
    to a disconnected road fragment. Then a bounded family of smooth curves is
    checked against the same clearance mask; if none fits, return no reference
    even if a discrete route exists. Paths are ordered from near to far. This
    image-space shape constraint does not model vehicle turning dynamics.
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
    # A reference should stay near the center of the observed corridor. The
    # previous color-dominated cost chased bright patches instead, including
    # misclassified columns. Appearance is only a weak tie-breaker after the
    # hard mask/clearance constraints, never the primary steering objective.
    row_clearance = np.maximum(distance.max(axis=1, keepdims=True), 1.0)
    local_cost = (
        1.0 - distance / row_clearance
        + 0.05 * (1.0 - probability)
        + 0.05 * goal_distance[None, :]
    )
    if preferred_path is not None and preferred_path.shape == road_mask.shape and preferred_path.any():
        prior_distance = cv2.distanceTransform(
            (preferred_path == 0).astype(np.uint8), cv2.DIST_L2, 3
        )
        local_cost += 0.08 * np.minimum(prior_distance / max(w * 0.1, 1), 1.0)

    # Prefer straight movement on equal costs, making symmetric cases repeatable.
    moves = [0] + [sign * step for step in range(1, max_lateral_step + 1) for sign in (-1, 1)]
    headings = np.asarray(moves)
    costs = np.full((w, len(moves)), np.inf)
    costs[start_x, 0] = 0.0
    # Track the previous direction as well as position: positional cost alone
    # permitted abrupt elbows and frame-texture zigzags. No unchecked spline or
    # post-filter may move the resulting path through a blocked pixel.
    parents = np.full((h, w, len(moves)), -1, dtype=np.int8)
    last_y = start_y
    for y in range(start_y - 1, top - 1, -1):
        best = np.full_like(costs, np.inf)
        both_rows = navigable[y] & navigable[y + 1]
        for heading, dx in enumerate(moves):
            allowed = both_rows.copy()
            direction = 1 if dx > 0 else -1
            for offset in range(1, abs(dx) + 1):
                allowed &= _shift(both_rows, direction * offset, False)
            candidate = _shift(costs, dx, np.inf) + 0.35 * (dx - headings[None, :]) ** 2
            previous_heading = candidate.argmin(axis=1)
            values = candidate[xs, previous_heading] + 0.005 * dx ** 2
            best[allowed, heading] = values[allowed] + local_cost[y, allowed]
            parents[y, allowed, heading] = previous_heading[allowed]
        if not np.isfinite(best).any():
            break
        costs = best
        last_y = y

    if start_y - last_y < h * min_path_height:
        return PathPlan(empty, nav_u8, "insufficient_lookahead")

    x, heading = np.unravel_index(np.argmin(costs + 0.25 * goal_distance[:, None]), costs.shape)
    x, heading = int(x), int(heading)
    points = [(x, last_y)]
    for y in range(last_y, start_y):
        previous_heading = int(parents[y, x, heading])
        x -= moves[heading]
        heading = previous_heading
        if heading < 0:
            raise RuntimeError("path reconstruction lost a reachable predecessor")
        points.append((x, y + 1))
    path = np.array(points[::-1], dtype=np.int32)
    path = _smooth_reference(path, navigable, local_cost, goal_x, max_lateral_step)
    if path is None:
        return PathPlan(empty, nav_u8, "no_smooth_reference")
    reason = "ok" if last_y == top else "limited_lookahead"
    return PathPlan(path, nav_u8, reason)

