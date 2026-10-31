"""Planar forward-only Pure Pursuit, independent of ROS and robot drivers.

The caller must provide a fresh path/pose in the SAME fixed metric frame and
perform collision/footprint checks. This module does not infer free space.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ControllerConfig:
    lookahead_m: float = 0.7
    cruise_speed_mps: float = 0.2
    max_angular_speed_rps: float = 0.5
    max_acceleration_mps2: float = 0.25
    max_deceleration_mps2: float = 0.4
    max_angular_acceleration_rps2: float = 0.8
    goal_tolerance_m: float = 0.15
    max_path_offset_m: float = 0.8
    max_segment_m: float = 0.5

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")
        if self.lookahead_m <= self.goal_tolerance_m:
            raise ValueError("lookahead_m must exceed goal_tolerance_m")


@dataclass(frozen=True)
class Demand:
    linear: float = 0.0
    angular: float = 0.0
    reason: str = "no_path"
    cross_track_m: float | None = None
    remaining_m: float | None = None
    target_robot_xy: tuple[float, float] | None = None


def validate_path(points, max_segment_m):
    if not math.isfinite(max_segment_m) or max_segment_m <= 0:
        raise ValueError("max_segment_m must be finite and positive")
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not 2 <= len(points) <= 4096 or not np.isfinite(points).all():
        raise ValueError("path needs 2..4096 finite metric [x, y] points")
    with np.errstate(over="ignore", invalid="ignore"):
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if not np.isfinite(lengths).all():
        raise ValueError("path segment lengths must be finite")
    points = points[np.r_[True, lengths > 1e-6]]
    if len(points) < 2:
        raise ValueError("path has no length")
    if (np.linalg.norm(np.diff(points, axis=0), axis=1) > max_segment_m).any():
        raise ValueError("path contains a segment gap exceeding max_segment_m")
    return points.copy()


def pursuit(points, pose_xyyaw, config=ControllerConfig()):
    """Track a continuous, ordered local path. No reversing or turn-in-place.

    The nearest projection on a segment is the arc-length origin. Self-crossing
    routes need a route-progress planner upstream; they are outside this local
    controller's contract. Speed is reduced so v*curvature respects omega_max.
    """
    try:
        path = validate_path(points, config.max_segment_m)
    except (ValueError, TypeError):
        return Demand(reason="invalid_path")
    pose = np.asarray(pose_xyyaw, dtype=float)
    if pose.shape != (3,) or not np.isfinite(pose).all():
        return Demand(reason="invalid_pose")
    segments = np.diff(path, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    with np.errstate(over="ignore", invalid="ignore"):
        fractions = np.clip(np.sum((pose[:2] - path[:-1]) * segments, axis=1) / lengths**2, 0, 1)
        nearest = path[:-1] + fractions[:, None] * segments
        distances = np.linalg.norm(nearest - pose[:2], axis=1)
    if not np.isfinite(distances).all():
        return Demand(reason="invalid_pose")
    index = int(np.argmin(distances))
    cross_track = float(distances[index])
    remaining = float((1 - fractions[index]) * lengths[index] + lengths[index + 1:].sum())
    if cross_track > config.max_path_offset_m:
        return Demand(reason="too_far_from_path", cross_track_m=cross_track, remaining_m=remaining)
    if remaining <= config.goal_tolerance_m and np.linalg.norm(path[-1] - pose[:2]) <= config.goal_tolerance_m:
        return Demand(reason="goal_reached", cross_track_m=cross_track, remaining_m=remaining)
    target = nearest[index]
    distance = config.lookahead_m
    for end, length in [(path[index + 1], float(np.linalg.norm(path[index + 1] - target))),
                        *zip(path[index + 2:], lengths[index + 1:])]:
        if length > distance:
            target = target + (end - target) * distance / length
            break
        target = end
        distance -= length
    dx, dy = target - pose[:2]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    local = (float(c*dx + s*dy), float(-s*dx + c*dy))
    if local[0] <= 0.02:
        return Demand(reason="target_not_ahead", cross_track_m=cross_track, remaining_m=remaining, target_robot_xy=local)
    curvature = 2 * local[1] / (local[0]**2 + local[1]**2)
    stopping_distance = max(remaining, float(np.linalg.norm(path[-1] - pose[:2])))
    speed = min(config.cruise_speed_mps, math.sqrt(2 * config.max_deceleration_mps2 * max(stopping_distance - config.goal_tolerance_m, 0)))
    if abs(curvature) > 1e-9:
        speed = min(speed, config.max_angular_speed_rps / abs(curvature))
    return Demand(float(speed), float(speed * curvature), "tracking", cross_track, remaining, local)


class CommandLimiter:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.linear = self.angular = 0.0

    def step(self, demand, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("control dt must be finite and positive")
        # Stopping on a fault or goal is immediate. Deceleration limiting only
        # applies during ordinary tracking; it must not delay a stop request.
        if demand.reason != "tracking" or not math.isfinite(demand.linear) or not math.isfinite(demand.angular):
            self.reset()
            return 0.0, 0.0
        dt = min(dt, 0.1)
        rate = self.config.max_acceleration_mps2 if demand.linear > self.linear else self.config.max_deceleration_mps2
        self.linear += float(np.clip(demand.linear - self.linear, -rate*dt, rate*dt))
        angular_step = self.config.max_angular_acceleration_rps2 * dt
        self.angular += float(np.clip(demand.angular - self.angular, -angular_step, angular_step))
        self.linear = float(np.clip(self.linear, 0, self.config.cruise_speed_mps))
        self.angular = float(np.clip(self.angular, -self.config.max_angular_speed_rps, self.config.max_angular_speed_rps))
        return self.linear, self.angular


def quaternion_matrix(x, y, z, w):
    values = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or abs(norm - 1) > 1e-3:
        raise ValueError("orientation must be a finite unit quaternion")
    x, y, z, w = values / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def fresh_stamp(stamp_ns, received_at, now_ros_ns, now_monotonic, timeout):
    """A future/zero source stamp is invalid; receipt time alone cannot hide lag."""
    age = (now_ros_ns - stamp_ns) / 1e9
    return stamp_ns > 0 and -0.05 <= age <= timeout and 0 <= now_monotonic - received_at <= timeout
