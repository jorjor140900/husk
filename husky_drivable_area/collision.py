"""Experimental 2-D scan/swept-rectangle geometry, independent of ROS.

This checks sampled returns and assumed constant-curvature stop motions.
It cannot certify terrain, sensor coverage between rays, or physical braking.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CollisionConfig:
    front_m: float = 0.45
    rear_m: float = 0.45
    half_width_m: float = 0.30
    margin_m: float = 0.15
    reaction_sec: float = 0.30
    braking_deceleration_mps2: float = 0.4
    braking_angular_deceleration_rps2: float = 0.8
    max_linear_mps: float = 0.25
    max_angular_rps: float = 0.6
    sweep_step_m: float = 0.01

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.sweep_step_m > 0.05:
            raise ValueError("sweep_step_m must be <= 0.05")


def scan_points(ranges, angle_min, angle_max, angle_increment, range_min, range_max, time_increment=0.0):
    """Require a complete, instantaneous/deskewed planar scan.

    Positive infinity means no return out to range_max. NaN, -inf and invalid
    distances are unknown and reject the scan; they are never made free space.
    """
    data = np.asarray(ranges, dtype=float)
    fields = (angle_min, angle_max, angle_increment, range_min, range_max, time_increment)
    if not all(math.isfinite(v) for v in fields) or data.ndim != 1 or not 180 <= len(data) <= 10000:
        raise ValueError("invalid_scan_metadata")
    if time_increment != 0:
        raise ValueError("scan_requires_deskew")
    if not 0 < angle_increment <= 0.02 or not 0 < range_min < range_max:
        raise ValueError("invalid_scan_metadata")
    if abs(angle_increment * len(data) - 2*math.pi) > angle_increment * 1.1:
        raise ValueError("incomplete_scan")
    if abs(angle_max - (angle_min + (len(data)-1)*angle_increment)) > angle_increment * 0.02:
        raise ValueError("invalid_scan_metadata")
    finite = np.isfinite(data)
    if np.isnan(data).any() or np.isneginf(data).any() or (data[finite] < range_min).any() or (data[finite] > range_max).any():
        raise ValueError("invalid_scan_ranges")
    angles = angle_min + np.arange(len(data)) * angle_increment
    return np.column_stack([data[finite] * np.cos(angles[finite]), data[finite] * np.sin(angles[finite])])


def stop_sweep(velocity, config, extra_reaction_sec=0.0):
    v, w = velocity
    if not all(math.isfinite(t) for t in (v, w, extra_reaction_sec)) or extra_reaction_sec < 0:
        raise ValueError("invalid_motion")
    if v < -1e-6 or v > config.max_linear_mps + 1e-6 or abs(w) > config.max_angular_rps + 1e-6:
        raise ValueError("motion_out_of_bounds")
    v = max(0, v)
    # Scale v and w down together to retain curvature during an assumed brake.
    # Integral of that scale = reaction + half of the braking duration.
    brake_time = max(v/config.braking_deceleration_mps2, abs(w)/config.braking_angular_deceleration_rps2)
    effective_time = config.reaction_sec + extra_reaction_sec + brake_time/2
    radius = math.hypot(max(config.front_m, config.rear_m) + config.margin_m,
                        config.half_width_m + config.margin_m)
    count = max(1, int(math.ceil(effective_time * (v + radius*abs(w)) / config.sweep_step_m))) + 1
    if count > 2048:
        raise ValueError("sweep_exceeds_limit")
    t = np.linspace(0, effective_time, count)
    if abs(w) < 1e-9:
        return np.column_stack([v*t, np.zeros(count), np.zeros(count)])
    return np.column_stack([v/w*np.sin(w*t), v/w*(1-np.cos(w*t)), w*t])


def check_sweep(points_base, requested, measured, config, *, extra_reaction_sec=0.0,
                sensor_origin_base=(0, 0), sensor_range_m=8.0, additional_velocities=()):
    """Return (reason, sampled poses). 'clear' only describes this 2-D model."""
    points = np.asarray(points_base, dtype=float).reshape(-1, 2)
    if not np.isfinite(points).all() or not math.isfinite(sensor_range_m) or sensor_range_m <= 0:
        raise ValueError("invalid_scan_geometry")
    origin = np.asarray(sensor_origin_base, dtype=float)
    if origin.shape != (2,) or not np.isfinite(origin).all():
        raise ValueError("invalid_scan_origin")
    poses = np.vstack([stop_sweep(velocity, config, extra_reaction_sec)
                       for velocity in (requested, measured, *additional_velocities)])
    # Inflate by sample spacing so a return cannot slip between sample poses.
    m = config.margin_m + config.sweep_step_m
    corners = np.array([[config.front_m+m, config.half_width_m+m],
                        [config.front_m+m, -config.half_width_m-m],
                        [-config.rear_m-m, -config.half_width_m-m],
                        [-config.rear_m-m, config.half_width_m+m]])
    for x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        if (np.linalg.norm(corners @ rotation.T + [x, y] - origin, axis=1) > sensor_range_m).any():
            return "sweep_outside_scan_range", poses
        local = (points - [x, y]) @ rotation
        inside = ((local[:, 0] <= config.front_m+m) & (local[:, 0] >= -config.rear_m-m) &
                  (np.abs(local[:, 1]) <= config.half_width_m+m))
        if inside.any():
            return "obstacle_in_sweep", poses
    return "clear", poses


def circle_scan(pose_xyyaw, circles, *, count=720, range_max=8.0):
    """Synthetic ray/circle intersections for the demo, not obstacle gating."""
    x, y, yaw = pose_xyyaw
    angles = -math.pi + np.arange(count) * 2*math.pi/count
    directions = np.column_stack([np.cos(angles+yaw), np.sin(angles+yaw)])
    distances = np.full(count, np.inf)
    for cx, cy, radius in circles:
        center = np.array([cx-x, cy-y])
        along = directions @ center
        discriminant = radius**2 - (center @ center - along**2)
        root = np.sqrt(np.maximum(0, discriminant))
        near, far = along-root, along+root
        hit = np.where(near >= 0, near, far)
        valid = (discriminant >= 0) & (hit >= 0) & (hit <= range_max)
        distances[valid] = np.minimum(distances[valid], hit[valid])
    return distances
