"""Read timed VLP16 clouds and deskew measured returns, without ROS imports.

This is a point-data adapter, not a free-space or motion-permission adapter.
It does not turn absent returns into free rays, select obstacle heights, remove
the robot, or publish a LaserScan. Those require measured field geometry.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TimedCloud:
    stamp_ns: int
    frame_id: str
    xyz: np.ndarray
    rings: np.ndarray
    offsets_sec: np.ndarray


def stamp_ns(stamp):
    sec = getattr(stamp, "sec", getattr(stamp, "secs", None))
    nsec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", None))
    if type(sec) is not int or type(nsec) is not int or not 0 <= nsec < 1_000_000_000:
        raise ValueError("invalid_cloud_stamp")
    value = sec*1_000_000_000+nsec
    if not 0 < value < 2**63-1_000_000_000:
        raise ValueError("invalid_cloud_stamp")
    return value


def read_timed_cloud(message):
    """Accept the actual x/y/z FLOAT32, ring UINT16, time FLOAT32 layout.

    Point time is a *signed* offset from the original cloud header. In the
    recorded Noetic driver it is mostly negative because the header uses the
    last packet's stamp. Preserve row padding and byte order.
    """
    width, height, step, row = message.width, message.height, message.point_step, message.row_step
    if (any(type(x) is not int for x in (width, height, step, row)) or
            width <= 0 or height <= 0 or width*height > 1_000_000 or
            not 18 <= step <= 4096 or row < width*step or row*height > 64*1024*1024):
        raise ValueError("invalid_cloud_dimensions")
    fields = {}
    for field in message.fields:
        if field.name in fields:
            raise ValueError("duplicate_cloud_field")
        fields[field.name] = field
    expected = {"x": (7, "f4", 4), "y": (7, "f4", 4), "z": (7, "f4", 4),
                "ring": (4, "u2", 2), "time": (7, "f4", 4)}
    used = set()
    for name, (datatype, _, size) in expected.items():
        field = fields.get(name)
        if (field is None or field.datatype != datatype or field.count != 1 or
                type(field.offset) is not int or field.offset < 0 or field.offset+size > step):
            raise ValueError("invalid_or_missing_cloud_field:"+name)
        occupied = set(range(field.offset, field.offset+size))
        if used & occupied:
            raise ValueError("overlapping_cloud_fields")
        used |= occupied
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": list(expected),
        "formats": [endian+spec[1] for spec in expected.values()],
        "offsets": [fields[name].offset for name in expected], "itemsize": step})
    raw = memoryview(message.data)
    if raw.nbytes != row*height:
        raise ValueError("invalid_cloud_data_length")
    data = np.ndarray((height, width), dtype=dtype, buffer=raw, strides=(row, step)).reshape(-1)
    xyz = np.column_stack([data[name] for name in ("x", "y", "z")]).astype(np.float64)
    offsets = data["time"].astype(np.float64)
    rings = data["ring"].astype(np.uint16)
    if not np.isfinite(xyz).all() or not np.isfinite(offsets).all():
        raise ValueError("nonfinite_cloud_returns_or_time")
    if np.any(rings > 15) or np.any(np.linalg.norm(xyz, axis=1) <= 0):
        raise ValueError("invalid_vlp16_returns")
    # Bounds for this last-packet-stamped VLP16 adapter, not all cloud drivers.
    if offsets.min() < -0.2 or offsets.max() > 0.005 or np.ptp(offsets) > 0.2:
        raise ValueError("unsupported_vlp16_time_offsets")
    frame = message.header.frame_id.lstrip("/")
    if not frame or any(c.isspace() for c in frame):
        raise ValueError("invalid_cloud_frame")
    return TimedCloud(stamp_ns(message.header.stamp), frame, xyz, rings, offsets)


def deskew_returns(cloud, odom_stamps_ns, odom_xy_yaw, lidar_to_base, *, max_odom_gap_sec=0.1):
    """Express measured returns in base coordinates at the cloud header stamp.

    Interpolate measured planar odometry at each point's actual signed time.
    Require brackets over the *entire* acquisition interval including the
    target stamp; never extrapolate or assume zero motion for missing odometry.
    Returns only observed xyz points. Unknown/unobserved space stays unknown.
    """
    if (type(cloud.stamp_ns) is not int or not 0 < cloud.stamp_ns < 2**63-1_000_000_000 or
            cloud.xyz.ndim != 2 or cloud.xyz.shape[1] != 3 or len(cloud.xyz) == 0 or
            cloud.offsets_sec.shape != (len(cloud.xyz),) or not np.isfinite(cloud.xyz).all() or
            not np.isfinite(cloud.offsets_sec).all() or cloud.offsets_sec.min() < -0.2 or
            cloud.offsets_sec.max() > 0.005):
        raise ValueError("invalid_timed_cloud")
    stamps = np.asarray(odom_stamps_ns)
    poses = np.asarray(odom_xy_yaw, dtype=float)
    transform = np.asarray(lidar_to_base, dtype=float)
    if (stamps.ndim != 1 or stamps.dtype.kind not in "iu" or len(stamps) < 2 or
            poses.shape != (len(stamps), 3) or not np.isfinite(poses).all() or
            np.any(stamps <= 0) or np.any(stamps >= 2**63-1_000_000_000) or
            np.any(np.diff(stamps.astype(np.int64)) <= 0)):
        raise ValueError("invalid_odometry_trajectory")
    if (transform.shape != (4, 4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-9) or
            not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-6)):
        raise ValueError("invalid_measured_lidar_transform")
    if not np.isfinite(max_odom_gap_sec) or max_odom_gap_sec <= 0:
        raise ValueError("invalid_odometry_gap_limit")
    point_stamps = cloud.stamp_ns+np.rint(cloud.offsets_sec*1e9).astype(np.int64)
    lower, upper = min(int(point_stamps.min()), cloud.stamp_ns), max(int(point_stamps.max()), cloud.stamp_ns)
    if lower < stamps[0] or upper > stamps[-1]:
        raise ValueError("odometry_does_not_bracket_cloud")
    first = max(0, int(np.searchsorted(stamps, lower, side="right"))-1)
    last = min(len(stamps)-1, int(np.searchsorted(stamps, upper, side="left")))
    if np.any(np.diff(stamps[first:last+1].astype(np.int64))/1e9 > max_odom_gap_sec):
        raise ValueError("odometry_gap_during_cloud")
    # Subtract the integer epoch before floating interpolation to preserve time.
    origin = int(stamps[0])
    timeline = (stamps.astype(np.int64)-origin)/1e9
    query = (point_stamps-origin)/1e9
    target = (cloud.stamp_ns-origin)/1e9
    yaw = np.unwrap(poses[:, 2])
    x, y, a = [np.interp(query, timeline, values) for values in (poses[:, 0], poses[:, 1], yaw)]
    tx, ty, ta = [float(np.interp(target, timeline, values)) for values in (poses[:, 0], poses[:, 1], yaw)]
    base_points = cloud.xyz @ transform[:3, :3].T+transform[:3, 3]
    fixed_x = np.cos(a)*base_points[:, 0]-np.sin(a)*base_points[:, 1]+x
    fixed_y = np.sin(a)*base_points[:, 0]+np.cos(a)*base_points[:, 1]+y
    dx, dy = fixed_x-tx, fixed_y-ty
    return np.column_stack([np.cos(ta)*dx+np.sin(ta)*dy,
                            -np.sin(ta)*dx+np.cos(ta)*dy, base_points[:, 2]])
