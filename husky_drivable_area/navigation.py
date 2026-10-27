"""Goal-navigation configuration and mapping-only scan normalization."""
import copy
import math
from pathlib import Path

import numpy as np


def saved_map_metadata(filename):
    """Validate a Nav2 map pair before starting any navigation nodes."""
    import cv2
    import yaml

    path = Path(filename).expanduser().resolve()
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not isinstance(data.get("image"), str) or not data["image"]:
        raise ValueError("map YAML must name its image")
    resolution, origin = data.get("resolution"), data.get("origin")
    if (type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0 or
            not isinstance(origin, list) or len(origin) != 3 or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in origin)):
        raise ValueError("map resolution/origin must be finite and resolution positive")
    free, occupied = data.get("free_thresh"), data.get("occupied_thresh")
    if (type(free) not in (int, float) or type(occupied) not in (int, float) or
            not 0 <= free < occupied <= 1 or data.get("negate") not in (0, 1) or
            data.get("mode", "trinary") not in ("trinary", "scale", "raw")):
        raise ValueError("invalid map thresholds, negate or mode")
    image = Path(data["image"])
    if not image.is_absolute():
        image = path.parent/image
    if not image.is_file():
        raise ValueError("saved map image is missing: "+str(image))
    pixels = cv2.imread(str(image), cv2.IMREAD_UNCHANGED)
    if pixels is None or pixels.ndim not in (2, 3) or pixels.size == 0:
        raise ValueError("saved map image cannot be decoded")
    return dict(yaml=str(path), image=str(image.resolve()), resolution=float(resolution),
                origin=list(map(float, origin)), width=int(pixels.shape[1]), height=int(pixels.shape[0]))


def mapping_scan_geometry(scan):
    """Correct array endpoints for mapping; never certify deskew/free space."""
    count = len(scan.ranges)
    numbers = (scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min,
               scan.range_max, scan.time_increment, scan.scan_time)
    if (not 180 <= count <= 10000 or not all(math.isfinite(v) for v in numbers) or
            not 0 < scan.angle_increment <= .04 or scan.range_min < 0 or
            scan.range_max <= scan.range_min or scan.time_increment < 0 or scan.scan_time < 0 or
            not math.pi <= (count-1)*scan.angle_increment <= 2*math.pi+.04):
        raise ValueError("unsupported_mapping_scan_geometry")
    ranges = np.asarray(scan.ranges, dtype=float)
    if np.isnan(ranges).any() or np.isneginf(ranges).any() or (ranges < 0).any():
        raise ValueError("invalid_mapping_scan_returns")
    finite = ranges[np.isfinite(ranges)]
    if np.any((finite < scan.range_min) | (finite > scan.range_max)):
        raise ValueError("mapping_return_out_of_range")
    return dict(angle_max=scan.angle_min+(count-1)*scan.angle_increment,
                range_min=max(scan.range_min, float(np.finfo(np.float32).eps)))


def navigation_parameters(profile, template, map_file, tree_file):
    """Render native Nav2 parameters using the measured robot footprint."""
    data = copy.deepcopy(template)
    robot, footprint = profile["robot"], profile["footprint"]
    for key in ("front_m", "rear_m", "half_width_m", "margin_m"):
        value = footprint.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("measure footprint."+key)
    base, odom = robot.get("base_frame"), robot.get("odom_frame")
    if (any(not isinstance(frame, str) or not frame or frame.startswith("/") or
            any(char.isspace() for char in frame) for frame in (base, odom)) or base == odom):
        raise ValueError("distinct real base_frame/odom_frame required, without whitespace or leading slash")
    margin = footprint["margin_m"]
    front, rear, half = footprint["front_m"]+margin, footprint["rear_m"]+margin, footprint["half_width_m"]+margin
    polygon = str([[front, half], [front, -half], [-rear, -half], [-rear, half]])
    data["map_server"]["ros__parameters"]["yaml_filename"] = str(Path(map_file).expanduser().resolve())
    amcl = data["amcl"]["ros__parameters"]
    amcl.update(base_frame_id=base, odom_frame_id=odom)
    navigator = data["bt_navigator"]["ros__parameters"]
    navigator.update(robot_base_frame=base, default_nav_to_pose_bt_xml=str(tree_file))
    for name, frame in (("local_costmap", odom), ("global_costmap", "map")):
        params = data[name][name]["ros__parameters"]
        params.update(robot_base_frame=base, global_frame=frame, footprint=polygon, footprint_padding=0.)
        params["inflation_layer"]["inflation_radius"] = max(front, rear, half)+.1
    return data
