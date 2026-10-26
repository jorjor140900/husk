"""Inspect a field bag offline; never arm, publish or infer free space.

python -m tools.inspect_vlp16_bag --bag recording.bag --output outputs/vlp16-review --plots
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

from husky_drivable_area.collision import scan_points
from husky_drivable_area.control import quaternion_matrix
from husky_drivable_area.vlp16 import deskew_returns, read_timed_cloud, stamp_ns


def stats(values):
    return dict(zip(("min", "median", "max"), map(float, np.percentile(values, [0, 50, 100])))) if len(values) else None


def static_transform(messages, source, target):
    graph = defaultdict(list)
    for message in messages:
        for transform in message.transforms:
            q, p = transform.transform.rotation, transform.transform.translation
            matrix = np.eye(4)
            matrix[:3, :3] = quaternion_matrix(q.x, q.y, q.z, q.w)
            matrix[:3, 3] = [p.x, p.y, p.z]
            parent, child = transform.header.frame_id.lstrip("/"), transform.child_frame_id.lstrip("/")
            graph[parent].append((child, matrix))
            graph[child].append((parent, np.linalg.inv(matrix)))
    queue, seen = [(source, np.eye(4))], set()
    for name, matrix in queue:
        if name == target:
            return matrix
        if name in seen:
            continue
        seen.add(name)
        queue.extend((child, matrix @ edge) for child, edge in graph[name] if child not in seen)
    raise ValueError("static_transform_missing:"+source+"->"+target)


def inspect(path):
    from rosbags.highlevel import AnyReader
    records = defaultdict(list)
    with AnyReader([path]) as reader:
        for connection, received, raw in reader.messages():
            records[connection.topic].append((received, reader.deserialize(raw, connection.msgtype)))
    required = ("/scan", "/velodyne_points", "/velodyne_packets", "/odometry/filtered", "/tf_static")
    missing = [name for name in required if not records[name]]
    if missing:
        raise ValueError("recording_missing_topics:"+",".join(missing))
    report = {"source_file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "message_counts": {k: len(v) for k, v in records.items()},
              "ready_for_autonomous_control": False, "runtime_adapter_connected": False}
    report["timing"] = {}
    for topic in required[:-1]:
        entries = records[topic]
        times = np.array([stamp_ns(m.header.stamp) for _, m in entries], dtype=np.int64)
        report["timing"][topic] = {"source_interval_ms": stats(np.diff(times)/1e6),
            "record_age_ms": stats([(t-stamp_ns(m.header.stamp))/1e6 for t, m in entries])}
    scans = [m for _, m in records["/scan"]]
    clouds = {stamp_ns(m.header.stamp): read_timed_cloud(m) for _, m in records["/velodyne_points"]}
    metadata_errors, ring_matches, spans, packet_errors, overwritten_near = Counter(), [], [], [], []
    all_ranges = np.concatenate([s.ranges for s in scans])
    for s in scans:
        try:
            scan_points(s.ranges, s.angle_min, s.angle_max, s.angle_increment, s.range_min, s.range_max, s.time_increment)
        except ValueError as exc:
            metadata_errors[str(exc)] += 1
        cloud = clouds.get(stamp_ns(s.header.stamp))
        if cloud is None:
            continue
        errors = []
        for ring in range(16):
            xyz = cloud.xyz[cloud.rings == ring]
            # Match the recorded C++ driver's float32 angular binning.
            xyz = xyz.astype(np.float32)
            bins = np.floor((np.arctan2(xyz[:, 1], xyz[:, 0])-s.angle_min)/s.angle_increment).astype(int)
            good = (bins >= 0) & (bins < len(s.ranges))
            predicted = np.full(len(s.ranges), np.inf)
            distances = np.sqrt(xyz[good, 0]*xyz[good, 0]+xyz[good, 1]*xyz[good, 1])
            valid_bins = bins[good]
            # Official legacy driver assigns ranges[bin] for every point;
            # the final return overwrites earlier returns, rather than min().
            unique, reverse_index = np.unique(valid_bins[::-1], return_index=True)
            predicted[unique] = distances[len(valid_bins)-1-reverse_index]
            if np.array_equal(np.isfinite(predicted), np.isfinite(s.ranges)) and np.allclose(
                    predicted[np.isfinite(predicted)], s.ranges[np.isfinite(predicted)], rtol=1e-5, atol=1e-5):
                errors.append(ring)
                nearest = np.full(len(s.ranges), np.inf)
                np.minimum.at(nearest, valid_bins, distances)
                finite = np.isfinite(nearest) & np.isfinite(s.ranges)
                overwritten_near.extend((s.ranges[finite]-nearest[finite])[s.ranges[finite]-nearest[finite] > .01].tolist())
        ring_matches.append(errors)
    for _, message in records["/velodyne_packets"]:
        header = stamp_ns(message.header.stamp)
        packet_times = [stamp_ns(p.stamp) for p in message.packets]
        spans.append((packet_times[-1]-packet_times[0])/1e6)
        cloud = clouds.get(header)
        if cloud is not None:
            packet_errors.append((cloud.offsets_sec.min()-(packet_times[0]-header)/1e9)*1e9)
    first = scans[0]
    report["scan"] = {"rays": len(first.ranges), "range_limits": [first.range_min, first.range_max],
        "angle_max_reported": first.angle_max,
        "angle_max_from_bins": first.angle_min+(len(first.ranges)-1)*first.angle_increment,
        "time_increment": first.time_increment, "scan_time": first.scan_time,
        "filter_rejections": dict(metadata_errors), "exact_matching_rings_per_scan": ring_matches,
        "bins_where_last_return_is_over_1cm_farther_than_nearest": len(overwritten_near),
        "overwritten_near_return_difference_m": stats(overwritten_near),
        "finite_ranges_m": stats(all_ranges[np.isfinite(all_ranges)]),
        "missing_bins_per_scan": stats([np.isposinf(s.ranges).sum() for s in scans]),
        "nan_count": int(np.isnan(all_ranges).sum())}
    cloud_list = list(clouds.values())
    xyz = np.concatenate([c.xyz for c in cloud_list])
    rings = np.concatenate([c.rings for c in cloud_list])
    report["point_cloud"] = {"counts": stats([len(c.xyz) for c in cloud_list]),
        "point_offsets_ms": stats(np.concatenate([c.offsets_sec*1000 for c in cloud_list])),
        "acquisition_span_ms": stats([np.ptp(c.offsets_sec)*1000 for c in cloud_list]),
        "packet_stamp_span_ms": stats(spans), "first_point_vs_first_packet_offset_error_ns": stats(packet_errors),
        "ring_elevation_deg": {int(r): stats(np.degrees(np.arctan2(
            xyz[rings == r, 2], np.linalg.norm(xyz[rings == r, :2], axis=1)))) for r in np.unique(rings)}}
    odom = [m for _, m in records["/odometry/filtered"]]
    odom_stamps = np.array([stamp_ns(m.header.stamp) for m in odom], dtype=np.int64)
    poses, speeds = [], []
    for message in odom:
        q, p = message.pose.pose.orientation, message.pose.pose.position
        matrix = quaternion_matrix(q.x, q.y, q.z, q.w)
        if abs(matrix[2, 2]-1) > 1e-3:
            raise ValueError("nonplanar_odometry_recording")
        poses.append([p.x, p.y, np.arctan2(matrix[1, 0], matrix[0, 0])])
        speeds.append([message.twist.twist.linear.x, message.twist.twist.angular.z])
    if any(m.header.frame_id.lstrip("/") != "odom" or m.child_frame_id.lstrip("/") != "base_link" for m in odom):
        raise ValueError("unexpected_recording_odometry_frames")
    transform = static_transform([m for _, m in records["/tf_static"]], "base_link", cloud_list[0].frame_id)
    failures, changes, example = Counter(), [], None
    for cloud in cloud_list:
        try:
            result = deskew_returns(cloud, odom_stamps, poses, transform)
            original = cloud.xyz @ transform[:3, :3].T+transform[:3, 3]
            changes.append(float(np.linalg.norm(result-original, axis=1).max()))
            if example is None:
                example = (cloud, result)
        except ValueError as exc:
            failures[str(exc)] += 1
    report["deskew_review"] = {"processed_clouds": len(changes), "rejected_clouds": dict(failures),
        "max_point_change_m": max(changes) if changes else None,
        "lidar_to_base": transform.tolist(), "odometry_pose_extent": np.ptp(poses, axis=0).tolist(),
        "odometry_max_abs_v_omega": np.abs(speeds).max(axis=0).tolist(),
        "moving_robot_validation": False}
    report["limitations"] = [
        "No runtime wiring or permission to move is added by this offline adapter.",
        "A single-ring scan is not volumetric obstacle coverage; absent returns are not proven free space.",
        "Point timestamps span a revolution despite LaserScan time_increment=0; use per-point time.",
        "Planar odometry interpolation is unverified on a moving robot, slopes or wheel slip.",
        "Measured mounting, footprint, ground/obstacle heights and independent permission remain necessary.",
    ]
    return report, example


def plot(example, output):
    if example is None:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cloud, base = example
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    top, side, timing = axes
    top.scatter(base[::8, 0], base[::8, 1], s=1, c="#b2bec3", label="All 16 rings")
    ring = cloud.rings == 8
    top.scatter(base[ring, 0], base[ring, 1], s=3, c="#d35400", label="Current /scan: ring 8")
    top.scatter([0], [0], marker="+", c="blue", label="base_link origin")
    top.set(xlim=(-4, 4), ylim=(-4, 4), xlabel="Base X / forward (m)", ylabel="Base Y / left (m)", title="Measured returns: top view")
    top.set_aspect("equal"); top.legend(fontsize=7, loc="upper left")
    subset = (abs(base[:, 1]) < .6) & (base[:, 0] >= 0) & (base[:, 0] <= 5)
    side.scatter(base[subset, 0], base[subset, 2], c=cloud.rings[subset], cmap="turbo", s=2)
    selected = subset & ring
    side.scatter(base[selected, 0], base[selected, 2], c="#d35400", s=6, label="Ring 8 (+1 degree)")
    side.axhline(0, color="blue", linewidth=.8, linestyle="--", label="base_link Z=0; not floor")
    side.set(xlim=(0, 5), ylim=(-.3, 2), xlabel="Base X / forward (m)", ylabel="Height relative to base_link (m)", title="Front strip: |Y| < 0.6 m")
    side.legend(fontsize=7, loc="upper left")
    timing.scatter(cloud.offsets_sec[::8]*1000, cloud.rings[::8], s=1, alpha=.5)
    timing.axvline(0, color="red", linewidth=1, label="Original header stamp")
    timing.set(xlabel="Point time minus cloud header (ms)", ylabel="Ring", title="Actual acquisition spans about 101 ms", yticks=[0, 4, 8, 12, 15])
    timing.legend(fontsize=7)
    for axis in axes: axis.grid(alpha=.2)
    fig.suptitle("Recorded Husky VLP16 — observation only; no motion validation")
    fig.savefig(output / "vlp16-review.png", dpi=160)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plots", action="store_true")
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "COLCON_IGNORE").touch()
    report, example = inspect(args.bag)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    if args.plots: plot(example, args.output)
    print(json.dumps({"report": str(args.output / "report.json"), "deskew_review": report["deskew_review"],
                      "filter_rejections": report["scan"]["filter_rejections"],
                      "ready_for_autonomous_control": False}, indent=2))


if __name__ == "__main__": main()
