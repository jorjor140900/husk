"""Make a standalone browser replay from a ROS1 bag, without ROS or a robot.

Compare the recorded scan with the nearest measured return in the same ring
and angular bins. This is an offline comparison, not a replacement safety scan
or a Nav2 costmap replay. Missing returns remain missing.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from husky_drivable_area.collision import scan_points
from husky_drivable_area.vlp16 import read_timed_cloud, stamp_ns
from tools.inspect_vlp16_bag import static_transform, stats


def bin_ring(cloud, scan, ring):
    """Reproduce legacy float32 binning, then compare last vs nearest return.

    Retain the original bin count and +inf for absent returns. Do not remove
    floor/body points, fill missing rays, alter timestamps or infer free space.
    """
    count = len(scan.ranges)
    if (type(ring) is not int or not 0 <= ring < 16 or not 1 <= count <= 10000 or
            not np.isfinite([scan.angle_min, scan.angle_increment]).all() or
            scan.angle_increment <= 0):
        raise ValueError("invalid_offline_scan_geometry_or_ring")
    xyz = cloud.xyz[cloud.rings == ring].astype(np.float32)
    bins = np.floor((np.arctan2(xyz[:, 1], xyz[:, 0])-scan.angle_min)/scan.angle_increment).astype(int)
    inside = (bins >= 0) & (bins < count)
    bins, xyz = bins[inside], xyz[inside]
    distance = np.sqrt(xyz[:, 0]*xyz[:, 0]+xyz[:, 1]*xyz[:, 1])
    last, nearest = np.full(count, np.inf), np.full(count, np.inf)
    unique, reverse = np.unique(bins[::-1], return_index=True)
    last[unique] = distance[len(bins)-1-reverse]
    np.minimum.at(nearest, bins, distance)
    return last, nearest


def matches_recorded(predicted, ranges):
    ranges = np.asarray(ranges)
    finite = np.isfinite(predicted)
    return bool(np.array_equal(finite, np.isfinite(ranges)) and
                np.allclose(predicted[finite], ranges[finite], rtol=1e-5, atol=1e-5))


def scan_xy(ranges, scan, transform):
    """Same bin-angle approximation used by a LaserScan consumer, in base XY."""
    indices = np.flatnonzero(np.isfinite(ranges))
    angles = scan.angle_min+indices*scan.angle_increment
    xyz = np.column_stack([ranges[indices]*np.cos(angles),
                           ranges[indices]*np.sin(angles), np.zeros(len(indices))])
    base = xyz @ transform[:3, :3].T+transform[:3, 3]
    return np.round(base[:, :2], 4).tolist()


def cloud_context(cloud, transform, selected_ring):
    # Sample each ring separately; stride=16 over an interleaved cloud could
    # accidentally show just one ring. This only limits browser drawing.
    base = cloud.xyz @ transform[:3, :3].T+transform[:3, 3]
    context = []
    for ring in np.unique(cloud.rings):
        indices = np.flatnonzero((cloud.rings == ring) & (np.abs(base[:, 1]) < .6))
        indices = indices[::max(1, int(np.ceil(len(indices)/100)))]
        context.extend(np.round(base[indices][:, [0, 2]], 4).tolist())
    selected = np.flatnonzero((cloud.rings == selected_ring) & (np.abs(base[:, 1]) < .6))
    return context, np.round(base[selected][:, [0, 2]], 4).tolist()


def build_replay(path):
    from rosbags.highlevel import AnyReader

    records, counts = defaultdict(list), Counter()
    wanted = {"/scan", "/velodyne_points", "/tf_static", "/odometry/filtered"}
    with AnyReader([path]) as reader:
        for connection in reader.connections:
            counts[connection.topic] += connection.msgcount
        for connection, received, raw in reader.messages(
                connections=[c for c in reader.connections if c.topic in wanted]):
            records[connection.topic].append((received, reader.deserialize(raw, connection.msgtype)))
    for name in ("/scan", "/velodyne_points", "/tf_static"):
        if not records[name]:
            raise ValueError("recording_missing_topic:"+name)
    clouds = {stamp_ns(m.header.stamp): read_timed_cloud(m) for _, m in records["/velodyne_points"]}
    static = [m for _, m in records["/tf_static"]]
    failures, metadata, frames, deltas = Counter(), Counter(), [], []
    occupied_bins = missing_bins = 0
    first_stamp = stamp_ns(records["/scan"][0][1].header.stamp)
    for _, scan in records["/scan"]:
        source_stamp = stamp_ns(scan.header.stamp)
        cloud = clouds.get(source_stamp)
        if cloud is None:
            failures["no_cloud_at_exact_scan_stamp"] += 1
            continue
        if scan.header.frame_id.lstrip("/") != cloud.frame_id:
            failures["scan_cloud_frame_mismatch"] += 1
            continue
        try:
            transform = static_transform(static, "base_link", cloud.frame_id)
        except ValueError as exc:
            failures[str(exc)] += 1
            continue
        candidates = []
        for ring in range(16):
            last, nearest = bin_ring(cloud, scan, ring)
            if matches_recorded(last, scan.ranges):
                candidates.append((ring, last, nearest))
        if len(candidates) != 1:
            failures["scan_ring_not_uniquely_reproduced"] += 1
            continue
        ring, last, nearest = candidates[0]
        finite = np.isfinite(last)
        difference = last[finite]-nearest[finite]
        # Refuse a purported comparison that drops a return or moves it farther.
        if not np.array_equal(finite, np.isfinite(nearest)) or np.any(difference < 0):
            raise ValueError("nearest_return_invariant_failed")
        deltas.extend(difference.tolist())
        occupied_bins += int(finite.sum())
        missing_bins += int(np.isposinf(nearest).sum())
        try:
            scan_points(scan.ranges, scan.angle_min, scan.angle_max, scan.angle_increment,
                        scan.range_min, scan.range_max, scan.time_increment)
            reason = "accepted_by_metadata_check_only"
        except ValueError as exc:
            reason = str(exc)
        metadata[reason] += 1
        context, selected = cloud_context(cloud, transform, ring)
        frames.append(dict(t=round((source_stamp-first_stamp)/1e9, 6),
            source_stamp_ns=str(source_stamp), ring=ring,
            recorded=scan_xy(np.asarray(scan.ranges), scan, transform),
            nearest=scan_xy(nearest, scan, transform), context=context, selected=selected,
            changed_over_1cm=int((difference > .01).sum()),
            changed_over_1m=int((difference > 1).sum()),
            max_difference_m=round(float(difference.max()), 4),
            missing_bins=int(np.isposinf(nearest).sum()), metadata_reason=reason))
    if not frames:
        raise ValueError("no_reproducible_scan_cloud_pairs:"+json.dumps(failures))
    odom_positions = [[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                      for _, m in records["/odometry/filtered"]]
    extent = np.ptp(odom_positions, axis=0).tolist() if odom_positions else None
    yaws = [2*np.arctan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)
            for _, m in records["/odometry/filtered"]]
    yaw_extent = float(np.ptp(np.unwrap(yaws))) if yaws else None
    summary = dict(source_file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        message_counts=dict(counts), frames_compared=len(frames), skipped_scans=dict(failures),
        matched_rings=sorted({f["ring"] for f in frames}),
        finite_bin_occurrences=occupied_bins, missing_bin_occurrences_preserved=missing_bins,
        farther_than_nearest_counts={str(limit): int((np.asarray(deltas) > limit).sum())
                                    for limit in (.01, .05, .1, .5, 1, 5, 10)},
        last_minus_nearest_m=stats(deltas), metadata_checks=dict(metadata),
        odometry_position_extent_m=extent, odometry_yaw_extent_rad=yaw_extent,
        runtime_changed=False, ready_for_control=False,
        limits=["Local base-frame snapshots, not a Nav2 costmap or localization replay.",
                "Nearest comparison uses only the uniquely reproduced scan ring; low objects may be unseen.",
                "No floor/body removal, no missing-ray filling, no change to source stamps or safety gates.",
                "No per-point deskew in these plots. Stationary data cannot validate moving operation.",
                "This view does not replay synchronized images, global costmaps or map-to-odom localization."])
    return dict(summary=summary, frames=frames)


def write_replay(replay, output):
    template = Path(__file__).with_name("lidar_replay.html").read_text()
    # The source filename is untrusted: prevent closing the inert script element.
    data = json.dumps(replay, ensure_ascii=True, allow_nan=False, separators=(",", ":")).replace("<", "\\u003c")
    (output / "replay.html").write_text(template.replace("__REPLAY_DATA__", data))
    (output / "report.json").write_text(json.dumps(replay["summary"], indent=2, allow_nan=False)+"\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not args.bag.is_file():
        parser.error("bag file does not exist: "+str(args.bag))
    if args.output.exists():
        parser.error("choose a new output directory: "+str(args.output))
    replay = build_replay(args.bag)
    args.output.mkdir(parents=True)
    (args.output / "COLCON_IGNORE").touch()
    write_replay(replay, args.output)
    print(json.dumps(dict(html=str(args.output / "replay.html"),
                          report=str(args.output / "report.json"),
                          frames_compared=replay["summary"]["frames_compared"],
                          runtime_changed=False), indent=2))


if __name__ == "__main__":
    main()
