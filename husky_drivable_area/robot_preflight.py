"""Read-only report of profile, bridged data, timestamps and required TF."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid

from .robot_profile import load_robot_profile


def save_report(directory, report, prefix):
    directory = Path(directory).expanduser().absolute()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "COLCON_IGNORE").touch()
    path = directory / (prefix+"-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")+"-"+uuid.uuid4().hex[:8]+".json")
    with path.open("x") as target:
        json.dump(report, target, indent=2, allow_nan=False)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--output-dir", default="outputs/robot-preflight")
    args = parser.parse_args()
    if not 2 <= args.duration <= 30:
        parser.error("duration must be 2..30 seconds")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener, TransformException
    from .collision import scan_points

    raw = json.loads(Path(args.profile).expanduser().read_text())
    profile_error = None
    try:
        profile, mount = load_robot_profile(args.profile)
    except (ValueError, KeyError, OSError) as exc:
        profile, mount, profile_error = raw, None, str(exc)
    rclpy.init()
    node = Node("robot_preflight_"+uuid.uuid4().hex[:8])
    latest, counts, ages, seen = {}, {}, {}, {}
    def received(kind, message):
        latest[kind] = (message, time.monotonic())
        counts[kind] = counts.get(kind, 0)+1
        if kind in ("odom", "scan", "path") and (kind != "path" or len(message.poses) >= 2):
            source = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            if source > 0 and source not in seen.setdefault(kind, set()):
                seen[kind].add(source)
                ages.setdefault(kind, []).append((node.get_clock().now().nanoseconds-source)/1e6)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    external = profile.get("robot", {}).get("external_permission_topic") or "/husky_safety/motion_permitted"
    subs = [node.create_subscription(cls, topic, lambda m, key=key: received(key, m), qos if key in ("odom", "scan") else 1)
            for key, cls, topic in (("link", String, "/husky_link/status"), ("odom", Odometry, "/husky_link/odom"),
                                    ("scan", LaserScan, "/husky_link/scan"), ("path", RosPath, "/drivable_area/reference_ground"),
                                    ("permission", Bool, external))]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    checks, detail = {}, {}
    started = time.monotonic()
    try:
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
        now, ros_ns = time.monotonic(), node.get_clock().now().nanoseconds
        checks["profile_complete"] = {"ok": profile_error is None, "reason": profile_error or "configured; physical checks recorded separately"}
        detail["field_measurements"] = profile.get("field_measurements", {})
        def fresh(kind, limit):
            if kind not in latest:
                return False, "no_messages"
            message, received_at = latest[kind]
            if kind == "path" and len(message.poses) < 2:
                return False, "empty_reference"
            source = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            age = (ros_ns-source)/1e9
            detail[kind+"_age_ms"] = age*1000
            return source > 0 and -0.05 <= age <= limit and now-received_at <= limit, "age_ms="+str(round(age*1000, 1))
        for kind, limit in (("odom", 0.3), ("scan", 0.3), ("path", 0.5)):
            ok, reason = fresh(kind, limit)
            checks[kind+"_fresh"] = {"ok": ok, "reason": reason}
        detail["source_age_at_delivery_ms"] = {}
        for kind, values in ages.items():
            ordered = sorted(values)
            budget = 500 if kind == "path" else 300
            p95 = ordered[max(0, math.ceil(0.95*len(ordered))-1)]
            detail["source_age_at_delivery_ms"][kind] = {"unique_samples": len(ordered), "p95": p95,
                "max": ordered[-1], "over_budget_samples": sum(value > budget or value < -50 for value in ordered)}
        path_ages = detail["source_age_at_delivery_ms"].get("path", {})
        checks["path_delivery_budget"] = {"ok": path_ages.get("unique_samples", 0) >= 3 and -50 <= path_ages.get("p95", 501) <= 500,
                                           "reason": str(path_ages) if path_ages else "need at least 3 valid reference samples"}
        odom = latest.get("odom", (None,))[0]
        expected = profile.get("robot", {})
        frames_match = odom is not None and odom.header.frame_id == expected.get("odom_frame") and odom.child_frame_id == expected.get("base_frame")
        checks["odometry_frames"] = {"ok": frames_match, "reason": "match profile" if frames_match else "odom parent/child differs from profile or no odometry"}
        state = json.loads(latest["link"][0].data) if "link" in latest else {}
        checks["bridge_state"] = {"ok": state.get("connected") is True and state.get("reason") == "ok" and
                                 "link" in latest and now-latest["link"][1] <= 0.3, "reason": state.get("reason", "no_link_status")}
        detail["receiver"] = state.get("receiver")
        detail["decode_errors"] = state.get("decode_errors", {})
        checks["decode"] = {"ok": bool(state) and not detail["decode_errors"], "reason": str(detail["decode_errors"])}
        compatible = "no_scan"
        if "scan" in latest:
            scan = latest["scan"][0]
            detail["scan"] = {"frame_id": scan.header.frame_id, "count": len(scan.ranges), "time_increment": scan.time_increment,
                              "angle_min": scan.angle_min, "angle_max": scan.angle_max, "angle_increment": scan.angle_increment}
            try:
                scan_points(scan.ranges, scan.angle_min, scan.angle_max, scan.angle_increment,
                            scan.range_min, scan.range_max, scan.time_increment)
                compatible = "compatible"
            except (ValueError, TypeError) as exc:
                compatible = str(exc)
        checks["scan_filter_compatible"] = {"ok": compatible == "compatible", "reason": compatible}
        for kind in ("scan", "path"):
            try:
                if not profile.get("robot", {}).get("odom_frame") or kind not in latest:
                    raise ValueError("profile/frame/message missing")
                message = latest[kind][0]
                buffer.lookup_transform(profile["robot"]["odom_frame"], message.header.frame_id, Time.from_msg(message.header.stamp))
                checks[kind+"_capture_time_tf"] = {"ok": True, "reason": "available"}
            except (TransformException, ValueError) as exc:
                checks[kind+"_capture_time_tf"] = {"ok": False, "reason": str(exc)[:200]}
        permission_fresh = "permission" in latest and now-latest["permission"][1] <= 0.3
        checks["external_permission_heartbeat"] = {"ok": permission_fresh,
            "reason": ("heartbeat present; value="+str(latest["permission"][0].data)) if permission_fresh else "missing/stale heartbeat on "+external}
        report = {"kind": "robot_io_preflight", "io_checks_passed": all(c["ok"] for c in checks.values()),
                  "ready_for_autonomous_control": False, "checks": checks, "details": detail,
                  "received_messages": counts, "duration_sec": args.duration,
                  "note": "Read-only observations; no robot arm, commands, clock adjustment or parameter changes performed."}
        path = save_report(args.output_dir, report, "robot-preflight")
        for key, check in checks.items():
            print(("PASS" if check["ok"] else "PENDING")+": "+key+" | "+check["reason"])
        print("Report: "+str(path))
        return 0 if report["io_checks_passed"] else 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
