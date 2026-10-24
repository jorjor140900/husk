"""Measure synthetic tracking and exercise stop behavior in a private namespace.

Run from a ROS-sourced Python environment. --package husky_vision checks an
installed exported workspace. This never publishes to real robot cmd_vel topics.
"""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path as FilePath
import signal
import subprocess
import sys
import time
import uuid

from .control import ControllerConfig
from .control_report import summarize_tracking, write_report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", choices=("husky_drivable_area", "husky_vision"), default=__package__.split(".")[0])
    parser.add_argument("--tracking-seconds", type=float, default=8.0, help="tracking observation duration, 4..60 seconds")
    parser.add_argument("--max-final-error-m", type=float, default=0.15, help="synthetic tail-mean tracking error criterion")
    parser.add_argument("--config", type=FilePath, help="controller ROS parameter YAML; defaults to installed config/controller.yaml")
    parser.add_argument("--output-dir", type=FilePath, default=FilePath("outputs/control-check"))
    args = parser.parse_args(argv)
    import math
    import yaml
    if not 4 <= args.tracking_seconds <= 60 or not math.isfinite(args.max_final_error_m) or args.max_final_error_m <= 0:
        parser.error("tracking-seconds must be 4..60; max-final-error-m must be finite and positive")
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
        try:
            args.config = FilePath(get_package_share_directory(args.package)) / "config/controller.yaml"
        except PackageNotFoundError:
            args.config = FilePath(__file__).resolve().parents[1] / "config/controller.yaml"
    try:
        args.config = args.config.expanduser().resolve()
        parameters_used = yaml.safe_load(args.config.read_text())["/**"]["ros__parameters"]
        if not isinstance(parameters_used, dict):
            raise ValueError("/**/ros__parameters must be a mapping")
        json.dumps(parameters_used, allow_nan=False)
        config = ControllerConfig(**{name: parameters_used.get(name, default)
                                     for name, default in asdict(ControllerConfig()).items()})
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        parser.error(f"cannot read controller config: {exc}")
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rcl_interfaces.srv import SetParameters
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros import TransformBroadcaster

    namespace = "control_probe_" + uuid.uuid4().hex[:10]
    odom_frame, base_frame = namespace + "/odom", namespace + "/base_link"
    children = []
    report = {"schema_version": 1, "result": "running", "scope": "synthetic_ros2_kinematic_control",
              "physical_robot_validated": False, "namespace": namespace,
              "started_utc": datetime.now(timezone.utc).isoformat(), "package": args.package,
              "config_file": str(args.config), "controller_parameters": parameters_used,
              "tracking": None, "stop_observations": {}, "checks": []}
    report["effective_overrides"] = {"odom_frame": odom_frame, "base_frame": base_frame,
        "start_enabled": False, "path_topic": "reference", "odom_topic": "odom", "permission_topic": "motion_permitted"}
    report["stop_latency_scope"] = "service request to received zero command/status; not hardware braking time"
    current_check = "startup"
    started = time.monotonic()
    rclpy.init()
    node = Node("acceptance_probe", namespace=namespace)
    status, commands, odometry = [], [], []
    subscriptions = [
        node.create_subscription(String, "control/status", lambda m: status.append(json.loads(m.data)), 1),
        node.create_subscription(Twist, "control/cmd_vel", commands.append, 1),
        node.create_subscription(Odometry, "odom", odometry.append, 1),
    ]
    reference_pub = node.create_publisher(Path, "reference", 1)
    enable = node.create_client(SetBool, "control/set_enabled")
    stop = node.create_client(SetBool, "simulation/set_stop")
    parameters = node.create_client(SetParameters, "control_simulator/set_parameters")
    tf = TransformBroadcaster(node)

    def wait(predicate, seconds=6):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            if any(child.poll() is not None for child in children):
                raise RuntimeError("a ROS child exited unexpectedly")
            rclpy.spin_once(node, timeout_sec=0.025)
        if not predicate():
            raise AssertionError(f"timeout; latest controller status: {status[-1:]}")

    def settle(seconds=0.15):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if any(child.poll() is not None for child in children):
                raise RuntimeError("a ROS child exited unexpectedly")
            rclpy.spin_once(node, timeout_sec=0.02)

    def call(client, request):
        wait(lambda: client.service_is_ready())
        future = client.call_async(request)
        wait(future.done)
        result = future.result()
        if hasattr(result, "success"):
            assert result.success, result.message
        return result

    def stream(name, value):
        request = SetParameters.Request(parameters=[Parameter(name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))])
        result = call(parameters, request)
        assert all(r.successful for r in result.results), result

    def reason(expected):
        wait(lambda: status and status[-1]["reason"] == expected)
        if expected != "tracking":
            assert status[-1]["linear_x"] == status[-1]["angular_z"] == 0, status[-1]
            wait(lambda: commands and commands[-1].linear.x == commands[-1].angular.z == 0)

    def record_stop(key, requested_at, expected, configured_timeout):
        elapsed = time.monotonic() - requested_at
        budget = configured_timeout + max(0.25, 3 / float(parameters_used.get("control_rate_hz", 20.0)))
        report["stop_observations"][key] = {"request_to_zero_seconds": elapsed, "budget_seconds": budget, "reason": expected}
        if elapsed > budget:
            raise AssertionError(f"{key}: observed stop {elapsed:.3f}s exceeds demo budget {budget:.3f}s")

    def send_path(frame=odom_frame, stamp=None, points=None):
        message = Path()
        message.header.frame_id = frame
        message.header.stamp = stamp or node.get_clock().now().to_msg()
        if points is None:
            p = odometry[-1].pose.pose.position
            points = [(p.x + i*0.1, p.y) for i in range(20)]
        for x, y in points:
            pose = PoseStamped(header=message.header)
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        reference_pub.publish(message)

    try:
        for module in ("controller_node", "control_simulator"):
            config_args = (["--params-file", str(args.config), "-p", "start_enabled:=false", "-p", "path_topic:=reference",
                            "-p", "odom_topic:=odom", "-p", "permission_topic:=motion_permitted"]
                           if module == "controller_node" else [])
            children.append(subprocess.Popen([
                sys.executable, "-c", f"from {args.package}.{module} import main; main()",
                "--ros-args", "-r", f"__ns:=/{namespace}",
                *config_args,
                "-p", f"odom_frame:={odom_frame}", "-p", f"base_frame:={base_frame}",
            ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True))
        wait(lambda: len(status) >= 8 and len(commands) >= 8 and odometry)
        assert all(c.linear.x == c.angular.z == 0 for c in commands)
        assert status[-1]["reason"] == "disabled"
        report["checks"].append("disabled_output_zero")
        print("PASS: disabled controller emits zero while inputs are present", flush=True)
        current_check = "tracking"
        call(enable, SetBool.Request(data=True))
        reason("tracking")
        start_x = odometry[-1].pose.pose.position.x
        first_status, tracking_start = len(status), time.monotonic()
        settle(args.tracking_seconds)
        observed = status[first_status:]
        if not observed or any(s["reason"] not in ("tracking", "goal_reached") for s in observed):
            raise AssertionError(f"tracking interrupted: {sorted({s['reason'] for s in observed})}")
        errors = [s["cross_track_m"] for s in observed if s["cross_track_m"] is not None]
        report["tracking"] = summarize_tracking(errors)
        report["tracking"].update({"observation_seconds": time.monotonic() - tracking_start,
            "distance_forward_m": odometry[-1].pose.pose.position.x - start_x,
            "max_final_error_m": args.max_final_error_m,
            "error_source": "controller-reported distance to reference polyline using simulated odometry"})
        if len(errors) < 20 or report["tracking"]["tail_mean_m"] > args.max_final_error_m:
            raise AssertionError(f"synthetic tracking criterion not met: {report['tracking']}")
        if report["tracking"]["distance_forward_m"] <= 0.02:
            raise AssertionError("simulation made no forward progress")
        assert all(0 <= c.linear.x <= config.cruise_speed_mps + 1e-9 and abs(c.angular.z) <= config.max_angular_speed_rps + 1e-9 for c in commands)
        report["checks"].extend(["tracking_convergence", "command_speed_limits"])
        print(f"PASS: tracking RMSE {report['tracking']['rmse_m']:.3f} m | tail mean {report['tracking']['tail_mean_m']:.3f} m", flush=True)
        if status[-1]["reason"] == "goal_reached":
            from std_srvs.srv import Trigger
            reset = node.create_client(Trigger, "simulation/reset")
            call(reset, Trigger.Request())
            reason("tracking")
        current_check = "software_stop"
        fault_started = time.monotonic()
        call(stop, SetBool.Request(data=True))
        reason("motion_not_permitted")
        record_stop("software_stop", fault_started, "motion_not_permitted", 0.0)
        report["checks"].append("software_stop_zero")
        call(stop, SetBool.Request(data=False))
        reason("tracking")
        for name, expected in (("publish_reference", "stale_path"),
                               ("publish_odometry", "stale_odometry"),
                               ("publish_permission", "permission_timeout")):
            current_check = expected
            fault_started = time.monotonic()
            stream(name, False)
            reason(expected)
            timeout_key, default_timeout = {"stale_path": ("path_timeout_sec", 0.5),
                "stale_odometry": ("odom_timeout_sec", 0.3), "permission_timeout": ("permission_timeout_sec", 0.3)}[expected]
            record_stop(expected, fault_started, expected, float(parameters_used.get(timeout_key, default_timeout)))
            report["checks"].append(expected + "_zero")
            stream(name, True)
            reason("tracking")
        print("PASS: software stop, path/odometry/permission loss, and fresh-input recovery", flush=True)
        current_check = "invalid_paths_and_tf"
        stream("publish_reference", False)
        reason("stale_path")
        send_path(points=[])
        reason("empty_path")
        send_path(frame=namespace + "/missing_frame")
        reason("tf_unavailable")
        send_path(stamp=(node.get_clock().now() - Duration(seconds=2)).to_msg())
        reason("stale_path")
        send_path(points=[(float("nan"), 0), (1, 0)])
        reason("invalid_path")
        # Two dynamic transforms: old camera position vs current camera position.
        # Lookup must use the path's source stamp, not the latest transform.
        source_frame = namespace + "/moving_reference"
        capture = node.get_clock().now() - Duration(seconds=0.1)
        position = odometry[-1].pose.pose.position
        transforms = []
        for stamp, x in ((capture.to_msg(), position.x), (node.get_clock().now().to_msg(), position.x + 2)):
            transform = TransformStamped()
            transform.header.frame_id, transform.header.stamp = odom_frame, stamp
            transform.child_frame_id = source_frame
            transform.transform.translation.x, transform.transform.translation.y = x, position.y
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        tf.sendTransform(transforms)
        settle(0.08)
        send_path(frame=source_frame, stamp=capture.to_msg(), points=[(i*0.1, 0) for i in range(20)])
        reason("tracking")  # Latest TF would be 2 m away and rejected.
        p = odometry[-1].pose.pose.position
        send_path(points=[(p.x - 0.1, p.y), (p.x + 0.05, p.y)])
        reason("goal_reached")
        call(enable, SetBool.Request(data=False))
        reason("disabled")
        report["checks"].extend(["empty_path_zero", "missing_tf_zero", "old_path_zero", "nonfinite_path_zero",
                                  "capture_time_tf", "goal_stop_zero", "disarm_zero", "fresh_input_recovery"])
        print("PASS: invalid/stale paths, capture-time TF, goal stop, and disarm", flush=True)
        report["result"] = "passed"
    except BaseException as exc:
        report["result"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        report["failed_check"] = current_check
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        for child in children:
            child.send_signal(signal.SIGINT) if child.poll() is None else None
        for child in children:
            try:
                output, _ = child.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                output, _ = child.communicate()
            if sys.exc_info()[0] is not None or child.returncode not in (0, -signal.SIGINT):
                print(output[-6000:], file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        report.update({"elapsed_seconds": time.monotonic() - started, "command_samples": len(commands), "status_samples": len(status)})
        path = write_report(args.output_dir, report)
        print(f"Report: {path}", flush=True)
        print(json.dumps({"result": report["result"], "checks_passed": len(report["checks"]), "report": str(path)}), flush=True)


if __name__ == "__main__":
    main()
