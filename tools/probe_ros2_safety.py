"""Check the LaserScan stop filter over real ROS messages in a private namespace."""

import argparse
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", default="husky_drivable_area", choices=("husky_drivable_area", "husky_vision"))
    args = parser.parse_args()
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    import json

    namespace = "safety_probe_" + uuid.uuid4().hex[:10]
    odom_frame, base_frame = namespace + "/odom", namespace + "/base_link"
    rclpy.init()
    node = Node("acceptance_probe", namespace=namespace)
    status, commands, stamped_commands = [], [], []
    subs = [node.create_subscription(String, "safety/status", lambda m: status.append(json.loads(m.data)), 1),
            node.create_subscription(Twist, "control/cmd_vel", commands.append, 1),
            node.create_subscription(TwistStamped, "control/cmd_vel_stamped", stamped_commands.append, 1)]
    pubs = {"scan": node.create_publisher(LaserScan, "scan", 1), "odom": node.create_publisher(Odometry, "odom", 1),
            "request": node.create_publisher(Twist, "control/requested_cmd_vel", 1),
            "permission": node.create_publisher(Bool, "motion_permitted", 1)}
    state = {"scan": True, "odom": True, "request": True, "permission": True, "permitted": True,
             "v": 0.2, "w": 0.0, "measured_v": 0.0, "point": None, "invalid": None,
             "scan_frame": base_frame, "range_max": 8.0, "time_increment": 0.0}
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = odom_frame, base_frame
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)

    def publish():
        stamp = node.get_clock().now().to_msg()
        if state["permission"]:
            pubs["permission"].publish(Bool(data=state["permitted"]))
        if state["odom"]:
            odom = Odometry()
            odom.header.frame_id, odom.header.stamp, odom.child_frame_id = odom_frame, stamp, base_frame
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = state["measured_v"]
            pubs["odom"].publish(odom)
        if state["request"]:
            command = Twist()
            command.linear.x, command.angular.z = state["v"], state["w"]
            pubs["request"].publish(command)
        if state["scan"]:
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = state["scan_frame"], stamp
            scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/720
            scan.angle_max = scan.angle_min + 719*scan.angle_increment
            scan.range_min, scan.range_max = 0.05, state["range_max"]
            scan.time_increment = state["time_increment"]
            ranges = np.full(720, np.inf)
            if state["point"] is not None:
                x, y = state["point"]
                index = int(round((math.atan2(y, x)+math.pi)/scan.angle_increment)) % 720
                ranges[index] = math.hypot(x, y)
            if state["invalid"] is not None:
                ranges[0] = state["invalid"]
            scan.ranges = ranges.tolist()
            pubs["scan"].publish(scan)

    timer = node.create_timer(0.05, publish)
    child = subprocess.Popen([sys.executable, "-c", f"from {args.package}.safety_node import main; main()", "--ros-args",
        "-r", f"__ns:=/{namespace}", "-p", f"odom_frame:={odom_frame}", "-p", f"base_frame:={base_frame}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    checks = []

    def expect(expected):
        start = len(status)
        end = time.monotonic() + 5
        while time.monotonic() < end:
            if child.poll() is not None:
                raise RuntimeError("safety node exited")
            rclpy.spin_once(node, timeout_sec=0.02)
            if len(status) > start and status[-1]["reason"] == expected:
                if expected != "clear":
                    assert status[-1]["linear_x"] == status[-1]["angular_z"] == 0
                checks.append(expected)
                return
        raise AssertionError((expected, status[-1:]))

    try:
        expect("clear")
        state["point"] = (0.68, 0)
        expect("obstacle_in_sweep")
        state["point"] = (0.3, 0.53)
        expect("clear")
        state["v"], state["w"] = 0.0, 0.6
        expect("obstacle_in_sweep")
        state.update(point=(0.75, 0), v=0.0, w=0.0, measured_v=0.25)
        expect("obstacle_in_sweep")
        state.update(point=None, v=0.2, measured_v=0.0)
        expect("clear")
        for stream, reason in (("scan", "stale_scan"), ("odom", "stale_odometry"),
                               ("request", "stale_request"), ("permission", "permission_timeout")):
            state[stream] = False
            expect(reason)
            state[stream] = True
            expect("clear")
        state["permitted"] = False
        expect("motion_not_permitted")
        state["permitted"] = True
        expect("clear")
        state["invalid"] = float("nan")
        expect("invalid_scan_ranges")
        state["invalid"] = None
        expect("clear")
        state["scan_frame"] = namespace + "/missing"
        expect("scan_tf_unavailable")
        state["scan_frame"] = base_frame
        expect("clear")
        state["range_max"] = 0.5
        expect("sweep_outside_scan_range")
        state["range_max"] = 8.0
        state["time_increment"] = 0.0001
        expect("scan_requires_deskew")
        state["time_increment"] = 0.0
        expect("clear")
        assert any(c.linear.x > 0 for c in commands)
        assert any(c.twist.linear.x > 0 for c in stamped_commands)
        assert all(c.header.frame_id == base_frame and (c.header.stamp.sec or c.header.stamp.nanosec) for c in stamped_commands)
        print(json.dumps({"result": "passed", "checks": checks, "namespace": namespace}), flush=True)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        try:
            output, _ = child.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            output, _ = child.communicate()
        if sys.exc_info()[0] is not None or child.returncode not in (0, -signal.SIGINT):
            print(output[-6000:], file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
