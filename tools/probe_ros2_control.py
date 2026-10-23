"""Exercise controller + kinematic plant in real ROS 2 DDS, in a private namespace.

Run from a ROS-sourced Python environment. --package husky_vision checks an
installed exported workspace. This never publishes to real robot cmd_vel topics.
"""

import argparse
import json
from pathlib import Path as FilePath
import signal
import subprocess
import sys
import time
import uuid

ROOT = FilePath(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", choices=("husky_drivable_area", "husky_vision"), default="husky_drivable_area")
    args = parser.parse_args()
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
            children.append(subprocess.Popen([
                sys.executable, "-c", f"from {args.package}.{module} import main; main()",
                "--ros-args", "-r", f"__ns:=/{namespace}",
                "-p", f"odom_frame:={odom_frame}", "-p", f"base_frame:={base_frame}",
            ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True))
        wait(lambda: len(status) >= 8 and len(commands) >= 8 and odometry)
        assert all(c.linear.x == c.angular.z == 0 for c in commands)
        assert status[-1]["reason"] == "disabled"
        print("PASS: disabled controller emits zero while inputs are present", flush=True)
        call(enable, SetBool.Request(data=True))
        reason("tracking")
        start_x = odometry[-1].pose.pose.position.x
        wait(lambda: odometry[-1].pose.pose.position.x > start_x + 0.12, seconds=4)
        assert all(0 <= c.linear.x <= 0.2 + 1e-9 and abs(c.angular.z) <= 0.5 + 1e-9 for c in commands)
        print("PASS: separate ROS processes close the odometry loop within speed limits", flush=True)
        call(stop, SetBool.Request(data=True))
        reason("motion_not_permitted")
        call(stop, SetBool.Request(data=False))
        reason("tracking")
        for name, expected in (("publish_reference", "stale_path"),
                               ("publish_odometry", "stale_odometry"),
                               ("publish_permission", "permission_timeout")):
            stream(name, False)
            reason(expected)
            stream(name, True)
            reason("tracking")
        print("PASS: software stop, path/odometry/permission loss, and fresh-input recovery", flush=True)
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
        print("PASS: invalid/stale paths, capture-time TF, goal stop, and disarm", flush=True)
        print(json.dumps({"namespace": namespace, "command_samples": len(commands), "status_samples": len(status), "result": "passed"}), flush=True)
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


if __name__ == "__main__":
    main()
