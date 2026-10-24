"""Offline WebSocket/DDS link test against a private receiver, never a robot."""
import argparse
import json
import math
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
import uuid

from husky_link_core.protocol import CommandWatchdog, dumps, loads, stamp_dict, stamp_ns
from .robot_preflight import save_report


class LocalReceiver:
    """An explicit rosbridge-protocol fixture with a separately ticking gate."""
    def __init__(self, frame_prefix):
        from websockets.sync.server import serve
        self.frames = frame_prefix
        self.lock, self.stop = threading.RLock(), threading.Event()
        self.gate = CommandWatchdog(frame_prefix+"/base")
        self.commands, self.outputs, self.connections = [], [], []
        self.publish_state = True
        self.server = serve(self.handler, "127.0.0.1", 0, max_size=2*1024*1024)
        self.port = self.server.socket.getsockname()[1]
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.clock_thread = threading.Thread(target=self.clock, daemon=True)
        self.server_thread.start()
        self.clock_thread.start()

    def clock(self):
        while not self.stop.wait(0.01):
            with self.lock:
                value = self.gate.tick(time.time_ns(), time.monotonic())
                self.outputs.append((time.monotonic(), value))

    def enable(self):
        with self.lock:
            self.gate.set_enabled(True, time.monotonic())

    def packets(self):
        stamp = stamp_dict(time.time_ns())
        header = {"seq": 0, "stamp": stamp, "frame_id": "/"+self.frames+"/odom"}
        xyz, quaternion = {"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        with self.lock:
            state = {"protocol": 1, "stamp": stamp, "armed": self.gate.armed, "lease": self.gate.lease,
                "reason": self.gate.reason, "driver_output_enabled": False, "output_topic": "/husky_vision/watchdog_preview",
                "base_frame": self.gate.base_frame, "max_linear_mps": 0.1, "max_angular_rps": 0.3, "command_timeout_sec": 0.25}
        odom = {"header": header, "child_frame_id": "/"+self.frames+"/base",
            "pose": {"pose": {"position": xyz, "orientation": quaternion}, "covariance": [0.0]*36},
            "twist": {"twist": {"linear": xyz, "angular": xyz}, "covariance": [0.0]*36}}
        scan = {"header": {**header, "frame_id": self.frames+"/laser"}, "angle_min": -math.pi,
            "angle_max": -math.pi+719*2*math.pi/720, "angle_increment": 2*math.pi/720,
            "time_increment": 0.0, "scan_time": 0.05, "range_min": 0.05, "range_max": 8.0,
            "ranges": [float("inf")]*719+[float("nan")], "intensities": []}
        dynamic = {"transforms": [{"header": header, "child_frame_id": self.frames+"/base",
                                    "transform": {"translation": xyz, "rotation": quaternion}}]}
        static = {"transforms": [{"header": {**header, "frame_id": self.frames+"/base", "stamp": stamp_dict(0)},
                                  "child_frame_id": self.frames+"/laser", "transform": {"translation": xyz, "rotation": quaternion}}]}
        packets = {kind+"_json": {"protocol": 1, "kind": kind, "msg": value} for kind, value in
                   (("odom", odom), ("scan", scan), ("tf", dynamic), ("tf_static", static))}
        if self.publish_state:
            packets["state"] = state
        return packets

    def handler(self, connection):
        from websockets.exceptions import ConnectionClosed
        self.connections.append(connection)
        subscribed, sent_at = set(), float("-inf")
        try:
            while not self.stop.is_set():
                try:
                    message = loads(connection.recv(timeout=0.01))
                    if message.get("op") == "subscribe":
                        subscribed.add(message["topic"])
                    elif message.get("op") == "publish":
                        assert message["topic"] == "/husky_vision/command"
                        packet = loads(message["msg"]["data"])
                        with self.lock:
                            self.commands.append(packet)
                            self.gate.accept(packet, time.time_ns(), time.monotonic())
                except TimeoutError:
                    pass
                if time.monotonic()-sent_at >= 0.05:
                    for suffix, packet in self.packets().items():
                        topic = "/husky_vision/"+suffix
                        if topic in subscribed:
                            connection.send(json.dumps({"op": "publish", "topic": topic, "msg": {"data": dumps(packet)}}))
                    sent_at = time.monotonic()
        except ConnectionClosed:
            pass

    def close(self):
        self.stop.set()
        for connection in self.connections:
            connection.close()
        self.server.shutdown()
        self.clock_thread.join(timeout=2)
        self.server_thread.join(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="outputs/robot-link-check")
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    ns = "link_test_"+uuid.uuid4().hex[:10]
    receiver = LocalReceiver(ns)
    rclpy.init()
    node = Node("offline_link_probe", namespace=ns)
    statuses, odometry, scans, sent_stamps = [], [], [], set()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    subs = [node.create_subscription(String, "status", lambda m: statuses.append(json.loads(m.data)), 1),
            node.create_subscription(Odometry, "odom", odometry.append, qos), node.create_subscription(LaserScan, "scan", scans.append, qos)]
    command_pub = node.create_publisher(TwistStamped, "filtered_command", 1)
    state = {"publish": True}
    def command():
        if state["publish"]:
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = ns+"/base"
            message.twist.linear.x, message.twist.angular.z = 0.08, 0.10
            sent_stamps.add(message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec)
            command_pub.publish(message)
    timer = node.create_timer(0.05, command)
    package = __package__
    child = subprocess.Popen([sys.executable, "-c", f"from {package}.robot_bridge_node import main; main()", "--ros-args",
        "-r", f"__ns:=/{ns}", "-p", f"bridge_url:=ws://127.0.0.1:{receiver.port}", "-p", "forward_commands:=true",
        "-p", f"base_frame:={ns}/base", "-p", "expected_output_topic:=/husky_vision/watchdog_preview",
        "-p", f"command_topic:=/{ns}/filtered_command"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    client = node.create_client(SetBool, "set_enabled")
    checks, failure, disconnect_stop_ms = [], None, None
    def wait(predicate, seconds=8):
        end = time.monotonic()+seconds
        while not predicate() and time.monotonic() < end:
            if child.poll() is not None:
                raise RuntimeError("bridge node exited")
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate(), statuses[-1:]
    def check(label):
        checks.append(label)
        print("PASS: "+label, flush=True)
    def enable():
        assert client.wait_for_service(timeout_sec=3)
        request = SetBool.Request(data=True)
        future = client.call_async(request)
        wait(future.done)
        assert future.result().success, future.result().message
    try:
        wait(lambda: statuses and statuses[-1]["connected"] and odometry and scans)
        assert not statuses[-1]["enabled"] and not receiver.commands
        assert odometry[-1].header.frame_id == ns+"/odom" and odometry[-1].child_frame_id == ns+"/base"
        assert math.isinf(scans[-1].ranges[0]) and math.isnan(scans[-1].ranges[-1])
        assert scans[-1].time_increment == 0
        check("DDS telemetry, frame normalization and scan +inf/NaN preserved; forwarding starts disabled")
        enable()
        receiver.enable()
        wait(lambda: receiver.commands and any(value and value[0] > 0 for _, value in receiver.outputs))
        assert all(stamp_ns(packet["stamp"]) in sent_stamps for packet in receiver.commands)
        check("manual link enable + receiver arm forwards original filtered-command stamps to private receiver")
        state["publish"] = False
        wait(lambda: not receiver.gate.armed and statuses[-1]["enabled"] is False)
        check("filtered-command loss stops and disarms locally; remote disarm drops the client latch")
        state["publish"] = True
        count = len(receiver.commands)
        receiver.enable()
        until = time.monotonic()+0.4
        wait(lambda: time.monotonic() >= until)
        assert len(receiver.commands) == count
        check("fresh input and a new receiver lease alone cannot restart forwarding")
        enable()
        wait(lambda: len(receiver.commands) > count)
        receiver.publish_state = False
        wait(lambda: not receiver.gate.armed and not statuses[-1]["enabled"])
        check("receiver heartbeat loss blocks forwarding despite other telemetry remaining live")
        receiver.publish_state = True
        wait(lambda: statuses[-1]["reason"] == "ok")
        enable()
        receiver.enable()
        count = len(receiver.commands)
        wait(lambda: len(receiver.commands) > count and receiver.outputs[-1][1] and receiver.outputs[-1][1][0] > 0)
        disconnected_at = time.monotonic()
        # Drop TCP immediately, rather than timing a graceful WebSocket close
        # handshake. Measure the independently recorded receiver output time.
        receiver.connections[-1].socket.shutdown(socket.SHUT_RDWR)
        wait(lambda: not receiver.gate.armed, seconds=2)
        zero_at = next(when for when, value in receiver.outputs if when >= disconnected_at and value == (0.0, 0.0))
        disconnect_stop_ms = (zero_at-disconnected_at)*1000
        assert disconnect_stop_ms <= 500, f"local stop took {disconnect_stop_ms:.0f}ms"
        wait(lambda: statuses[-1]["connected"] and not statuses[-1]["enabled"])
        check("WebSocket loss stops the independent receiver; automatic reconnect remains disabled")
        assert all(packet["base_frame"] == ns+"/base" for packet in receiver.commands)
    except Exception as exc:
        failure = traceback.format_exc()
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        try:
            output, _ = child.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill(); output, _ = child.communicate()
        receiver.close()
        node.destroy_node()
        rclpy.try_shutdown()
    report = {"kind": "offline_rosbridge_link_self_test", "result": "failed" if failure else "passed",
              "checks": checks, "error": failure, "namespace": ns,
              "disconnect_to_zero_ms": disconnect_stop_ms,
              "physical_robot_used": False, "receiver": "localhost WebSocket fixture + shared monotonic watchdog"}
    path = save_report(args.output_dir, report, "robot-link")
    print("Report: "+str(path))
    if failure:
        print(output[-6000:], file=sys.stderr)
        print(failure, file=sys.stderr)
        return 1
    return 0
