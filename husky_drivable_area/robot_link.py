"""Bounded rosbridge String transport; disconnect always drops the enable latch."""

import json
import threading
import time
from urllib.parse import urlparse

from husky_link_core.protocol import PROTOCOL, dumps, finite, frame_id, loads, stamp_dict, stamp_ns


class RosbridgeLink:
    KINDS = ("odom", "scan", "tf", "tf_static", "permission", "image", "state")

    def __init__(self, url, *, prefix="/husky_vision", forward_commands=False, base_frame="",
                 expected_output_topic="", command_provider=None, ros_clock=None):
        parsed = urlparse(url)
        if parsed.scheme not in ("ws", "wss") or not parsed.hostname or parsed.username or parsed.password:
            raise ValueError("use a ws/wss URL without embedded credentials; default is an SSH-forwarded loopback socket")
        if prefix == "/" or not prefix.startswith("/") or any(c.isspace() for c in prefix) or any(c in prefix for c in "*?[]"):
            raise ValueError("absolute ros1_prefix without wildcards required")
        if forward_commands and (not base_frame or not expected_output_topic.startswith("/")):
            raise ValueError("forwarding requires measured base_frame and verified expected_output_topic")
        self.url, self.prefix = url, prefix.rstrip("/")
        self.forward_commands, self.base_frame = forward_commands, frame_id(base_frame) if base_frame else ""
        self.expected_output_topic = expected_output_topic
        self.provider, self.ros_clock = command_provider, ros_clock or time.time_ns
        self.lock, self.stop = threading.RLock(), threading.Event()
        self.connected = self.enabled = False
        self.receiver = None
        self.pending, self.error, self.sequence = {}, "not_connected", 0
        self.image_count = 0
        self.image_received_at = None
        self.image_source_age_ms = self.image_last_gap_ms = None
        self.image_max_gap_ms = 0.0
        self.stop_stamp = None
        self.thread = threading.Thread(target=self._run, daemon=True, name="ros1_link")

    def start(self):
        try:
            import websocket
        except ImportError as exc:
            raise ValueError("Install requirements-bridge.txt inside the ROS2 venv first") from exc
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)

    def _receiver_reason(self, now, ros_ns):
        if not self.connected:
            return "disconnected"
        if self.receiver is None:
            return "no_receiver_state"
        state, received = self.receiver
        if now-received > 0.3 or not -0.05 <= (ros_ns-stamp_ns(state["stamp"]))/1e9 <= 0.3:
            return "receiver_state_stale_or_clock_skew"
        if self.base_frame and frame_id(state["base_frame"]) != self.base_frame:
            return "receiver_frame_mismatch"
        if self.forward_commands and state["output_topic"] != self.expected_output_topic:
            return "receiver_output_topic_mismatch"
        if self.forward_commands and self.expected_output_topic != "/husky_vision/watchdog_preview" and not state["driver_output_enabled"]:
            return "receiver_driver_output_disabled"
        return "ok"

    def snapshot(self):
        with self.lock:
            reason = self._receiver_reason(time.monotonic(), self.ros_clock())
            return {"connected": self.connected, "enabled": self.enabled, "forward_commands": self.forward_commands,
                    "reason": reason, "transport_error": self.error,
                    "receiver": self.receiver[0].copy() if self.receiver else None,
                    "motion_permitted": reason == "ok" and self.forward_commands and self.enabled and self.receiver[0]["armed"],
                    "camera_transport": {"received_frames": self.image_count,
                        "source_age_at_receipt_ms": self.image_source_age_ms,
                        "last_gap_ms": self.image_last_gap_ms, "max_gap_ms": self.image_max_gap_ms,
                        "receipt_age_ms": (time.monotonic()-self.image_received_at)*1000 if self.image_received_at is not None else None}}

    def enable(self, enabled):
        with self.lock:
            if not enabled:
                self.enabled, self.stop_stamp = False, self.ros_clock()
                return True, "disabled; zero requested, robot-local watchdog remains active"
            if not self.forward_commands:
                return False, "observe mode: forward_commands is false"
            reason = self._receiver_reason(time.monotonic(), self.ros_clock())
            if reason != "ok":
                return False, reason
            self.enabled = True
            return True, "enabled; local ROS1 receiver arm and fresh filtered commands still required"

    def take(self):
        with self.lock:
            result, self.pending = self.pending, {}
            return result

    def _received(self, kind, packet):
        now = time.monotonic()
        if not isinstance(packet, dict) or type(packet.get("protocol")) is not int or packet.get("protocol") != PROTOCOL:
            raise ValueError("protocol_mismatch")
        with self.lock:
            if kind == "state":
                expected = {"protocol", "stamp", "armed", "lease", "reason", "driver_output_enabled", "output_topic",
                            "base_frame", "max_linear_mps", "max_angular_rps", "command_timeout_sec"}
                if set(packet) != expected or not isinstance(packet.get("reason"), str):
                    raise ValueError("invalid_receiver_state_schema")
                for key in ("armed", "driver_output_enabled"):
                    if type(packet.get(key)) is not bool:
                        raise ValueError("invalid_receiver_state")
                frame_id(packet["base_frame"])
                stamp_ns(packet["stamp"])
                if (not isinstance(packet.get("lease"), str) or len(packet["lease"]) != 32 or
                        not isinstance(packet.get("output_topic"), str) or not packet["output_topic"].startswith("/") or
                        any(not finite(packet.get(k)) or packet[k] <= 0 for k in ("max_linear_mps", "max_angular_rps", "command_timeout_sec"))):
                    raise ValueError("invalid_receiver_limits_or_lease")
                if self.receiver and self.receiver[0]["armed"] and (not packet["armed"] or packet["lease"] != self.receiver[0]["lease"]):
                    self.enabled = False
                self.receiver = (packet, now)
            elif packet.get("kind") != kind or not isinstance(packet.get("msg"), dict):
                raise ValueError("invalid_telemetry_envelope")
            if kind == "image":
                source = stamp_ns(packet["msg"]["header"]["stamp"])
                self.image_source_age_ms = (self.ros_clock()-source)/1e6
                if self.image_received_at is not None:
                    self.image_last_gap_ms = (now-self.image_received_at)*1000
                    self.image_max_gap_ms = max(self.image_max_gap_ms, self.image_last_gap_ms)
                self.image_received_at = now
                self.image_count += 1
            self.pending[kind] = packet

    def _send_command(self, socket):
        with self.lock:
            if self._receiver_reason(time.monotonic(), self.ros_clock()) != "ok":
                self.enabled = False
                return
            state = self.receiver[0]
            if not self.forward_commands or not state["armed"]:
                return
            stopping, stop_stamp = self.stop_stamp is not None, self.stop_stamp
            self.stop_stamp = None
            if not self.enabled and not stopping:
                return
            command = {"source_ns": stop_stamp, "linear_x": 0.0, "angular_z": 0.0} if stopping else self.provider(state)
            if command is None:
                return
            self.sequence += 1
            packet = {"protocol": PROTOCOL, "lease": state["lease"], "sequence": self.sequence,
                      "stamp": stamp_dict(command["source_ns"]), "base_frame": self.base_frame,
                      "linear_x": command["linear_x"], "angular_z": command["angular_z"]}
        socket.send(json.dumps({"op": "publish", "topic": self.prefix+"/command", "msg": {"data": dumps(packet)}}))

    def _run(self):
        import websocket
        while not self.stop.is_set():
            socket = None
            try:
                socket = websocket.create_connection(self.url, timeout=1.0, enable_multithread=True,
                                                     http_no_proxy=[urlparse(self.url).hostname])
                socket.settimeout(0.02)
                for kind in self.KINDS:
                    topic = self.prefix+("/state" if kind == "state" else "/"+kind+"_json")
                    socket.send(json.dumps({"op": "subscribe", "id": kind, "topic": topic, "type": "std_msgs/String",
                                            "queue_length": 1, "throttle_rate": 100 if kind == "image" else 0}))
                if self.forward_commands:
                    socket.send(json.dumps({"op": "advertise", "topic": self.prefix+"/command", "type": "std_msgs/String"}))
                with self.lock:
                    self.connected, self.enabled, self.error = True, False, ""
                    self.receiver, self.pending = None, {}
                sent_at = float("-inf")
                while not self.stop.is_set():
                    try:
                        raw = socket.recv()
                        if not raw:
                            raise OSError("socket_closed")
                        message = loads(raw)
                        if message.get("op") == "publish":
                            topic = message.get("topic")
                            for kind in self.KINDS:
                                expected = self.prefix+("/state" if kind == "state" else "/"+kind+"_json")
                                if topic == expected:
                                    self._received(kind, loads(message["msg"]["data"]))
                                    break
                        elif message.get("op") == "status" and message.get("level") == "error":
                            raise ValueError("rosbridge_subscription_error")
                    except websocket.WebSocketTimeoutException:
                        pass
                    if time.monotonic()-sent_at >= 0.05:
                        self._send_command(socket)
                        sent_at = time.monotonic()
            except Exception as exc:
                with self.lock:
                    self.error = type(exc).__name__+": "+str(exc)[:160]
            finally:
                with self.lock:
                    self.connected = self.enabled = False
                    self.receiver, self.pending, self.stop_stamp = None, {}, None
                if socket is not None:
                    socket.close()
            self.stop.wait(1.0)
