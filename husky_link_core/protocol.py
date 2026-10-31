"""Python 2/3-compatible protocol. No ROS, network or hardware dependencies."""

import json
import math
import uuid

PROTOCOL = 1
MAX_PACKET_BYTES = 2 * 1024 * 1024
try:
    STRING_TYPES = (basestring,)
    INTEGER_TYPES = (int, long)
except NameError:
    STRING_TYPES, INTEGER_TYPES = (str,), (int,)


def finite(value):
    return type(value) in INTEGER_TYPES + (float,) and not math.isnan(value) and not math.isinf(value)


def wire(value):
    """Preserve +inf/-inf/NaN explicitly inside a standard ROS String payload."""
    if isinstance(value, float) and not finite(value):
        return {"__float__": "nan" if math.isnan(value) else "+inf" if value > 0 else "-inf"}
    if value is None or isinstance(value, STRING_TYPES + INTEGER_TYPES + (float, bool)):
        return value
    if isinstance(value, dict):
        return dict((key, wire(item)) for key, item in value.items())
    if hasattr(value, "__slots__"):
        return dict((key, wire(getattr(value, key))) for key in value.__slots__)
    if hasattr(value, "__iter__"):
        return [wire(item) for item in value]
    raise ValueError("unsupported wire value")


def unwire(value):
    if isinstance(value, dict):
        if set(value) == {"__float__"}:
            if value["__float__"] not in ("nan", "+inf", "-inf"):
                raise ValueError("invalid special float")
            return float(value["__float__"])
        return dict((key, unwire(item)) for key, item in value.items())
    if isinstance(value, list):
        return [unwire(item) for item in value]
    return value


def dumps(value):
    result = json.dumps(wire(value), separators=(",", ":"), allow_nan=False)
    if len(result.encode("utf-8")) > MAX_PACKET_BYTES:
        raise ValueError("packet_too_large")
    return result


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("duplicate_json_key")
        result[key] = value
    return result


def loads(value):
    if not isinstance(value, STRING_TYPES) or len(value.encode("utf-8")) > MAX_PACKET_BYTES:
        raise ValueError("invalid_or_oversized_packet")
    def invalid_constant(_):
        raise ValueError("nonfinite_json_constant")
    return unwire(json.loads(value, parse_constant=invalid_constant, object_pairs_hook=_pairs))


def stamp_ns(value):
    if not isinstance(value, dict) or set(value) != {"secs", "nsecs"}:
        raise ValueError("invalid_timestamp")
    secs, nsecs = value["secs"], value["nsecs"]
    if type(secs) not in INTEGER_TYPES or type(nsecs) not in INTEGER_TYPES or secs < 0 or not 0 <= nsecs < 1000000000:
        raise ValueError("invalid_timestamp")
    return secs * 1000000000 + nsecs


def stamp_dict(value_ns):
    secs, nsecs = divmod(int(value_ns), 1000000000)
    return {"secs": secs, "nsecs": nsecs}


def frame_id(value):
    if not isinstance(value, STRING_TYPES):
        raise ValueError("invalid_frame")
    value = value.lstrip("/")
    if not value or any(c.isspace() for c in value):
        raise ValueError("invalid_frame")
    return value


def joy_permitted(buttons, button, source_ns, received, ros_ns, now, timeout=0.3):
    """A held, in-range button with both fresh source and local receipt time."""
    return (type(button) in INTEGER_TYPES and 0 <= button < len(buttons) and buttons[button] == 1 and
            source_ns > 0 and -0.05 <= (ros_ns-source_ns)/1e9 <= timeout and 0 <= now-received <= timeout)


class CommandWatchdog(object):
    """A local lease, timestamp/sequence checks and monotonic deadline.

    Loss/rejection disarms and rotates the lease. Reconnect cannot resume old
    commands. None means release the mux input; a short zero hold precedes it.
    """
    def __init__(self, base_frame, max_linear=0.10, max_angular=0.30, timeout=0.25, zero_hold=0.5):
        if any(not finite(v) or v <= 0 for v in (max_linear, max_angular, timeout, zero_hold)):
            raise ValueError("invalid_watchdog_limits")
        self.base_frame = frame_id(base_frame)
        self.max_linear, self.max_angular, self.timeout, self.zero_hold = max_linear, max_angular, timeout, zero_hold
        self.armed = False
        self.lease = uuid.uuid4().hex
        self.reason = "disabled"
        self.sequence = -1
        self.command = (0.0, 0.0)
        self.received = self.armed_at = float("-inf")
        self.source_ns = 0
        self.zero_until = float("-inf")
        self.clock_ns = None
        self.clock_advanced_at = float("-inf")

    def set_enabled(self, enabled, now):
        if type(enabled) is not bool:
            raise ValueError("enabled must be bool")
        self.lease = uuid.uuid4().hex
        self.armed = enabled
        self.reason = "waiting_for_command" if enabled else "disabled"
        self.sequence, self.source_ns = -1, 0
        self.command = (0.0, 0.0)
        self.received = float("-inf")
        self.armed_at = now
        self.zero_until = now + self.zero_hold

    def disarm(self, reason, now):
        if self.armed:
            self.set_enabled(False, now)
        self.reason = reason

    def accept(self, packet, ros_ns, now):
        if not self.armed:
            return False
        try:
            required = {"protocol", "lease", "sequence", "stamp", "base_frame", "linear_x", "angular_z"}
            if not isinstance(packet, dict) or set(packet) != required or type(packet["protocol"]) not in INTEGER_TYPES or packet["protocol"] != PROTOCOL:
                raise ValueError("invalid_command_schema")
            if packet["lease"] != self.lease:
                raise ValueError("wrong_or_expired_lease")
            seq = packet["sequence"]
            if type(seq) not in INTEGER_TYPES or not self.sequence < seq < 2**63:
                raise ValueError("replayed_command")
            source = stamp_ns(packet["stamp"])
            age = (ros_ns-source)/1e9
            if source <= 0 or not -0.05 <= age <= self.timeout or source < self.source_ns:
                raise ValueError("stale_future_or_reversed_command")
            if frame_id(packet["base_frame"]) != self.base_frame:
                raise ValueError("command_frame_mismatch")
            v, w = packet["linear_x"], packet["angular_z"]
            if not finite(v) or not finite(w) or not 0 <= v <= self.max_linear or abs(w) > self.max_angular:
                raise ValueError("command_out_of_bounds")
            if source == self.source_ns:
                if (v, w) != self.command or now-self.received > self.timeout:
                    raise ValueError("reused_or_expired_command_stamp")
            else:
                self.received = now
            self.sequence, self.source_ns = seq, source
            self.command, self.reason = (v, w), "tracking"
            return True
        except (ValueError, TypeError, OverflowError) as exc:
            self.disarm(str(exc), now)
            return False

    def tick(self, ros_ns, now):
        if self.clock_ns is None or ros_ns > self.clock_ns:
            self.clock_advanced_at = now
        elif ros_ns < self.clock_ns and self.armed:
            self.disarm("ros_clock_reversed", now)
        self.clock_ns = ros_ns
        if self.armed and now-self.clock_advanced_at > self.timeout:
            self.disarm("ros_clock_stalled", now)
        if self.armed:
            if not self.source_ns:
                if now-self.armed_at > 1.0:
                    self.disarm("no_first_command", now)
            elif (not -0.05 <= (ros_ns-self.source_ns)/1e9 <= self.timeout or now-self.received > self.timeout):
                self.disarm("command_timeout", now)
        if self.armed and self.source_ns:
            return self.command
        return (0.0, 0.0) if self.armed or now <= self.zero_until else None
