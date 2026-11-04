"""Robot-local, bounded operator permission; no ROS or velocity publishers.

Compatible with the ROS1 Python runtime. A key grants one finite session, not
an unlimited true heartbeat. A lost/disarmed gateway requires a new start.
"""
from .protocol import INTEGER_TYPES, PROTOCOL, STRING_TYPES, finite, stamp_ns


PERMISSION_TOPIC = "/husky_operator/permission"
PREVIEW_TOPIC = "/husky_vision/watchdog_preview"
FIELD_CHECKS = {"geometry_checked", "mux_and_base_timeout_checked", "independent_permission_gate_checked"}


def operator_settings(profile, allow_output=False, supervised_trial=False):
    from .supervised_trial import trial_mode
    trial = trial_mode(profile, supervised_trial)
    if type(allow_output) is not bool:
        raise ValueError("allow_robot_output must be bool")
    robot, bridge = profile.get("robot", {}), profile.get("bridge", {})
    if robot.get("joy_topic") or robot.get("joy_deadman_button") is not None:
        raise ValueError("operator console requires joy_topic and joy_deadman_button=null")
    if robot.get("ros1_permission_topic") != PERMISSION_TOPIC:
        raise ValueError("configure robot.ros1_permission_topic=" + PERMISSION_TOPIC)
    if allow_output:
        checks = profile.get("field_measurements", {})
        if not trial and (set(checks) != FIELD_CHECKS or any(checks[key] is not True for key in FIELD_CHECKS)):
            raise ValueError("field geometry, mux/base timeout and independent permission checks are pending")
        topic = bridge.get("expected_output_topic")
        if (bridge.get("forward_commands") is not True or not isinstance(topic, STRING_TYPES) or
                not topic.startswith("/") or topic.startswith("/husky_vision/") or
                any(c.isspace() for c in topic)):
            raise ValueError("motion profile requires explicitly enabled bridge and verified mux input")
        return PERMISSION_TOPIC, topic
    return PERMISSION_TOPIC, PREVIEW_TOPIC


class OperatorSession(object):
    def __init__(self, duration=30.0, expected_output_topic=PREVIEW_TOPIC, driver_output_enabled=False, max_duration_sec=120.):
        if type(max_duration_sec) not in (int,float) or max_duration_sec not in (20.,120.,300.,900.):
            raise ValueError("invalid fixed session maximum")
        if not finite(duration) or not 1 <= duration <= max_duration_sec:
            raise ValueError("session duration must be 1..%s seconds"%max_duration_sec)
        if type(driver_output_enabled) is not bool:
            raise ValueError("driver_output_enabled must be bool")
        self.duration = float(duration)
        self.expected_output_topic = expected_output_topic
        self.driver_output_enabled = driver_output_enabled
        self.state = None
        self.phase, self.reason = "stopped", "startup"
        self.deadline = self.prime_until = self.arm_until = 0.0
        self.last_now = None

    def receive_state(self, value, received):
        self.state = (value, received)

    def _state_reason(self, now, ros_ns, snapshot):
        try:
            state, received = snapshot
            source = stamp_ns(state["stamp"])
            if (type(state["protocol"]) not in INTEGER_TYPES or state["protocol"] != PROTOCOL or type(state["armed"]) is not bool or
                    type(state["driver_output_enabled"]) is not bool):
                return "invalid_gateway_state"
            if source <= 0 or not -0.05 <= (ros_ns-source)/1e9 <= .3 or not 0 <= now-received <= .3:
                return "gateway_state_stale"
            if (state["output_topic"] != self.expected_output_topic or
                    state["driver_output_enabled"] != self.driver_output_enabled):
                return "gateway_output_mismatch"
        except (TypeError, KeyError, ValueError, OverflowError):
            return "invalid_or_missing_gateway_state"
        return "ok"

    def start(self, now, ros_ns):
        if self.phase != "stopped":
            return False  # Repeated g must not renew or extend an active lease.
        snapshot = self.state
        reason = self._state_reason(now, ros_ns, snapshot)
        if reason != "ok":
            self.reason = reason
            return False
        if snapshot[0]["armed"]:
            self.reason = "gateway_already_armed_stop_first"
            return False
        self.phase, self.reason = "priming", "priming_permission"
        self.deadline, self.prime_until = now+self.duration, now+.15
        self.last_now = now
        return True

    def stop(self, reason="operator_stop"):
        self.phase, self.reason = "stopped", reason

    def arm_result(self, success):
        if success is not True:
            self.stop("gateway_arm_rejected")

    def tick(self, now, ros_ns):
        """Return (permission, request_arm_once). Caller sends no velocity."""
        if self.phase == "stopped":
            return False, False
        if self.last_now is not None and now < self.last_now:
            self.stop("monotonic_clock_reversed")
        self.last_now = now
        if now >= self.deadline:
            self.stop("session_expired")
        if self.phase == "stopped":
            return False, False
        snapshot = self.state
        reason = self._state_reason(now, ros_ns, snapshot)
        if reason != "ok":
            self.stop(reason)
            return False, False
        armed = snapshot[0]["armed"]
        if self.phase == "priming":
            if armed:
                self.stop("unexpected_gateway_arm")
                return False, False
            if now >= self.prime_until:
                self.phase, self.reason = "waiting_arm", "waiting_gateway_arm"
                self.arm_until = now+.3
                return True, True
        elif self.phase == "waiting_arm":
            if armed:
                self.phase, self.reason = "active", "operator_session"
            elif now > self.arm_until:
                self.stop("gateway_arm_not_observed")
        elif not armed:
            self.stop("gateway_disarmed")
        return self.phase != "stopped", False
