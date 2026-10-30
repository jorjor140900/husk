"""Finite operator sessions must never auto-rearm after a fault or stale state."""
import copy
import fnmatch
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from husky_link_core.operator_permission import OperatorSession, operator_settings, PERMISSION_TOPIC, PREVIEW_TOPIC
from husky_link_core.protocol import CommandWatchdog, PROTOCOL, stamp_dict


class OperatorPermissionTests(unittest.TestCase):
    def setUp(self):
        self.session = OperatorSession(duration=1)
        self.now = 100.
        self.ros = 1_000_000_000_000
        self.state = dict(protocol=PROTOCOL, stamp=stamp_dict(self.ros), armed=False,
                          output_topic=PREVIEW_TOPIC, driver_output_enabled=False)
        self.receive()

    def receive(self, delta=0, armed=False):
        self.now += delta
        self.ros += int(delta * 1e9)
        self.state.update(stamp=stamp_dict(self.ros), armed=armed)
        self.session.receive_state(copy.deepcopy(self.state), self.now)

    def start_and_arm(self):
        self.assertTrue(self.session.start(self.now, self.ros))
        self.assertEqual(self.session.tick(self.now, self.ros), (True, False))
        self.receive(.16)
        self.assertEqual(self.session.tick(self.now, self.ros), (True, True))
        self.session.arm_result(True)
        self.receive(.05, armed=True)
        self.assertEqual(self.session.tick(self.now, self.ros), (True, False))
        self.assertEqual(self.session.phase, "active")

    def test_startup_false_and_repeated_start_never_extends_session(self):
        self.assertEqual(self.session.tick(self.now, self.ros), (False, False))
        self.start_and_arm()
        deadline = self.session.deadline
        self.assertFalse(self.session.start(self.now, self.ros))
        self.assertEqual(self.session.deadline, deadline)
        self.receive(.2, armed=True)
        self.assertEqual(self.session.tick(self.now, self.ros), (True, False))
        self.session.deadline = self.now
        self.assertEqual(self.session.tick(self.now, self.ros), (False, False))
        self.assertEqual(self.session.reason, "session_expired")

    def test_link_state_loss_requires_new_explicit_start(self):
        self.start_and_arm()
        self.assertEqual(self.session.tick(self.now+.31, self.ros+310_000_000), (False, False))
        self.assertEqual(self.session.reason, "gateway_state_stale")
        self.receive(.4, armed=False)
        self.assertEqual(self.session.tick(self.now, self.ros), (False, False))
        self.assertTrue(self.session.start(self.now, self.ros))

    def test_gateway_disarm_never_automatically_rearms(self):
        self.start_and_arm()
        self.receive(.05, armed=False)
        self.assertEqual(self.session.tick(self.now, self.ros), (False, False))
        self.assertEqual(self.session.reason, "gateway_disarmed")
        self.receive(.05, armed=True)
        self.assertEqual(self.session.tick(self.now, self.ros), (False, False))

    def test_arm_rejection_and_unobserved_arm_fail_closed(self):
        for reject in (True, False):
            with self.subTest(reject=reject):
                self.setUp()
                self.assertTrue(self.session.start(self.now, self.ros))
                self.receive(.16)
                self.assertEqual(self.session.tick(self.now, self.ros), (True, True))
                self.session.arm_result(not reject)
                self.receive(.2)
                self.session.tick(self.now, self.ros)
                self.receive(.2)
                self.assertEqual(self.session.tick(self.now, self.ros), (False, False))
                self.assertEqual(self.session.reason, "gateway_arm_rejected" if reject else "gateway_arm_not_observed")

    def test_bad_wrong_output_and_replayed_state_prevent_start(self):
        variants = [None, {}, dict(self.state, armed="true"), dict(self.state, output_topic="/cmd_vel"),
                    dict(self.state, driver_output_enabled=True), dict(self.state, stamp=stamp_dict(self.ros-500_000_000))]
        for state in variants:
            with self.subTest(state=state):
                self.session.receive_state(state, self.now)
                self.assertFalse(self.session.start(self.now, self.ros))
                self.assertEqual(self.session.tick(self.now, self.ros), (False, False))

    def test_operator_stop_and_clock_reversal_drop_permission(self):
        self.start_and_arm()
        self.assertEqual(self.session.tick(self.now-.01, self.ros), (False, False))
        self.assertEqual(self.session.reason, "monotonic_clock_reversed")
        self.receive(.05)
        self.assertTrue(self.session.start(self.now, self.ros))
        self.session.stop()
        self.assertEqual(self.session.tick(self.now, self.ros), (False, False))

    def test_settings_require_bool_source_and_preserve_physical_gate(self):
        profile = dict(robot=dict(joy_topic=None, joy_deadman_button=None, ros1_permission_topic=PERMISSION_TOPIC),
                       bridge=dict(forward_commands=False, expected_output_topic="/cmd_vel"), field_measurements={})
        self.assertEqual(operator_settings(profile), (PERMISSION_TOPIC, PREVIEW_TOPIC))
        with self.assertRaisesRegex(ValueError, "pending"):
            operator_settings(profile, True)
        profile["field_measurements"] = dict(geometry_checked=True, mux_and_base_timeout_checked=True,
                                             independent_permission_gate_checked=True)
        profile["bridge"]["forward_commands"] = True
        self.assertEqual(operator_settings(profile, True), (PERMISSION_TOPIC, "/cmd_vel"))
        profile["robot"]["joy_topic"] = "/joy"
        with self.assertRaisesRegex(ValueError, "null"):
            operator_settings(profile, True)

    def test_extended_session_requires_explicit_maximum_and_expires_without_renewal(self):
        with self.assertRaises(ValueError):OperatorSession(900.)
        with self.assertRaises(ValueError):OperatorSession(900.,max_duration_sec=300.)
        self.session=OperatorSession(900.,max_duration_sec=900.)
        self.receive();self.start_and_arm();deadline=self.session.deadline
        self.receive(300.,armed=True)
        self.assertFalse(self.session.start(self.now,self.ros))
        self.assertEqual(self.session.deadline,deadline)
        self.receive(600.,armed=True)
        self.assertEqual(self.session.tick(self.now,self.ros),(False,False))
        self.assertEqual(self.session.reason,'session_expired')

    def test_duration_bounds(self):
        for value in (0, -1, 121, float("nan"), float("inf"), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                OperatorSession(value)

    def test_console_permission_is_outside_remote_command_topic_allowlist(self):
        launch = ET.parse(Path(__file__).parents[1]/"ros1/husky_vision_bridge/launch/bridge.launch")
        allowed = launch.find("include/arg[@name='topics_glob']").attrib["value"].strip("[]").split(",")
        self.assertFalse(any(fnmatch.fnmatch(PERMISSION_TOPIC, pattern) for pattern in allowed))

    def test_command_watchdog_still_expires_even_if_console_process_stays_alive(self):
        self.start_and_arm()
        gate = CommandWatchdog("base_link")
        gate.set_enabled(True, self.now)
        packet = dict(protocol=PROTOCOL, lease=gate.lease, sequence=0, stamp=stamp_dict(self.ros),
                      base_frame="base_link", linear_x=.05, angular_z=0.)
        self.assertTrue(gate.accept(packet, self.ros, self.now))
        self.assertEqual(gate.tick(self.ros, self.now), (.05, 0.))
        self.receive(.26, armed=True)
        self.assertEqual(self.session.tick(self.now, self.ros), (True, False))
        self.assertEqual(gate.tick(self.ros, self.now), (0., 0.))
        self.assertFalse(gate.armed)
        self.receive(.05, armed=False)
        self.assertEqual(self.session.tick(self.now, self.ros), (False, False))
