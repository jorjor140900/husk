"""Protocol leases, local stop deadlines and preservation of scan unknowns."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from husky_link_core.protocol import CommandWatchdog, dumps, joy_permitted, loads, stamp_dict
from husky_drivable_area.robot_link import RosbridgeLink


def receiver(armed=False):
    return {"protocol": 1, "stamp": stamp_dict(10_000_000_000), "armed": armed, "lease": "a"*32,
            "reason": "disabled", "driver_output_enabled": False, "output_topic": "/husky_vision/watchdog_preview",
            "base_frame": "base_link", "max_linear_mps": 0.10, "max_angular_rps": 0.30, "command_timeout_sec": 0.25}


class WatchdogTests(unittest.TestCase):
    def setUp(self):
        self.gate = CommandWatchdog("base_link")
        self.gate.set_enabled(True, 2.0)
        self.packet = {"protocol": 1, "lease": self.gate.lease, "sequence": 1, "stamp": stamp_dict(10_000_000_000),
                       "base_frame": "base_link", "linear_x": 0.08, "angular_z": 0.1}

    def test_fresh_commands_then_local_deadline_zero_and_mux_release(self):
        self.assertTrue(self.gate.accept(self.packet, 10_000_000_000, 2.0))
        self.assertEqual(self.gate.tick(10_100_000_000, 2.1), (0.08, 0.1))
        old_lease = self.gate.lease
        self.assertEqual(self.gate.tick(10_300_000_000, 2.3), (0, 0))
        self.assertFalse(self.gate.armed)
        self.assertNotEqual(self.gate.lease, old_lease)
        self.assertIsNone(self.gate.tick(10_900_000_000, 2.9))
        self.assertFalse(self.gate.accept(self.packet, 10_900_000_000, 2.9))

    def test_monotonic_deadline_stops_even_if_ros_clock_freezes(self):
        self.assertTrue(self.gate.accept(self.packet, 10_000_000_000, 2.0))
        self.assertEqual(self.gate.tick(10_000_000_000, 2.3), (0, 0))
        self.assertFalse(self.gate.armed)

    def test_retransmitting_same_stamp_cannot_extend_monotonic_deadline(self):
        self.assertTrue(self.gate.accept(self.packet, 10_000_000_000, 2.0))
        self.gate.tick(10_000_000_000, 2.0)
        self.packet["sequence"] = 2
        self.assertTrue(self.gate.accept(self.packet, 10_000_000_000, 2.2))
        self.assertEqual(self.gate.received, 2.0)
        self.assertEqual(self.gate.tick(10_000_000_000, 2.3), (0, 0))
        self.assertFalse(self.gate.armed)

    def test_frozen_receiver_clock_disarms_even_with_changing_command_stamps(self):
        self.gate.tick(10_000_000_000, 2.0)
        for sequence, now in ((1, 2.0), (2, 2.1), (3, 2.2)):
            packet = {**self.packet, "sequence": sequence, "stamp": stamp_dict(10_000_000_000+sequence)}
            self.assertTrue(self.gate.accept(packet, 10_000_000_000, now))
            self.gate.tick(10_000_000_000, now)
        self.assertEqual(self.gate.tick(10_000_000_000, 2.3), (0, 0))
        self.assertEqual(self.gate.reason, "ros_clock_stalled")

    def test_rearm_changes_lease_and_old_queued_command_is_never_applied(self):
        old = self.packet.copy()
        self.gate.set_enabled(False, 2.1)
        self.gate.set_enabled(True, 2.2)
        self.assertFalse(self.gate.accept(old, 10_000_000_000, 2.2))
        self.assertFalse(self.gate.armed)

    def test_replay_stale_future_wrong_frame_nonfinite_and_limits_disarm(self):
        cases = [{"sequence": -1}, {"stamp": stamp_dict(9_000_000_000)}, {"stamp": stamp_dict(11_000_000_000)},
                 {"base_frame": "other"}, {"linear_x": float("nan")}, {"linear_x": -0.1},
                 {"linear_x": 0.11}, {"angular_z": 0.31}, {"linear_x": True}, {"protocol": True}]
        for change in cases:
            self.gate.set_enabled(True, 2.0)
            value = {**self.packet, "lease": self.gate.lease, **change}
            self.assertFalse(self.gate.accept(value, 10_000_000_000, 2.0), change)
            self.assertFalse(self.gate.armed)
            self.assertEqual(self.gate.tick(10_000_000_000, 2.1), (0, 0))

    def test_identical_sequence_is_rejected(self):
        self.assertTrue(self.gate.accept(self.packet, 10_000_000_000, 2.0))
        self.assertFalse(self.gate.accept(self.packet, 10_010_000_000, 2.01))
        self.assertEqual(self.gate.reason, "replayed_command")

    def test_no_first_command_disarms_and_does_not_latch_mux_forever(self):
        self.assertEqual(self.gate.tick(10_000_000_000, 2.5), (0, 0))
        self.assertEqual(self.gate.tick(11_000_000_000, 3.1), (0, 0))
        self.assertFalse(self.gate.armed)
        self.assertIsNone(self.gate.tick(12_000_000_000, 3.7))

    def test_deadman_requires_held_button_and_fresh_source_and_receipt(self):
        self.assertTrue(joy_permitted([0, 1], 1, 10_000_000_000, 2.0, 10_100_000_000, 2.1))
        for buttons, button, source, received in (([0, 0], 1, 10_000_000_000, 2.0),
                ([1], 2, 10_000_000_000, 2.0), ([1], True, 10_000_000_000, 2.0),
                ([1], 0, 9_000_000_000, 2.0), ([1], 0, 11_000_000_000, 2.0), ([1], 0, 10_000_000_000, 1.0)):
            self.assertFalse(joy_permitted(buttons, button, source, received, 10_100_000_000, 2.1))

    def test_special_scan_values_round_trip_without_becoming_free_space(self):
        value = loads(dumps({"ranges": [1.2, float("inf"), float("-inf"), float("nan")]}))["ranges"]
        self.assertEqual(value[:3], [1.2, float("inf"), float("-inf")])
        self.assertTrue(math.isnan(value[3]))
        self.assertNotIn("Infinity", dumps(value))
        for packet in ('{"x":1,"x":2}', '{"x":NaN}', '{"__float__":"other"}'):
            with self.assertRaises(ValueError):
                loads(packet)


class LinkEnableTests(unittest.TestCase):
    def link(self, **overrides):
        return RosbridgeLink("ws://127.0.0.1:9090", ros_clock=lambda: 10_000_000_000, **overrides)

    def test_observe_never_enables_command_forwarding(self):
        link = self.link()
        link.connected = True
        link._received("state", receiver())
        self.assertEqual(link.snapshot()["reason"], "ok")
        self.assertFalse(link.enable(True)[0])
        self.assertFalse(link.snapshot()["motion_permitted"])

    def test_camera_metrics_distinguish_source_age_from_receipt_gaps_without_restamping(self):
        link = self.link()
        packet = {"protocol": 1, "kind": "image", "msg": {"header": {"stamp": stamp_dict(9_900_000_000)}}}
        with mock.patch("husky_drivable_area.robot_link.time.monotonic", return_value=2.0):
            link._received("image", packet)
        with mock.patch("husky_drivable_area.robot_link.time.monotonic", return_value=2.5):
            link._received("image", packet)
            stats = link.snapshot()["camera_transport"]
        self.assertEqual(stats["received_frames"], 2)
        self.assertEqual(stats["source_age_at_receipt_ms"], 100)
        self.assertEqual(stats["last_gap_ms"], 500)
        self.assertEqual(stats["receipt_age_ms"], 0)
        self.assertEqual(link.take()["image"]["msg"]["header"]["stamp"], stamp_dict(9_900_000_000))

    def test_matching_receiver_and_manual_enable_are_both_required(self):
        link = self.link(forward_commands=True, base_frame="base_link", expected_output_topic="/husky_vision/watchdog_preview")
        link.connected = True
        link._received("state", receiver(True))
        self.assertFalse(link.snapshot()["motion_permitted"])
        self.assertTrue(link.enable(True)[0])
        self.assertTrue(link.snapshot()["motion_permitted"])
        link._received("state", receiver(False))
        self.assertFalse(link.snapshot()["enabled"])
        link._received("state", receiver(True))
        self.assertFalse(link.snapshot()["motion_permitted"])

    def test_late_state_latches_disable_reason_even_when_fresh_state_returns(self):
        clock=[10_000_000_000]
        link=self.link(forward_commands=True,base_frame="base_link",
                       expected_output_topic="/husky_vision/watchdog_preview")
        link.ros_clock=lambda:clock[0];link.connected=True
        with mock.patch("husky_drivable_area.robot_link.time.monotonic",return_value=2.):
            link._received("state",receiver(True))
            self.assertTrue(link.enable(True)[0])
        clock[0]+=350_000_000
        with mock.patch("husky_drivable_area.robot_link.time.monotonic",return_value=2.35):
            link._send_command(mock.Mock())
            timing=link.snapshot()["receiver_timing_ms"]
            self.assertAlmostEqual(timing["receipt_age_ms"],350)
            self.assertAlmostEqual(timing["source_age_ms"],350)
            self.assertAlmostEqual(timing["source_age_at_receipt_estimate_ms"],0)
            fresh=receiver(True);fresh["stamp"]=stamp_dict(clock[0])
            link._received("state",fresh)
            status=link.snapshot()
            self.assertEqual(status["reason"],"ok")
            self.assertFalse(status["enabled"])
            self.assertEqual(status["disable_reason"],"receiver_state_stale_or_clock_skew")
            self.assertTrue(link.enable(True)[0])
            self.assertEqual(link.snapshot()["disable_reason"],"")

    def test_clock_offset_is_distinct_from_receiver_delivery_gap(self):
        link=self.link();link.connected=True
        with mock.patch("husky_drivable_area.robot_link.time.monotonic",return_value=2.):
            state=receiver();state["stamp"]=stamp_dict(9_600_000_000)
            link._received("state",state)
            timing=link.snapshot()["receiver_timing_ms"]
            self.assertEqual(timing["receipt_age_ms"],0)
            self.assertEqual(timing["source_age_at_receipt_estimate_ms"],400)

    def test_unexpected_driver_topic_and_source_clock_skew_block_enable(self):
        link = self.link(forward_commands=True, base_frame="base_link", expected_output_topic="/verified/mux_input")
        link.connected = True
        link._received("state", receiver(True))
        self.assertFalse(link.enable(True)[0])
        state = receiver(True)
        state["output_topic"], state["driver_output_enabled"] = "/verified/mux_input", True
        state["stamp"] = stamp_dict(9_000_000_000)
        link._received("state", state)
        self.assertEqual(link.snapshot()["reason"], "receiver_state_stale_or_clock_skew")
        self.assertFalse(link.enable(True)[0])

    def test_reject_credentials_wildcards_or_incomplete_forwarding_config(self):
        for kwargs in ({"prefix": "/*"}, {"forward_commands": True}, {"prefix": "/"}):
            with self.assertRaises(ValueError):
                self.link(**kwargs)
        with self.assertRaises(ValueError):
            RosbridgeLink("ws://user:password@127.0.0.1:9090")


if __name__ == "__main__":
    unittest.main()
