"""Exercise the actual ROS1 gateway callbacks with message stubs, no robot.

This tests the local Bool gate plus command watchdog, not physical stop time.
"""
from types import SimpleNamespace as NS
import unittest

from husky_link_core.operator_permission import OperatorSession, PERMISSION_TOPIC
from husky_link_core.protocol import dumps, loads, stamp_dict
import test_gateway_media


class OperatorGatewayTests(unittest.TestCase):
    def setUp(self):
        self.fixture = test_gateway_media.GatewayMediaTests()
        self.fixture.setUp()
        self.g = self.fixture.gateway
        self.clock = self.fixture.clock
        self.fixture.module.SetBoolResponse = NS
        self.g.allow_output = True
        self.g.permission_topic = PERMISSION_TOPIC
        self.g.output_topic = "/cmd_vel"
        self.session = OperatorSession(expected_output_topic="/cmd_vel", driver_output_enabled=True)

    def advance(self, seconds):
        self.clock[0] += seconds
        self.clock[1] += int(seconds*1e9)

    def receive(self):
        self.g.tick(None)
        self.session.receive_state(loads(self.g.state.messages[-1].data), self.clock[0])

    def start(self):
        self.receive()
        self.assertTrue(self.session.start(*self.clock))
        permitted, arm = self.session.tick(*self.clock)
        self.g.permission(NS(data=permitted))
        self.assertFalse(arm)
        self.advance(.16)
        self.receive()
        permitted, arm = self.session.tick(*self.clock)
        self.g.permission(NS(data=permitted))
        self.assertTrue(arm)
        result = self.g.set_enabled(NS(data=True))
        self.assertTrue(result.success)
        self.session.arm_result(result.success)
        self.advance(.05)
        self.receive()
        self.assertEqual(self.session.tick(*self.clock), (True, False))
        self.assertEqual(self.session.phase, "active")

    def command(self):
        self.g.command(NS(data=dumps(dict(protocol=1, lease=self.g.gate.lease, sequence=1,
            stamp=stamp_dict(self.clock[1]), base_frame="base_link", linear_x=.05, angular_z=.1))))
        self.g.tick(None)
        return self.g.output.messages[-1]

    def test_no_joy_required_but_local_bool_permission_required_to_arm(self):
        self.assertIsNone(self.g.joy_topic)
        self.assertFalse(self.g.set_enabled(NS(data=True)).success)
        self.start()
        command = self.command()
        self.assertEqual((command.linear.x, command.angular.z), (.05, .1))

    def test_console_loss_stops_even_when_command_is_fresh(self):
        self.start()
        self.advance(.31)
        self.command()
        self.assertFalse(self.g.gate.armed)
        self.assertEqual(self.g.gate.reason, "local_permission_lost")
        self.assertEqual(self.g.output.messages[-1].linear.x, 0)
        self.receive()
        self.assertEqual(self.session.tick(*self.clock), (False, False))

    def test_space_stop_disarms_and_reconnect_does_not_restore_old_session(self):
        self.start()
        self.command()
        lease = self.g.gate.lease
        self.session.stop()
        self.g.permission(NS(data=False))
        self.receive()
        self.assertFalse(self.g.gate.armed)
        self.assertNotEqual(self.g.gate.lease, lease)
        self.advance(.05)
        self.g.permission(NS(data=True))
        self.receive()
        self.assertEqual(self.session.tick(*self.clock), (False, False))
        self.assertFalse(self.g.gate.armed)
