"""Robot-local trial limits remain active without the ROS2 trial process."""
from types import SimpleNamespace as NS
import unittest
import test_operator_gateway
from test_gateway_media import Header
from husky_link_core.protocol import dumps, stamp_dict
from husky_link_core.supervised_trial import TrialBudget

class SupervisedGatewayTests(unittest.TestCase):
    def setUp(self):
        self.fixture=test_operator_gateway.OperatorGatewayTests();self.fixture.setUp()
        self.g,self.clock=self.fixture.g,self.fixture.clock
        self.g.trial=TrialBudget();self.g.odom_frame='odom'
        self.g.gate.max_linear=.03;self.g.gate.max_angular=.15
        self.sequence=0
        self.pose()
        self.fixture.start()

    def pose(self,x=0):
        h=Header(self.clock[1]);h.frame_id='odom'
        message=NS(header=h,child_frame_id='base_link',pose=NS(pose=NS(
            position=NS(x=float(x),y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
        self.g.telemetry(message,'odom')

    def command(self):
        self.sequence+=1
        self.g.permission(NS(data=True))
        self.g.command(NS(data=dumps(dict(protocol=1,lease=self.g.gate.lease,sequence=self.sequence,
            stamp=stamp_dict(self.clock[1]),base_frame='base_link',linear_x=.03,angular_z=0.))))
        self.g.tick(None)

    def test_robot_local_distance_limit_overrides_fresh_commands(self):
        self.command();self.assertTrue(self.g.gate.armed)
        self.fixture.advance(.05);self.pose(.51);self.command()
        self.assertFalse(self.g.gate.armed)
        self.assertEqual(self.g.gate.reason,'trial_distance_limit')
        self.assertEqual(self.g.output.messages[-1].linear.x,0.)

    def test_robot_local_time_limit_and_no_renewal_while_armed(self):
        self.command()
        started=self.g.trial.started
        self.assertFalse(self.g.set_enabled(NS(data=True)).success)
        self.assertEqual(self.g.trial.started,started)
        self.fixture.advance(20.1);self.pose();self.command()
        self.assertFalse(self.g.gate.armed)
        self.assertEqual(self.g.gate.reason,'trial_time_limit')
        self.assertEqual(self.g.output.messages[-1].linear.x,0.)
