"""Goal replacement must not display or execute a late superseded request."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest import mock


class GoalDispatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stubs = {
            "rclpy": NS(), "rclpy.node": NS(Node=object),
            "rclpy.action": NS(ActionClient=object),
            "rclpy.qos": NS(QoSProfile=object, DurabilityPolicy=object, ReliabilityPolicy=object),
            "geometry_msgs": NS(), "geometry_msgs.msg": NS(PoseStamped=object),
            "nav_msgs": NS(), "nav_msgs.msg": NS(Path=NS),
            "nav2_msgs": NS(), "nav2_msgs.action": NS(ComputePathToPose=object, NavigateToPose=object),
            "std_msgs": NS(), "std_msgs.msg": NS(String=NS),
        }
        source = Path(__file__).resolve().parents[1]/"husky_drivable_area/goal_dispatch.py"
        spec = importlib.util.spec_from_file_location("_goal_dispatch_test", source)
        cls.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, stubs):
            spec.loader.exec_module(cls.module)

    def setUp(self):
        self.node = object.__new__(self.module.GoalDispatch)
        self.node.allow_motion = False
        self.node.generation = 2
        self.node.goal_handle = None
        self.node.status = mock.Mock()
        self.node.path_pub = mock.Mock()
        self.node.goal_pub = mock.Mock()
        self.node.client = mock.Mock()

    def test_superseded_late_acceptance_is_cancelled_and_never_observed_as_active(self):
        handle = mock.Mock(accepted=True)
        self.node.accepted(NS(result=lambda: handle), 1)
        handle.cancel_goal_async.assert_called_once()
        handle.get_result_async.assert_not_called()
        self.assertIsNone(self.node.goal_handle)

    def test_superseded_result_cannot_replace_current_preview(self):
        future = mock.Mock()
        self.node.finished(future, 1)
        future.result.assert_not_called()
        self.node.path_pub.publish.assert_not_called()

    def test_replacement_cancels_old_goal_and_clears_old_path_even_if_server_unavailable(self):
        old = mock.Mock()
        self.node.goal_handle = old
        self.node.client.server_is_ready.return_value = False
        message = NS(header=NS(frame_id="map"), pose=NS(position=NS(x=1.,y=0.,z=0.),
            orientation=NS(x=0.,y=0.,z=0.,w=1.)))
        self.node.goal(message)
        old.cancel_goal_async.assert_called_once()
        self.assertEqual(self.node.generation, 3)
        self.node.path_pub.publish.assert_called_once()
        self.node.client.send_goal_async.assert_not_called()

    def test_action_transport_failure_is_reported_without_reusing_a_handle(self):
        future = mock.Mock()
        future.result.side_effect = RuntimeError("server lost")
        self.node.accepted(future, 2)
        self.assertIn("goal_transport_error", self.node.status.publish.call_args.args[0].data)
        self.assertIsNone(self.node.goal_handle)

    def test_planning_failure_retains_nav2_code_name_and_diagnostic_message(self):
        class FailedResult:
            TIMEOUT=207
            error_code=207
            error_msg="Costmap timed out waiting for update"
        self.node.finished(NS(result=lambda:NS(status=6,result=FailedResult())),2)
        status=json.loads(self.node.status.publish.call_args.args[0].data)
        self.assertEqual(status["error_code"],207)
        self.assertEqual(status["error_name"],"TIMEOUT")
        self.assertIn("Costmap timed out",status["error_msg"])
        self.node.path_pub.publish.assert_not_called()
