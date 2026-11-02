import math
import unittest
import numpy as np
from husky_drivable_area.vision_local import local_goal_target
from husky_drivable_area.vision_goal import plan_vision_reference
from husky_link_core.supervised_trial import selected_limits,TrialBudget,LONG_GOAL_LIMITS
from tools.demo_junction_turn import run_case


class JunctionTests(unittest.TestCase):
    def test_side_goal_is_explicit_and_backward_goal_is_rejected(self):
        for sign in (-1,1):
            with self.assertRaises(ValueError):local_goal_target([0,sign*3])
            np.testing.assert_allclose(local_goal_target([0,sign*3],allow_side_goal=True),[0,sign*2.5])
        with self.assertRaises(ValueError):local_goal_target([-3,0],allow_side_goal=True)

    def test_left_and_right_junction_closed_loop_reaches_clicked_goal(self):
        for sign in (-1,1):
            report,_=run_case(sign)
            self.assertLess(report['error_m'],.15)
            self.assertGreater(sign*report['final_heading_deg'],60)

    def test_unknown_near_field_assumption_does_not_erase_known_veto(self):
        def sample(points):
            return np.where(points[:,0]<.8,0,1).astype(np.uint8)
        path,_,detail=plan_vision_reference(sample,.8,[0,2.5],goal_priority=True)
        # No turning arc may cross the known near veto despite supervision.
        self.assertNotEqual(detail.get('reference_kind'),'junction_arc')

    def test_turn_budget_is_explicit_preserves_old_preset_and_still_has_a_limit(self):
        limits=selected_limits(True,True,900.,turning=True)
        self.assertEqual(LONG_GOAL_LIMITS['max_rotation_rad'],.35)
        with self.assertRaises(ValueError):selected_limits(True,True,300.,turning=True)
        b=TrialBudget(limits);b.observe((0,0,0,10_000_000_000),1.);b.reset(1.)
        for i in range(1,10):
            b.observe((0,0,i*math.pi/4,10_000_000_000+i*100_000_000),1.+i*.1)
            reason=b.check(1.+i*.1,10_000_000_000+i*100_000_000)
            if i==2:self.assertEqual(reason,'clear')
        self.assertEqual(reason,'trial_rotation_limit')
