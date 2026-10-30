"""Attended trial cannot silently enable normal profiles or extend budgets."""
import math
import unittest
from husky_link_core.supervised_trial import TrialBudget, LIMITS, MODE, trial_mode, bounded_command

class TrialBudgetTests(unittest.TestCase):
    def budget(self):
        budget=TrialBudget();budget.observe((0,0,0,10_000_000_000),1.);budget.reset(1.)
        return budget

    def test_profile_and_explicit_flag_both_required_and_limits_fixed(self):
        profile=dict(operation_mode=MODE,trial_limits=LIMITS.copy())
        self.assertTrue(trial_mode(profile,True))
        for p,flag in ((profile,False),({},True),(dict(profile,trial_limits=dict(LIMITS,max_linear_mps=1.)),True)):
            with self.assertRaises(ValueError):trial_mode(p,flag)
        self.assertFalse(trial_mode({}))

    def test_goal_session_is_explicit_and_old_profile_limits_stay_unchanged(self):
        from husky_link_core.supervised_trial import GOAL_LIMITS, profile_trial_limits
        profile=dict(operation_mode=MODE,trial_limits=GOAL_LIMITS.copy())
        self.assertTrue(trial_mode(profile,True))
        self.assertEqual(profile_trial_limits(profile)['max_linear_mps'],.05)
        self.assertEqual(LIMITS['max_linear_mps'],.03)
        self.assertEqual(LIMITS['max_duration_sec'],20.)
        with self.assertRaises(ValueError):trial_mode(profile,False)
        for key,value in (('max_linear_mps',.8),('max_distance_m',100.),('max_duration_sec',float('inf'))):
            bad=dict(profile,trial_limits=dict(GOAL_LIMITS,**{key:value}))
            with self.assertRaises(ValueError):trial_mode(bad,True)

    def test_goal_command_and_budget_pass_old_limits_but_keep_backup_limits(self):
        from husky_link_core.supervised_trial import GOAL_LIMITS
        self.assertEqual(bounded_command(.05,0.,GOAL_LIMITS),(.05,0.))
        budget=TrialBudget(GOAL_LIMITS);budget.observe((0,0,0,10_000_000_000),1.);budget.reset(1.)
        budget.observe((.8,0,0,31_000_000_000),22.)
        self.assertEqual(budget.check(22.,31_000_000_000),'clear')
        budget.observe((3.01,0,0,32_000_000_000),23.)
        self.assertEqual(budget.check(23.,32_000_000_000),'trial_distance_limit')
        budget=TrialBudget(GOAL_LIMITS);budget.observe((0,0,0,10_000_000_000),1.);budget.reset(1.)
        budget.observe((0,0,0,130_000_000_000),121.)
        self.assertEqual(budget.check(121.,130_000_000_000),'trial_time_limit')

    def test_command_scaling_preserves_curvature_and_rejects_invalid(self):
        v,w=bounded_command(.05,.2)
        self.assertAlmostEqual(v,.03);self.assertAlmostEqual(w/v,4)
        self.assertLessEqual(abs(w),.15)
        for values in ((-.01,0),(1,0),(0,float('nan'))):
            with self.assertRaises(ValueError):bounded_command(*values)

    def test_distance_uses_traveled_arc_and_does_not_cancel_on_return(self):
        b=self.budget();b.observe((.3,0,0,10_100_000_000),1.1)
        b.observe((0,0,0,10_200_000_000),1.2)
        self.assertEqual(b.check(1.2,10_200_000_000),'trial_distance_limit')
        b.observe((0,0,0,10_300_000_000),1.3)
        self.assertEqual(b.check(1.3,10_300_000_000),'trial_distance_limit')

    def test_rotation_accumulates_and_time_limits_work_with_fresh_pose(self):
        b=self.budget();b.observe((0,0,.2,10_100_000_000),1.1)
        b.observe((0,0,0,10_200_000_000),1.2)
        self.assertEqual(b.check(1.2,10_200_000_000),'trial_rotation_limit')
        b=self.budget();b.observe((0,0,0,30_000_000_000),21.)
        self.assertEqual(b.check(21.,30_000_000_000),'trial_time_limit')

    def test_duplicate_pose_cannot_renew_freshness_and_fault_requires_reset(self):
        b=self.budget();b.observe((0,0,0,10_000_000_000),1.2)
        self.assertEqual(b.check(1.31,10_310_000_000),'trial_odometry_stale')
        b.observe((0,0,0,10_400_000_000),1.4)
        self.assertEqual(b.check(1.4,10_400_000_000),'trial_odometry_stale')
        b.reset(1.4);self.assertEqual(b.check(1.4,10_400_000_000),'clear')
