"""Diagnose NO_VALID_PATH without a path or changing a published grid."""
import copy
import unittest
from test_navigation_plan_check import grid
from tools.check_goal_costmap import analyze_goal

class GoalCostmapCheckTests(unittest.TestCase):
    def test_free_goal_does_not_need_plan_and_does_not_certify_control(self):
        result=analyze_goal([.25,.55],[1.75,.55],grid(),grid())
        self.assertTrue(result['point_grid_connected_4_neighbour'])
        self.assertFalse(result['ready_for_control'])
        self.assertEqual(result['start']['costmap_label'],'free')

    def test_endpoints_and_outside_are_reported_independently(self):
        costmap=grid();costmap.data[5*20+2]=100;costmap.data[5*20+17]=-1
        result=analyze_goal([.25,.55],[1.75,.55],costmap,grid())
        self.assertEqual(result['start']['costmap_label'],'blocked')
        self.assertEqual(result['goal']['costmap_label'],'unknown')
        self.assertIsNone(result['point_grid_connected_4_neighbour'])
        result=analyze_goal([.25,.55],[3.,.55],costmap,grid())
        self.assertEqual(result['goal']['costmap_label'],'outside')

    def test_added_wall_disconnects_free_endpoints_but_saved_map_is_free(self):
        costmap=grid()
        for row in range(20):costmap.data[row*20+10]=100
        before=copy.deepcopy(costmap.data)
        result=analyze_goal([.25,.55],[1.75,.55],costmap,grid())
        self.assertFalse(result['point_grid_connected_4_neighbour'])
        self.assertGreater(result['straight_line']['blocked_but_free_in_saved_map'],0)
        self.assertEqual(costmap.data,before)

    def test_soft_inflation_is_not_automatically_blocked(self):
        costmap=grid()
        for row in range(20):costmap.data[row*20+10]=60
        result=analyze_goal([.25,.55],[1.75,.55],costmap,grid())
        self.assertTrue(result['point_grid_connected_4_neighbour'])
        self.assertEqual(result['straight_line']['blocked_cells'],0)
