"""Costmap evidence explains a detour without weakening planning constraints."""
import copy
from types import SimpleNamespace as NS
import unittest

import numpy as np

from husky_drivable_area.navigation_plan_check import analyze_plan


def grid():
    return NS(header=NS(frame_id="map"),info=NS(width=20,height=20,resolution=.1,
        origin=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*400)


def path(points):
    return NS(header=NS(frame_id="map"),poses=[NS(header=NS(frame_id="map"),
        pose=NS(position=NS(x=x,y=y))) for x,y in points])


class PlanCheckTests(unittest.TestCase):
    def test_straight_free_plan_has_no_detour_and_does_not_certify_control(self):
        report=analyze_plan(path([[.2,.5],[1.8,.5]]),grid(),grid())
        self.assertEqual(report["detour_ratio"],1.)
        self.assertEqual(report["straight_line"]["blocked_cells"],0)
        self.assertEqual(report["max_lateral_departure_m"],0.)
        self.assertFalse(report["ready_for_control"])

    def test_added_costmap_obstacle_and_soft_costs_are_distinguished_from_saved_map(self):
        costmap=grid();saved=grid()
        costmap.data[5*20+10]=100;costmap.data[5*20+9]=60
        before=copy.deepcopy(costmap.data)
        report=analyze_plan(path([[.2,.55],[.5,1.3],[1.5,1.3],[1.8,.55]]),costmap,saved)
        line=report["straight_line"]
        self.assertGreater(report["detour_ratio"],1.5)
        self.assertGreater(report["max_lateral_departure_m"],.7)
        self.assertEqual(line["blocked_cells"],1)
        self.assertEqual(line["soft_cost_cells"],1)
        self.assertEqual(line["saved_map_occupied_cells"],0)
        self.assertEqual(line["blocked_but_free_in_saved_map"],1)
        self.assertEqual(costmap.data,before)

    def test_saved_map_obstacle_unknown_and_outside_are_separate(self):
        costmap=grid();saved=grid()
        costmap.data[5*20+10]=100;saved.data[5*20+10]=100
        costmap.data[5*20+9]=-1
        report=analyze_plan(path([[-.2,.55],[1.8,.55]]),costmap,saved)
        line=report["straight_line"]
        self.assertGreater(line["outside_costmap_cells"],0)
        self.assertEqual(line["unknown_cells"],1)
        self.assertEqual(line["saved_map_occupied_cells"],1)
        self.assertEqual(line["blocked_but_free_in_saved_map"],0)

    def test_rotated_grid_origin_is_respected(self):
        costmap=grid();saved=grid()
        for value in (costmap,saved):
            value.info.origin.orientation.z=np.sqrt(.5)
            value.info.origin.orientation.w=np.sqrt(.5)
            value.info.origin.position.x=2.
        costmap.data[5*20+10]=100
        report=analyze_plan(path([[1.45,.2],[1.45,1.8]]),costmap,saved)
        self.assertEqual(report["straight_line"]["blocked_cells"],1)

    def test_invalid_frames_empty_route_and_zero_distance_are_rejected(self):
        for route in (path([]),path([[1,1],[1,1]])):
            with self.assertRaises(ValueError):analyze_plan(route,grid(),grid())
        route=path([[.2,.5],[1.8,.5]]);route.poses[0].header.frame_id="odom"
        with self.assertRaises(ValueError):analyze_plan(route,grid(),grid())
        route=path([[.2,.5],[1.8,.5]]);costmap=grid();costmap.header.frame_id="odom"
        with self.assertRaises(ValueError):analyze_plan(route,costmap,grid())
