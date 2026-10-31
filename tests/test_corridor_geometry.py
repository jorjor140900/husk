"""Separate a narrow projected lane, an off-axis goal and a real center veto."""
import json
import unittest

import numpy as np

from tools.check_corridor_geometry import summarize_sections,ground_labels
from husky_drivable_area.ground_geometry import CameraCalibration,CameraMount
from test_ground_geometry import camera_mapping,mount_mapping


def corridor(points,half=.6):
    return np.where(np.abs(points[:,1])<=half+1e-8,1,0).astype(np.uint8)


class CorridorGeometryTests(unittest.TestCase):
    def test_centered_120_cm_corridor_fits_existing_96_cm_reservation(self):
        report=summarize_sections(corridor,.33,.15,np.array([4.,0.]))
        self.assertEqual(report["reserved_width_m"],.96)
        self.assertEqual(report["goal_angle_from_robot_forward_deg"],0.)
        for row in report["sections"]:
            self.assertAlmostEqual(row["width_around_robot_axis_m"],1.2,delta=.03)
            self.assertTrue(row["reserved_width_fits_at_axis"])
            self.assertTrue(row["reserved_width_fits_at_goal_line"])
        self.assertFalse(report["ready_for_control"])
        json.dumps(report,allow_nan=False)

    def test_off_axis_goal_can_cross_veto_while_robot_axis_remains_clear(self):
        report=summarize_sections(corridor,.33,.08,np.array([4.,1.2]))
        self.assertGreater(report["goal_angle_from_robot_forward_deg"],16.)
        self.assertTrue(all(row["reserved_width_fits_at_axis"] for row in report["sections"]))
        self.assertEqual(report["sections"][-1]["goal_line_label"],"veto")
        self.assertFalse(report["sections"][-1]["reserved_width_fits_at_goal_line"])

    def test_projected_narrow_lane_is_reported_without_moving_axis_or_changing_mask(self):
        report=summarize_sections(lambda p:corridor(p,.35),.33,.08)
        for row in report["sections"]:
            self.assertAlmostEqual(row["width_around_robot_axis_m"],.7,delta=.03)
            self.assertTrue(row["axis_drivable"])
            self.assertFalse(row["reserved_width_fits_at_axis"])

    def test_center_hole_does_not_select_a_different_free_interval(self):
        def sample(points):
            labels=corridor(points)
            labels[(np.abs(points[:,0]-1.5)<1e-6)&(np.abs(points[:,1])<.06)]=0
            return labels
        report=summarize_sections(sample,.33,.08)
        row=report["sections"][2]
        self.assertFalse(row["axis_drivable"])
        self.assertIsNone(row["width_around_robot_axis_m"])
        self.assertFalse(row["reserved_width_fits_at_axis"])

    def test_unobserved_plane_is_not_reported_as_a_free_corridor(self):
        report=summarize_sections(lambda p:np.full(len(p),255,np.uint8),.33,.08)
        self.assertTrue(all(row["observed_samples"]==0 and row["width_around_robot_axis_m"] is None
                            and not row["reserved_width_fits_at_axis"] for row in report["sections"]))

    def test_small_interior_hole_is_distinguished_from_the_outer_corridor_boundary(self):
        def sample(points):
            labels=corridor(points)
            labels[(np.abs(points[:,0]-1.5)<1e-6)&(np.abs(points[:,1]-.15)<.026)]=0
            return labels
        report=summarize_sections(sample,.33,.15,colour_sample=lambda p:np.ones(len(p),np.uint8))
        row=report["sections"][2]
        self.assertEqual(len(row["drivable_intervals_y_m"]),2)
        self.assertAlmostEqual(row["width_around_robot_axis_m"],.7375,delta=.04)
        self.assertEqual(len(row["interior_veto_holes"]),1)
        hole=row["interior_veto_holes"][0]
        self.assertFalse(hole["colour_veto_present"])
        self.assertGreater(hole["model_veto_without_colour_samples"],0)
        json.dumps(report,allow_nan=False)

    def test_colour_veto_at_same_capture_is_reported_without_removing_the_hole(self):
        def sample(points):
            labels=corridor(points)
            labels[np.abs(points[:,1]-.15)<.026]=0
            return labels
        report=summarize_sections(sample,.33,.08,colour_sample=sample)
        self.assertTrue(report["sections"][0]["interior_veto_holes"][0]["colour_veto_present"])
        self.assertEqual(report["sections"][0]["interior_veto_holes"][0]["model_veto_without_colour_samples"],0)
        self.assertFalse(report["sections"][0]["reserved_width_fits_at_axis"])

    def test_projection_reports_only_visible_consistent_raw_labels(self):
        camera=CameraCalibration.from_mapping(camera_mapping())
        mount=CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,camera_x_m=-.26,camera_y_m=.21))
        labels=np.ones((480,640),np.uint8)
        actual=ground_labels(np.array([[1.,0.],[1.,100.]]),labels,camera,mount)
        self.assertEqual(actual.tolist(),[1,255])
