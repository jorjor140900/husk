"""Local planning requires current observed image AND measured scan-plane rays."""
from types import SimpleNamespace as NS
import math
import unittest

import numpy as np

from husky_drivable_area.collision import circle_scan
from husky_drivable_area.vision_goal import plan_vision_reference
from husky_drivable_area.vision_local import (LocalLidarGrid, LocalFusionConfig, fused_sampler,
    local_goal_target,validate_near_field_assumption)


def scan(circles=(),unknown=False):
    ranges=circle_scan([0,0,0],circles,count=900,range_max=8.)
    # A measured distant enclosing wall, NOT synthetic Inf-to-free conversion.
    # The fixture represents a scene where every non-object ray hits that wall.
    if not unknown:ranges[np.isposinf(ranges)]=8.
    return NS(ranges=ranges,angle_min=-math.pi,angle_increment=2*math.pi/900,
        angle_max=-math.pi+899*2*math.pi/900,range_min=.1,range_max=8.,scan_time=.1,time_increment=0.)


def lane(points):
    return np.where(np.abs(points[:,1])<1.2,1,0).astype(np.uint8)


class LocalFusionTests(unittest.TestCase):
    def test_clear_corridor_plans_directly_toward_goal_without_global_path(self):
        grid=LocalLidarGrid(scan(),np.eye(4))
        sample=fused_sampler(lane,grid,np.eye(4))
        path,reason,_=plan_vision_reference(sample,1.,local_goal_target([4.,0.]),
            candidate_check=lambda p:grid.reference_clear(p,np.eye(4),1.))
        self.assertEqual(reason,'ok');self.assertGreater(len(path),20)
        self.assertLess(np.max(np.abs(path[:,1])),.1)
        self.assertTrue(grid.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2)))

    def test_no_return_is_unknown_and_does_not_support_motion(self):
        grid=LocalLidarGrid(scan(unknown=True),np.eye(4))
        self.assertEqual(grid.sample([[1.,0.]]).tolist(),[255])
        self.assertFalse(grid.command_clear((.05,0),(0,0),np.eye(4),.1,(.05,.2)))
        self.assertEqual(len(plan_vision_reference(fused_sampler(lane,grid,np.eye(4)),1.,[3.,0.])[0]),0)

    def test_fractionally_offset_lidar_does_not_leave_raster_holes_around_robot(self):
        matrix=np.eye(4);matrix[0,3]=-.2317
        grid=LocalLidarGrid(scan(),matrix)
        self.assertTrue(grid.clear_poses([[0.,0.,0.],[1.,0.,0.]]))
        self.assertTrue(grid.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2)))

    def test_configured_blind_range_is_not_made_free_by_a_distant_return(self):
        matrix=np.eye(4);matrix[0,3]=-.2317
        grid=LocalLidarGrid(scan(),matrix,LocalFusionConfig(lidar_min_range_m=.4))
        self.assertEqual(grid.sample([[-.60,0.]]).tolist(),[255])
        self.assertFalse(grid.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2)))
        # A distant visible reference is useful in preview, while the actual
        # command still stops for the unsupported near field.
        path,reason,_=plan_vision_reference(fused_sampler(lane,grid,np.eye(4)),1.,[4.,0.],
            candidate_check=lambda p:grid.reference_clear(p,np.eye(4),1.))
        self.assertEqual(reason,'ok');self.assertGreater(len(path),20)

    def test_live_obstacle_blocks_even_with_all_green_image(self):
        grid=LocalLidarGrid(scan([(1.4,0.,.25)]),np.eye(4))
        sample=fused_sampler(lambda p:np.ones(len(p),np.uint8),grid,np.eye(4))
        path,_,_=plan_vision_reference(sample,1.,[4.,0.],candidate_check=lambda p:grid.reference_clear(p,np.eye(4),1.))
        self.assertEqual(len(path),0)
        # Immediate near-field obstacle blocks the actual command stop sweep.
        near=LocalLidarGrid(scan([(.85,0.,.15)]),np.eye(4))
        self.assertFalse(near.command_clear((.15,0),(0,0),np.eye(4),.2,(.15,.2)))

    def test_veto_and_unknown_image_cannot_be_overruled_by_clear_lidar(self):
        grid=LocalLidarGrid(scan(),np.eye(4))
        for value in (0,255):
            sample=fused_sampler(lambda p:np.full(len(p),value,np.uint8),grid,np.eye(4))
            self.assertEqual(sample([[1.,0.]]).tolist(),[value])
            self.assertEqual(len(plan_vision_reference(sample,1.,[4.,0.])[0]),0)

    def test_new_scan_does_not_keep_a_disappeared_hit_as_free_without_evidence(self):
        blocked=LocalLidarGrid(scan([(1.4,0.,.25)]),np.eye(4))
        unknown=LocalLidarGrid(scan(unknown=True),np.eye(4))
        clear=LocalLidarGrid(scan(),np.eye(4))
        self.assertEqual(blocked.sample([[1.2,0.]]).tolist(),[0])
        self.assertEqual(unknown.sample([[1.2,0.]]).tolist(),[255])
        self.assertEqual(clear.sample([[1.2,0.]]).tolist(),[1])

    def test_registered_scan_pose_moves_evidence_into_capture_coordinates(self):
        grid=LocalLidarGrid(scan([(.9,0.,.15)]),np.eye(4))
        m=np.eye(4);m[0,3]=.2
        self.assertFalse(grid.command_clear((.05,0),(0,0),m,.1,(.05,.2)))
        self.assertEqual(grid.sample([[20.,0.]]).tolist(),[255])

    def test_goal_behind_or_large_turn_is_not_an_unchecked_spin(self):
        for goal in ([-1,0],[.1,2],[0,0],[float('nan'),0]):
            with self.assertRaises(ValueError):local_goal_target(goal)

    def test_malformed_scan_or_uncompensated_acquisition_is_rejected(self):
        for kind in ('nan','zero','timed','angle','tilt'):
            message=scan();matrix=np.eye(4)
            if kind=='nan':message.ranges[10]=float('nan')
            if kind=='zero':message.range_min=0.;message.ranges[10]=0.
            if kind=='timed':message.time_increment=.0001
            if kind=='angle':message.angle_max+=.1
            if kind=='tilt':matrix[2,2]=.9
            with self.subTest(kind=kind),self.assertRaises(ValueError):LocalLidarGrid(message,matrix)

    def test_supervised_assumption_supports_only_bounded_near_unknown_cells(self):
        matrix=np.eye(4);matrix[0,3]=-.2317
        strict=LocalLidarGrid(scan(),matrix,LocalFusionConfig(lidar_min_range_m=.4))
        assumed=LocalLidarGrid(scan(),matrix,LocalFusionConfig(lidar_min_range_m=.4,assume_near_field_clear=True))
        self.assertFalse(strict.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2)))
        self.assertTrue(assumed.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2)))
        self.assertGreater(assumed.assumed_unknown_cells,0)
        self.assertEqual(assumed.sample([[-.60,0.]]).tolist(),[255])
        np.testing.assert_array_equal(strict.labels,assumed.labels)
        blind=LocalLidarGrid(scan(unknown=True),matrix,LocalFusionConfig(lidar_min_range_m=.4,assume_near_field_clear=True))
        self.assertFalse(blind.clear_poses([[1.5,0,0]]))
        self.assertFalse(blind.command_clear((.05,0),(0,0),np.eye(4),.1,(.15,.2)))
        self.assertEqual(len(plan_vision_reference(fused_sampler(lane,blind,np.eye(4)),1.,[3.,0.])[0]),0)

    def test_assumption_never_removes_a_measured_obstacle_even_below_clearing_minimum(self):
        matrix=np.eye(4);matrix[0,3]=-.2317
        for x in (.70,-.60):
            message=scan([(x+.2317,0.,.02)])
            grid=LocalLidarGrid(message,matrix,LocalFusionConfig(lidar_min_range_m=.4,assume_near_field_clear=True))
            self.assertFalse(grid.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2)))
            self.assertTrue(np.any(grid.labels==0))
            self.assertTrue(np.all(grid.pose_support[grid.labels==0]==0))

    def test_point_outside_configured_clearance_is_not_blocked_twice_by_grid_rounding(self):
        matrix=np.eye(4);matrix[0,3]=-.2317
        message=scan([(-.80+.2317,-.15,.01)])
        grid=LocalLidarGrid(message,matrix,LocalFusionConfig(lidar_min_range_m=.4,assume_near_field_clear=True))
        self.assertTrue(np.any(grid.labels==0),'Fixture must contain a measured obstacle')
        self.assertTrue(grid.command_clear((.15,0),(0,0),np.eye(4),.1,(.15,.2)))

    def test_assumption_is_boolean_and_rejected_outside_supervised_local_mode(self):
        for local,trial in ((False,False),(True,False),(False,True)):
            with self.assertRaises(ValueError):validate_near_field_assumption(True,local,trial)
        validate_near_field_assumption(True,True,True)
        with self.assertRaises(ValueError):LocalFusionConfig(assume_near_field_clear=1)
        with self.assertRaises(ValueError):validate_near_field_assumption(True,True,'false')


if __name__=='__main__':unittest.main()
