"""Image evidence determines steering; global routes only select direction."""
import unittest
import numpy as np

from husky_drivable_area.vision_goal import (VisionReferenceConfig, plan_vision_reference,
    route_direction, sample_ground_labels,validate_guidance_mode)
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
from husky_drivable_area.vision_goal_overlay import draw_vision_reference
from test_ground_geometry import camera_mapping,mount_mapping


def lane(points,centre=0.,half=.6):
    return (np.abs(points[:,1]-centre)<=half).astype(np.uint8)


class VisionGoalTests(unittest.TestCase):
    def test_measured_near_waypoint_does_not_overshoot_its_lateral_coordinate(self):
        for sign in (-1,1):
            target=np.array([1.4591349766356165,sign*.38263434220985104])
            path,reason,detail=plan_vision_reference(lambda p:np.ones(len(p),np.uint8),
                1.0085270909607027,target,goal_priority=True)
            self.assertEqual(reason,'ok')
            np.testing.assert_allclose(path[-1],target,atol=1e-8)
            self.assertLess(detail['max_curvature'],1.5)
            self.assertLess(detail['reference_end_to_target_m'],1e-8)

    def test_exact_goal_arc_cannot_override_an_image_veto(self):
        target=np.array([1.4591349766356165,.38263434220985104])
        def sample(p):
            return np.where(np.linalg.norm(p-target,axis=1)<.15,0,1).astype(np.uint8)
        _,_,detail=plan_vision_reference(sample,1.0085270909607027,target,goal_priority=True)
        self.assertGreater(detail.get('reference_end_to_target_m',1.),.15)

    def test_almost_straight_exact_arc_remains_bounded_and_close_goal_survives_grid_rounding(self):
        for target in ([1.4,.00001],[1.2,0.]):
            path,reason,_=plan_vision_reference(lambda p:np.ones(len(p),np.uint8),1.0085270909607027,target,goal_priority=True)
            self.assertEqual(reason,'ok');self.assertLess(len(path),500)
            self.assertLess(np.linalg.norm(path[-1]-target),.03)

    def test_goal_priority_does_not_follow_an_offset_corridor_centre_away_from_a_clear_goal(self):
        sample=lambda p:lane(p,centre=.3,half=.8)
        legacy,_,_=plan_vision_reference(sample,.8,[2.5,0.])
        path,reason,detail=plan_vision_reference(sample,.8,[2.5,0.],goal_priority=True)
        self.assertEqual(reason,'ok');self.assertTrue(detail['goal_priority'])
        self.assertGreater(legacy[-1,1],.05)
        np.testing.assert_allclose(path[-1],[2.5,0.],atol=1e-8)
        self.assertLess(np.abs(path[:,1]).max(),1e-8)

    def test_goal_priority_reaches_near_goal_endpoint_and_responds_to_left_right_goals(self):
        for y in (-.3,.3):
            target=np.array([2.4,y])
            path,reason,detail=plan_vision_reference(lambda p:lane(p,half=1.2),.8,target,goal_priority=True)
            self.assertEqual(reason,'ok')
            np.testing.assert_allclose(path[-1],target,atol=1e-8)
            self.assertLess(detail['reference_end_to_target_m'],1e-8)
            self.assertLessEqual(path[:,0].max(),target[0]+1e-9)

    def test_goal_priority_preserves_veto_unknown_and_start_width_constraints(self):
        for label in (0,255):
            def sample(p):
                values=lane(p,half=1.2)
                values[(p[:,0]>1.2)&(p[:,0]<1.4)]=label
                return values
            path,_,_=plan_vision_reference(sample,1.,[2.5,0.],goal_priority=True)
            self.assertEqual(len(path),0)
        path,_,_=plan_vision_reference(lambda p:lane(p,half=.35),1.,[2.5,0.],goal_priority=True)
        self.assertEqual(len(path),0)

    def test_image_only_requires_explicit_supervision_and_vision_steering(self):
        with self.assertRaisesRegex(ValueError,'supervised_trial'):
            validate_guidance_mode('image_only')
        with self.assertRaisesRegex(ValueError,'requires vision'):
            validate_guidance_mode('image_only',False,True)
        validate_guidance_mode('image_only',True,True)
        validate_guidance_mode('local_fusion')
        validate_guidance_mode('global_route')
        with self.assertRaisesRegex(ValueError,'guidance_mode'):
            validate_guidance_mode('imag_only',True,True)

    def test_straight_corridor_yields_straight_reference_even_if_global_route_bends(self):
        path,reason,detail=plan_vision_reference(lane,1.,[2.5,.3])
        self.assertEqual(reason,"ok")
        self.assertGreater(len(path),30)
        self.assertLess(np.max(np.abs(path[:,1])),.16)
        self.assertAlmostEqual(detail["reserved_width_m"],.82)
        self.assertFalse(detail["footprint_checked"])
        np.testing.assert_allclose(path[0],[1.,0.])

    def test_camera_corridor_can_curve_without_copying_straight_map_guide(self):
        def sample(p):
            t=np.clip((p[:,0]-1)/1.8,0,1)
            centre=.4*(3*t*t-2*t*t*t)
            return (np.abs(p[:,1]-centre)<.55).astype(np.uint8)
        path,reason,_=plan_vision_reference(sample,1.,[4.,0.])
        self.assertEqual(reason,"ok")
        self.assertGreater(path[-1,1],.2)

    def test_veto_hole_in_start_width_is_not_filled_or_seed_relocated(self):
        def sample(p):
            labels=lane(p)
            labels[(p[:,0]<1.1)&(np.abs(p[:,1]-.2)<.04)]=0
            return labels
        path,reason,_=plan_vision_reference(sample,1.,[3.,0.])
        self.assertEqual(len(path),0)
        self.assertIn("start_blocked",reason)

    def test_no_jump_over_blocked_or_unknown_crossbar(self):
        for label in (0,255):
            def sample(p):
                labels=lane(p);labels[(p[:,0]>1.2)&(p[:,0]<1.4)]=label
                return labels
            path,reason,_=plan_vision_reference(sample,1.,[3.,0.])
            self.assertEqual(len(path),0)
            self.assertEqual(reason,"no_observed_image_corridor")

    def test_too_narrow_lawn_and_unobserved_frame_cannot_produce_reference(self):
        for sample in (lambda p:lane(p,half=.35),lambda p:np.zeros(len(p),np.uint8),
                       lambda p:np.full(len(p),255,np.uint8)):
            self.assertEqual(len(plan_vision_reference(sample,1.,[3.,0.])[0]),0)

    def test_goal_direction_selects_a_branch_inside_wide_image_corridor(self):
        paths=[]
        for sign in (-1,1):
            path,reason,_=plan_vision_reference(lambda p:lane(p,half=1.2),.8,[2.,sign*.9])
            self.assertEqual(reason,"ok")
            paths.append(path)
            self.assertGreater(sign*path[-1,1],.4)
        np.testing.assert_allclose(paths[0][:,1],-paths[1][:,1],atol=.06)

    def test_route_behind_or_extreme_heading_requires_reorientation(self):
        route=np.column_stack([np.linspace(0,4,41),np.zeros(41)])
        np.testing.assert_allclose(route_direction(route,[4.,0.]),[2.5,0.])
        for goal in ([-1.,0.],[.1,2.]):
            with self.assertRaises(ValueError):
                route_direction(route,goal)

    def test_projection_preserves_veto_unknown_and_actual_camera_size(self):
        camera=CameraCalibration.from_mapping(camera_mapping())
        mount=CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,
            camera_x_m=-.26,camera_y_m=.21))
        for value in (0,1,255):
            labels=np.full((480,640),value,np.uint8)
            result=sample_ground_labels([[1.5,0.],[1.,100.]],labels,camera,mount)
            self.assertEqual(result.tolist(),[value,255])

    def test_invalid_configuration_and_unobservable_axis_fail_closed(self):
        for values in ({"step_m":.5},{"side_margin_m":-1},{"max_forward_m":20}):
            with self.assertRaises(ValueError):VisionReferenceConfig(**values)
        for start in (-1,3,float("nan")):
            self.assertEqual(len(plan_vision_reference(lane,start,[3.,0.])[0]),0)

    def test_camera_overlay_draws_controller_reference_without_second_pixel_planner(self):
        camera=CameraCalibration.from_mapping(camera_mapping())
        mount=CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,
            camera_x_m=-.26,camera_y_m=.21))
        frame=np.full((480,640,3),140,np.uint8)
        labels=np.ones((480,640),np.uint8)
        reference=np.column_stack([np.linspace(1,2.5,61),np.full(61,.15)])
        image=draw_vision_reference(frame,labels,reference,camera,mount,"test")
        self.assertEqual(image.shape,frame.shape)
        yellow=(image[90:,:,0]<80)&(image[90:,:,1]>180)&(image[90:,:,2]>180)
        self.assertGreater(np.count_nonzero(yellow),100)
        stopped=draw_vision_reference(frame,labels,[],camera,mount,"no path")
        yellow=(stopped[90:,:,0]<80)&(stopped[90:,:,1]>180)&(stopped[90:,:,2]>180)
        self.assertEqual(np.count_nonzero(yellow),0)

    def test_red_edges_can_show_while_waiting_for_goal_without_inventing_yellow_reference(self):
        camera=CameraCalibration.from_mapping(camera_mapping())
        mount=CameraMount.from_mapping(mount_mapping())
        frame=np.full((480,640,3),140,np.uint8)
        labels=np.zeros((480,640),np.uint8)
        for y in range(140,480):
            left=round(240-.35*(y-140));right=round(400+.35*(y-140))
            labels[y,left:right]=1
        labels[270:300,310:325]=0  # Interior veto is never filled by an edge fit.
        before=labels.copy()
        image=draw_vision_reference(frame,labels,[],camera,mount,"waiting_for_goal_guide")
        red=(image[90:,:,0]<60)&(image[90:,:,1]<60)&(image[90:,:,2]>200)
        yellow=(image[90:,:,0]<80)&(image[90:,:,1]>180)&(image[90:,:,2]>180)
        self.assertGreater(np.count_nonzero(red),300)
        self.assertEqual(np.count_nonzero(yellow),0)
        np.testing.assert_array_equal(labels,before)
        self.assertTrue(np.all(frame==140))
        for value in (1,255):
            # All-green with edges outside view, or all-unknown, has no fitted sides.
            absent=draw_vision_reference(frame,np.full_like(labels,value),[],camera,mount,"waiting")
            red=(absent[90:,:,0]<60)&(absent[90:,:,1]<60)&(absent[90:,:,2]>200)
            self.assertEqual(np.count_nonzero(red),0)


if __name__=="__main__":
    unittest.main()
