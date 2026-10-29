"""Image evidence determines steering; global routes only select direction."""
import unittest
import numpy as np

from husky_drivable_area.vision_goal import (VisionReferenceConfig, plan_vision_reference,
    route_direction, sample_ground_labels)
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
from husky_drivable_area.vision_goal_overlay import draw_vision_reference
from test_ground_geometry import camera_mapping,mount_mapping


def lane(points,centre=0.,half=.6):
    return (np.abs(points[:,1]-centre)<=half).astype(np.uint8)


class VisionGoalTests(unittest.TestCase):
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
