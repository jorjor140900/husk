import unittest
import numpy as np
from unittest import mock
from husky_drivable_area.reference_stability import reusable_reference,choose_stable_reference,EdgeSmoother
from road_navigation.reference_geometry import BoundaryLine
from road_navigation.detector import RoadDetector,DetectorConfig


class ReferenceStabilityTests(unittest.TestCase):
    def test_small_optical_flow_grid_preserves_output_dimensions_and_negative_veto(self):
        detector=RoadDetector(DetectorConfig(use_temporal=True,temporal_flow_width=160,
            temporal_alpha=.35,generate_reference_path=False))
        frame=np.full((480,640,3),100,np.uint8)
        p=np.full((480,640),.8,np.float32)
        detector.process(frame,drivable_probability=p)
        with mock.patch('cv2.calcOpticalFlowFarneback',return_value=np.zeros((120,160,2),np.float32)) as flow:
            result=detector.process(frame,drivable_probability=np.full((480,640),.2,np.float32))
        self.assertEqual(flow.call_args.args[0].shape,(120,160))
        self.assertEqual(result.raw_mask.shape,(480,640))
        self.assertEqual(result.metadata()['drivable_pixels'],0)

    def test_probability_flicker_is_filtered_but_strong_current_negative_wins(self):
        detector=RoadDetector(DetectorConfig(max_width=320,analysis_width=320,use_temporal=True,
            temporal_alpha=.35,generate_reference_path=False))
        frame=np.full((240,320,3),100,np.uint8)
        def process(p):return detector.process(frame,drivable_probability=np.full((240,320),p,np.float32))
        self.assertGreater(process(.8).raw_mask[200,160],0)
        self.assertGreater(process(.58).raw_mask[200,160],0)
        self.assertEqual(process(.2).raw_mask[200,160],0)
        for _ in range(10):result=process(.58)
        self.assertEqual(result.raw_mask[200,160],0)
    def test_mask_hysteresis_keeps_weak_road_but_never_reopens_negative_or_new_weak_area(self):
        detector=RoadDetector(DetectorConfig(max_width=320,analysis_width=320,use_temporal=True,
            temporal_flow_width=160,temporal_alpha=.35,probability_release_threshold=.54,generate_reference_path=False))
        frame=np.full((240,320,3),100,np.uint8)
        def process(p):return detector.process(frame,drivable_probability=np.full((240,320),p,np.float32))
        self.assertGreater(process(.8).raw_mask[200,160],0)
        for _ in range(20):result=process(.58)
        self.assertGreater(result.raw_mask[200,160],0)
        self.assertEqual(process(.2).raw_mask[200,160],0)
        for _ in range(20):result=process(.58)
        self.assertEqual(result.raw_mask[200,160],0)

    def test_previous_geometry_is_kept_only_when_new_capture_supports_whole_width(self):
        path=np.column_stack([np.linspace(.9,2.5,65),np.zeros(65)])
        sample=lambda p:np.ones(len(p),np.uint8)
        old=reusable_reference(path,sample,1.,.41)
        self.assertIsNotNone(old);self.assertGreaterEqual(old[0,0],1.)
        for value in (0,255):
            def blocked(p):return np.where((p[:,0]>1.5)&(np.abs(p[:,1]-.2)<.05),value,1).astype(np.uint8)
            self.assertIsNone(reusable_reference(path,blocked,1.,.41))

    def test_hysteresis_preserves_a_valid_old_path_but_not_wrong_goal_direction(self):
        old=np.array([[1.,0.],[2.,0.]])
        new=np.array([[1.,0.],[2.05,0.]])
        self.assertTrue(choose_stable_reference(new,old,np.array([2.05,0.]))[1])
        self.assertFalse(choose_stable_reference(new,old,np.array([2.05,0.]),slack=.02)[1])
        self.assertTrue(choose_stable_reference([],old,np.array([2.,0.]))[1])

    def test_current_red_edges_are_smoothed_without_inventing_absent_edges(self):
        smooth=EdgeSmoother();edge=BoundaryLine(0.,100.,200,450,40)
        smooth.update((edge,None),1_000_000_000,(480,640))
        moved=BoundaryLine(0.,110.,200,450,40)
        result=smooth.update((moved,None),1_066_000_000,(480,640))
        self.assertAlmostEqual(result[0].intercept,104.)
        self.assertIsNone(result[1])
        self.assertIsNone(smooth.update((None,None),1_100_000_000,(480,640))[0])
