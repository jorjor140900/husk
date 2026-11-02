"""Mask-only assistance: continuous green corridors, no goals or red fits."""
import unittest
import numpy as np
from husky_drivable_area.area_reference import plan_area_reference


class AreaAssistTests(unittest.TestCase):
    def test_straight_green_corridor_continues_forward_without_a_goal(self):
        sample=lambda p:(abs(p[:,1])<.9).astype(np.uint8)
        path,reason,detail=plan_area_reference(sample,.8)
        self.assertEqual(reason,'ok');self.assertGreater(path[-1,0],2.4)
        self.assertLess(abs(path[:,1]).max(),.08)
        self.assertFalse(detail['red_edges_used_for_planning'])
        self.assertFalse(detail['goal_priority'])

    def test_curved_green_corridor_steers_in_both_directions(self):
        for sign in (-1,1):
            def sample(p):
                centre=sign*.35*np.maximum(p[:,0]-1.,0)**2
                return (abs(p[:,1]-centre)<.85).astype(np.uint8)
            path,reason,_=plan_area_reference(sample,.8)
            self.assertEqual(reason,'ok');self.assertGreater(sign*path[-1,1],.3)
            delta=np.gradient(path,axis=0);normal=np.column_stack([-delta[:,1],delta[:,0]])/np.linalg.norm(delta,axis=1)[:,None]
            strip=(path[:,None,:]+normal[:,None,:]*np.linspace(-.41,.41,35)[None,:,None]).reshape(-1,2)
            self.assertTrue(np.all(sample(strip)==1))

    def test_disconnected_green_is_not_a_route_across_veto_or_unknown(self):
        for label in (0,255):
            def sample(p):return np.where((p[:,0]>1.1)&(p[:,0]<1.8),label,1).astype(np.uint8)
            path,_,_=plan_area_reference(sample,.8)
            self.assertEqual(len(path),0)

    def test_narrow_green_is_not_widened_to_fit_robot(self):
        path,_,_=plan_area_reference(lambda p:(abs(p[:,1])<.3).astype(np.uint8),.8)
        self.assertEqual(len(path),0)

    def test_search_can_detour_around_non_drivable_island(self):
        def sample(p):
            return ((abs(p[:,1])<1.5)&~((p[:,0]>1.6)&(p[:,0]<1.9)&(abs(p[:,1])<.18))).astype(np.uint8)
        path,reason,_=plan_area_reference(sample,.8)
        self.assertEqual(reason,'ok');self.assertGreater(path[-1,0],2.4)
        self.assertGreater(abs(path[:,1]).max(),.6)


if __name__=='__main__':unittest.main()
