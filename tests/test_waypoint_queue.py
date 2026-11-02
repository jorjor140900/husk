import unittest
from husky_drivable_area.waypoint_queue import WaypointQueue


class QueueTests(unittest.TestCase):
    def test_u_waypoints_are_dispatched_in_order_and_old_completion_is_ignored(self):
        q=WaypointQueue()
        for point in [(1.2,0),(3,1.5),(1.5,3),(0,3)]:q.add(*point)
        q.start()
        for i in range(4):
            q.token=100+i
            self.assertFalse(q.reached(99+i))
            self.assertEqual(q.index,i)
            self.assertTrue(q.reached(100+i))
            self.assertFalse(q.reached(100+i))
        self.assertFalse(q.active);self.assertEqual(q.index,4)

    def test_collecting_does_not_start_motion_and_clear_prevents_auto_advance(self):
        q=WaypointQueue();q.add(1,0);self.assertFalse(q.active)
        q.start();q.token=123;q.clear()
        self.assertFalse(q.reached(123));self.assertFalse(q.active)
        self.assertEqual(q.points,[])

    def test_empty_active_and_unbounded_queue_are_rejected(self):
        q=WaypointQueue()
        with self.assertRaises(ValueError):q.start()
        with self.assertRaises(ValueError):q.add(float('nan'),0)
        q.add(1,0);q.start()
        with self.assertRaises(ValueError):q.start()
        for i in range(127):q.add(i,0)
        with self.assertRaises(ValueError):q.add(129,0)
