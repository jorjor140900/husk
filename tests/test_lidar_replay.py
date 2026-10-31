"""Offline scan comparison must preserve nearer returns and missing evidence."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest

import numpy as np

from husky_drivable_area.vlp16 import TimedCloud
from tools.replay_lidar_bag import bin_ring, cloud_context, matches_recorded, write_replay


class LidarReplayTests(unittest.TestCase):
    def cloud(self, xyz, rings):
        return TimedCloud(100_000_000_000, "velodyne", np.array(xyz, dtype=float),
                          np.array(rings, dtype=np.uint16), np.zeros(len(rings)))

    def test_nearest_survives_duplicate_bin_and_later_farther_return(self):
        cloud = self.cloud([[1, 0, 1], [8, 0, 1], [3, 0, 1], [2, .6, 1], [.1, 0, 1]],
                           [8, 8, 8, 8, 7])
        scan = NS(ranges=[np.inf]*4, angle_min=0., angle_increment=.1)
        last, nearest = bin_ring(cloud, scan, 8)
        self.assertEqual(last[0], 3.)
        self.assertEqual(nearest[0], 1.)
        self.assertAlmostEqual(nearest[2], np.hypot(2, .6), places=6)
        np.testing.assert_array_equal(np.isposinf(nearest), [False, True, False, True])
        self.assertTrue(np.all(nearest <= last))
        # Ring 7's much closer return must not silently replace the matched ring.
        self.assertNotEqual(nearest[0], .1)

    def test_order_changes_legacy_last_but_not_nearest(self):
        scan = NS(ranges=[np.inf]*2, angle_min=0., angle_increment=.1)
        a = self.cloud([[1, 0, 1], [8, 0, 1]], [8, 8])
        b = self.cloud([[8, 0, 1], [1, 0, 1]], [8, 8])
        la, na = bin_ring(a, scan, 8)
        lb, nb = bin_ring(b, scan, 8)
        self.assertNotEqual(la[0], lb[0])
        np.testing.assert_array_equal(na, nb)

    def test_matching_does_not_accept_filled_or_missing_bins(self):
        predicted = np.array([1, np.inf, 3])
        self.assertTrue(matches_recorded(predicted, [1, np.inf, 3]))
        self.assertFalse(matches_recorded(predicted, [1, 200, 3]))
        self.assertFalse(matches_recorded(predicted, [1, np.inf, np.inf]))

    def test_side_context_really_is_the_front_strip_and_samples_every_ring(self):
        cloud = self.cloud([[1, 0, 0], [2, 0, 1], [3, 1, 2]], [0, 8, 8])
        context, selected = cloud_context(cloud, np.eye(4), 8)
        self.assertEqual(context, [[1., 0.], [2., 1.]])
        self.assertEqual(selected, [[2., 1.]])

    def test_untrusted_filename_cannot_inject_script_into_standalone_page(self):
        replay = dict(summary=dict(source_file="</script><script>alert(1)</script>"), frames=[])
        with TemporaryDirectory() as directory:
            write_replay(replay, Path(directory))
            html = (Path(directory)/"replay.html").read_text()
            self.assertNotIn(replay["summary"]["source_file"], html)
            embedded = html.split('<script id="data" type="application/json">', 1)[1].split('</script>', 1)[0]
            self.assertEqual(json.loads(embedded), replay)
            self.assertEqual(json.loads((Path(directory)/"report.json").read_text()), replay["summary"])


if __name__ == "__main__":
    unittest.main()
