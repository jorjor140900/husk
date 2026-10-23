"""Recorded tracking statistics and report preservation, without ROS."""

import json
from pathlib import Path
import tempfile
import unittest

from husky_drivable_area.control_report import summarize_tracking, write_report


class ControlReportTests(unittest.TestCase):
    def test_rmse_uses_every_sample_and_tail_uses_final_tenth(self):
        summary = summarize_tracking([0.3, 0.4] + [0.0] * 18)
        self.assertAlmostEqual(summary["rmse_m"], (0.25/20)**0.5)
        self.assertEqual(summary["samples"], 20)
        self.assertEqual(summary["max_m"], 0.4)
        self.assertEqual(summary["initial_m"], 0.3)
        self.assertEqual(summary["tail_mean_m"], 0)
        self.assertEqual(summarize_tracking([0.0] * 10)["rmse_m"], 0)
        self.assertEqual(summarize_tracking([1e308] * 20)["rmse_m"], 1e308)

    def test_invalid_recordings_do_not_create_a_report(self):
        for errors in ([], [None], [True], [-1], [float("nan")], [float("inf")]):
            with self.assertRaises(ValueError):
                summarize_tracking(errors)
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "not-created"
            with self.assertRaises(ValueError):
                write_report(output, {"bad": float("nan")})
            self.assertFalse(output.exists())

    def test_each_run_preserves_prior_reports_including_failed_runs(self):
        with tempfile.TemporaryDirectory() as folder:
            first = write_report(folder, {"result": "passed", "physical_robot_validated": False})
            second = write_report(folder, {"result": "failed", "failed_check": "stale_path"})
            self.assertNotEqual(first, second)
            self.assertEqual(json.loads(first.read_text())["result"], "passed")
            self.assertEqual(json.loads(second.read_text())["failed_check"], "stale_path")
            self.assertTrue((Path(folder) / "COLCON_IGNORE").is_file())


if __name__ == "__main__":
    unittest.main()
