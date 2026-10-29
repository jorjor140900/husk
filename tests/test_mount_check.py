import unittest
from unittest import mock
import json
from pathlib import Path
import tempfile
import yaml

import numpy as np

from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, ground_to_image
from husky_drivable_area.mount_check import check_floor_points, corridor_points, check_corridor_points, raw_measurement_frame
from test_ros_adapter import header
from husky_drivable_area.transport import fill_image
from types import SimpleNamespace
from husky_drivable_area import mount_check
from test_ground_geometry import camera_mapping, mount_mapping


class MountCheckTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration.from_mapping(camera_mapping())
        self.mount = CameraMount.from_mapping(mount_mapping(height_m=.99,
            pitch_down_deg=18.16, roll_deg=-.47, yaw_deg=1.53,
            camera_x_m=-.26, camera_y_m=.21))
        self.world = np.array([[1., 0.], [1.5, 0.], [1.5, .2]])

    def test_recovers_independent_ground_distances_and_side_offset(self):
        pixels, _ = ground_to_image(self.world, self.camera, self.mount)
        report = check_floor_points(self.camera, self.mount, pixels, self.world)
        np.testing.assert_allclose(report["estimated_xy_m"], self.world, atol=1e-9)
        np.testing.assert_allclose(report["error_xy_cm"], 0, atol=1e-7)
        self.assertFalse(report["mount_changed"])
        self.assertFalse(report["ready_for_control"])

    def test_reports_measured_errors_without_refitting(self):
        actual = self.world + [0.08, -.02]
        pixels, _ = ground_to_image(actual, self.camera, self.mount)
        report = check_floor_points(self.camera, self.mount, pixels, self.world)
        np.testing.assert_allclose(report["error_xy_cm"], [[8, -2]]*3, atol=1e-7)

    def test_rejects_bad_points_and_horizon(self):
        for pixels in ([[np.nan, 1]]*3, [[-1, 20]]*3, [[1000, 1000]]*3, [[1, 2]]):
            with self.assertRaises(ValueError):
                check_floor_points(self.camera, self.mount, pixels, self.world)
        level = CameraMount.from_mapping(mount_mapping(pitch_down_deg=0))
        with self.assertRaisesRegex(ValueError, "intersect"):
            check_floor_points(self.camera, level, [[319.5, 239.5]], [[1., 0.]])

    def test_three_crossbars_recover_width_and_center_independently_of_ai(self):
        world = corridor_points(1.5, 2.5, 1.0)
        np.testing.assert_allclose(world, [[1.5,.5],[1.5,-.5],[2.,.5],[2.,-.5],[2.5,.5],[2.5,-.5]])
        pixels, visible = ground_to_image(world, self.camera, self.mount)
        self.assertTrue(visible.all())
        report = check_corridor_points(self.camera, self.mount, pixels, world)
        for section in report["width_checks"]:
            self.assertAlmostEqual(section["measured_width_m"], 1.0)
            self.assertAlmostEqual(section["estimated_width_m"], 1.0)
            self.assertAlmostEqual(section["estimated_lateral_width_m"], 1.0)
            self.assertAlmostEqual(section["width_error_cm"], 0)
            np.testing.assert_allclose(section["center_error_xy_cm"], 0, atol=1e-7)
        self.assertFalse(report["ai_mask_used"])
        self.assertFalse(report["mount_changed"])

    def test_wrong_camera_height_reports_narrow_projection_without_changing_mount(self):
        from dataclasses import asdict
        world = corridor_points(1.5, 2.5, 1.0)
        pixels, _ = ground_to_image(world, self.camera, self.mount)
        wrong_mount = CameraMount.from_mapping({**asdict(self.mount), "height_m": self.mount.height_m*.8})
        before = asdict(wrong_mount)
        report = check_corridor_points(self.camera, wrong_mount, pixels, world)
        for section in report["width_checks"]:
            self.assertAlmostEqual(section["estimated_width_m"], .8)
            self.assertAlmostEqual(section["width_error_cm"], -20)
        self.assertEqual(asdict(wrong_mount), before)

    def test_corridor_rejects_invalid_measurements_and_wrong_pair_count(self):
        for values in ((0,2,1),(2,1,1),(1,2,0),(1,2,float("nan"))):
            with self.assertRaises(ValueError): corridor_points(*values)
        pixels, _ = ground_to_image(self.world, self.camera, self.mount)
        with self.assertRaisesRegex(ValueError,"three measured"):
            check_corridor_points(self.camera,self.mount,pixels,self.world)

    def test_live_capture_requires_original_size_frame_and_fresh_stamp(self):
        frame = np.full((480,640,3),[10,20,30],np.uint8)
        message = fill_image(SimpleNamespace(),frame,header())
        image, metadata = raw_measurement_frame(message,self.camera,self.mount,10_100_000_000)
        np.testing.assert_array_equal(image,frame)
        self.assertEqual(metadata["source_age_ms"],100)
        self.assertEqual(metadata["stamp"],dict(sec=10,nanosec=0))
        with self.assertRaisesRegex(ValueError,"stale"):
            raw_measurement_frame(message,self.camera,self.mount,11_000_000_000)
        message.header.frame_id="other_camera"
        with self.assertRaisesRegex(ValueError,"camera frame"):
            raw_measurement_frame(message,self.camera,self.mount,10_100_000_000)
        message = fill_image(SimpleNamespace(),frame[::2,::2],header())
        with self.assertRaisesRegex(ValueError,"resolution"):
            raw_measurement_frame(message,self.camera,self.mount,10_100_000_000)

    def test_six_point_live_cli_saves_report_without_changing_active_mount(self):
        from dataclasses import asdict
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            camera_file,mount_file=root/"camera.yaml",root/"mount.json"
            camera_file.write_text(yaml.safe_dump(camera_mapping()))
            mount_file.write_text(json.dumps(asdict(self.mount)))
            before=mount_file.read_bytes()
            pixels,_=ground_to_image(corridor_points(1.5,2.5,1.),self.camera,self.mount)
            frame=np.full((480,640,3),140,np.uint8)
            source=dict(frame_id=self.mount.camera_frame,stamp=dict(sec=10,nanosec=0))
            def submit(image,labels,instructions,callback):
                self.assertEqual(labels,["N-L","N-R","M-L","M-R","F-L","F-R"])
                return callback(pixels)
            with mock.patch.object(mount_check,"capture_frame",return_value=(frame,source)), \
                 mock.patch.object(mount_check,"serve_points",side_effect=submit):
                report=mount_check.main(["--capture-topic","/camera/image_raw",
                    "--camera-calibration",str(camera_file),"--camera-mount",str(mount_file),
                    "--near-x","1.5","--far-x","2.5","--corridor-width","1.0",
                    "--output-dir",str(root/"results")])
            self.assertEqual(len(report["width_checks"]),3)
            self.assertEqual(report["source_header"],source)
            self.assertEqual(mount_file.read_bytes(),before)
            self.assertTrue((Path(report["saved_to"])/"raw.png").is_file())
            self.assertTrue((Path(report["saved_to"])/"report.json").is_file())
