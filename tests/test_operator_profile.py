"""No-joystick preparation preserves measured geometry and pending checks."""
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from husky_drivable_area.operator_profile import prepare_operator_profile
from husky_drivable_area.robot_profile import load_robot_profile
from husky_link_core.operator_permission import PERMISSION_TOPIC
from test_ground_geometry import camera_mapping, mount_mapping


class OperatorProfileTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root/"robot.json"
        self.output = self.root/"nojoy.json"
        self.robot_output = self.root/"ros1.json"
        self.profile = json.loads((Path(__file__).parents[1]/"config/robot.example.json").read_text())
        self.profile["robot"].update(joy_topic="/joy_teleop/joy", joy_deadman_button=0,
            base_frame="base_link", odom_frame="odom", base_to_ground_z_m=-.13228,
            odom_topic="/odometry/filtered", scan_topic="/scan")
        self.profile["footprint"].update(front_m=.5, rear_m=.56, half_width_m=.33,
                                         margin_m=.15, lateral_margin_m=.08)
        camera, mount = self.root/"camera.yaml", self.root/"mount.json"
        camera.write_text(yaml.safe_dump(camera_mapping()))
        mount.write_text(json.dumps(mount_mapping()))
        self.profile["camera"].update(calibration_path=str(camera), mount_path=str(mount), model_path=str(self.root))
        self.source.write_text(json.dumps(self.profile))

    def prepare(self, motion=False):
        return prepare_operator_profile(self.source, self.output, self.robot_output, motion)

    def test_preview_preserves_pending_checks_geometry_and_original_files(self):
        before = self.source.read_bytes()
        self.prepare()
        actual = json.loads(self.output.read_text())
        self.assertEqual(actual["footprint"], self.profile["footprint"])
        self.assertEqual(actual["camera"], self.profile["camera"])
        self.assertEqual(actual["field_measurements"], self.profile["field_measurements"])
        self.assertEqual(self.source.read_bytes(), before)
        self.assertIsNone(actual["robot"]["joy_topic"])
        self.assertIsNone(actual["robot"]["joy_deadman_button"])
        self.assertEqual(actual["robot"]["ros1_permission_topic"], PERMISSION_TOPIC)
        self.assertFalse(actual["bridge"]["forward_commands"])
        self.assertEqual(actual["bridge"]["expected_output_topic"], "/cmd_vel")
        self.assertEqual(load_robot_profile(self.output)[0]["footprint"]["lateral_margin_m"], .08)
        minimal = json.loads(self.robot_output.read_text())
        self.assertNotIn("camera", minimal)
        self.assertEqual(actual["robot"], minimal["robot"])

    def test_motion_rejected_without_creating_files_when_field_checks_are_pending(self):
        with self.assertRaisesRegex(ValueError, "pending"):
            self.prepare(True)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.robot_output.exists())

    def test_motion_requires_valid_camera_files_even_with_checks_recorded(self):
        self.profile["field_measurements"] = {key: True for key in self.profile["field_measurements"]}
        self.profile["robot"].update(ros1_permission_topic=PERMISSION_TOPIC, joy_topic=None, joy_deadman_button=None)
        self.profile["bridge"]["expected_output_topic"] = "/cmd_vel"
        self.profile["camera"]["calibration_path"] = str(self.root/"missing.yaml")
        self.source.write_text(json.dumps(self.profile))
        with self.assertRaisesRegex(ValueError, "calibration_path"):
            self.prepare(True)
        self.assertFalse(self.output.exists())

    def test_completed_profile_makes_explicit_motion_variant_without_altering_checks(self):
        self.prepare()
        self.source = self.output
        self.profile = json.loads(self.source.read_text())
        self.profile["field_measurements"] = {key: True for key in self.profile["field_measurements"]}
        self.source.write_text(json.dumps(self.profile))
        self.output, self.robot_output = self.root/"motion.json", self.root/"ros1-motion.json"
        self.prepare(True)
        loaded, _ = load_robot_profile(self.output)
        self.assertTrue(loaded["bridge"]["forward_commands"])
        self.assertEqual(loaded["field_measurements"], self.profile["field_measurements"])
        self.assertFalse(json.loads(self.source.read_text())["bridge"]["forward_commands"])

    def test_joystick_checks_cannot_be_reused_for_console_motion(self):
        self.profile["field_measurements"] = {key: True for key in self.profile["field_measurements"]}
        self.source.write_text(json.dumps(self.profile))
        with self.assertRaisesRegex(ValueError, "pending"):
            self.prepare(True)
        report = self.prepare()
        self.assertIn("independent_permission_gate_checked", report["invalidated_checks"])
        self.assertFalse(report["field_measurements"]["independent_permission_gate_checked"])
        self.assertTrue(report["field_measurements"]["geometry_checked"])

    def test_never_overwrites_original_or_either_output(self):
        self.output.write_text("local edits")
        with self.assertRaisesRegex(ValueError, "NEW"):
            self.prepare()
        self.assertEqual(self.output.read_text(), "local edits")
        self.assertFalse(self.robot_output.exists())
        with self.assertRaises(ValueError):
            prepare_operator_profile(self.source, self.robot_output, self.robot_output)

    def test_supervised_profile_preserves_pending_checks_and_requires_explicit_mode(self):
        from husky_link_core.operator_permission import operator_settings
        prepare_operator_profile(self.source,self.output,self.robot_output,supervised_trial=True)
        value=json.loads(self.output.read_text())
        self.assertEqual(value["operation_mode"],"supervised_vision_trial")
        self.assertFalse(value["field_measurements"]["geometry_checked"])
        self.assertFalse(value["field_measurements"]["independent_permission_gate_checked"])
        with self.assertRaisesRegex(ValueError,"must match"):load_robot_profile(self.output)
        self.assertTrue(load_robot_profile(self.output,allow_supervised_trial=True)[0]["bridge"]["forward_commands"])
        with self.assertRaisesRegex(ValueError,"must match"):operator_settings(value,True)
        self.assertEqual(operator_settings(value,True,True)[1],"/cmd_vel")
