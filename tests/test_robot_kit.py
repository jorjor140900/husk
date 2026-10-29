"""Field bundle export preserves models/bench calibration and pending values."""
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from husky_drivable_area.robot_kit import create_robot_kit
from husky_drivable_area.robot_profile import load_robot_profile
from tools.create_ros2_workspace import create_workspace, update_workspace_code


class RobotKitTests(unittest.TestCase):
    def workspace(self, root):
        model = root / "model"
        model.mkdir()
        for name, value in (("config.json", "{}"), ("preprocessor_config.json", "{}"), ("model.safetensors", "fixture")):
            (model / name).write_text(value)
        destination = root / "ws"
        create_workspace(destination, model)
        return destination

    def test_exported_ros1_package_is_standalone_and_not_a_dangling_source_link(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = self.workspace(Path(temporary))
            root = workspace / "src/husky_vision"
            core = root / "ros1/husky_vision_bridge/src/husky_link_core/protocol.py"
            self.assertTrue(core.is_file())
            self.assertFalse(any(parent.is_symlink() for parent in core.parents))
            self.assertEqual(core.read_bytes(), (root / "husky_link_core/protocol.py").read_bytes())
            setup = (root / "setup.py").read_text()
            self.assertIn("husky_link_core", setup)
            self.assertIn("husky_vision.robot_bridge_node:main", setup)

    def test_creation_preserves_bench_mount_model_and_existing_kit(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = self.workspace(Path(temporary))
            calibration = workspace / "camera_calibration"
            calibration.mkdir()
            (calibration / "mount.json").write_text("bench rig, keep")
            (calibration / "camera-v2.yaml").write_text("calibration, keep")
            result = create_robot_kit(workspace)
            kit = Path(result["kit"])
            profile = json.loads((kit / "robot.json").read_text())
            self.assertFalse(profile["bridge"]["forward_commands"])
            self.assertIsNone(profile["robot"]["base_frame"])
            self.assertIsNone(json.loads((kit / "robot-mount.json").read_text())["height_m"])
            self.assertTrue((kit / "ros1_catkin/src/husky_vision_bridge/src/husky_link_core/protocol.py").is_file())
            self.assertTrue((kit / "ros1_catkin/src/husky_vision_bridge/scripts/robot_gateway.py").stat().st_mode & 0o111)
            self.assertEqual((calibration / "mount.json").read_text(), "bench rig, keep")
            self.assertEqual((workspace / "models/husky-segformer/model.safetensors").read_text(), "fixture")
            with self.assertRaises(FileExistsError):
                create_robot_kit(workspace)
            (kit / "robot.json").write_text("user measured profile")
            update_workspace_code(workspace)
            self.assertEqual((kit / "robot.json").read_text(), "user measured profile")

    def test_incomplete_profile_reports_measured_fields_instead_of_guessing(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = self.workspace(Path(temporary))
            result = create_robot_kit(workspace)
            with self.assertRaisesRegex(ValueError, "odom_frame"):
                load_robot_profile(result["profile"])

    def test_completed_observe_profile_and_field_activation_requirements(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = self.workspace(Path(temporary))
            kit = create_robot_kit(workspace)
            path = Path(kit["profile"])
            profile = json.loads(path.read_text())
            calibration = workspace / "camera.yaml"
            calibration.write_text(yaml.safe_dump({"image_width": 640, "image_height": 480, "distortion_model": "plumb_bob",
                "camera_matrix": {"data": [500, 0, 320, 0, 500, 240, 0, 0, 1]}, "distortion_coefficients": {"data": [0, 0, 0, 0, 0]}}))
            mount = json.loads(Path(kit["mount"]).read_text())
            mount.update(height_m=0.6, pitch_down_deg=30.0, camera_x_m=0.0, camera_y_m=0.0)
            Path(kit["mount"]).write_text(json.dumps(mount))
            profile["camera"]["calibration_path"] = str(calibration)
            profile["robot"].update(odom_frame="odom", base_frame="base_link", base_to_ground_z_m=-0.10,
                                     odom_topic="/actual/odom", scan_topic="/actual/scan", ros1_permission_topic="/actual/permission")
            profile["footprint"].update(front_m=0.5, rear_m=0.5, half_width_m=0.35)
            path.write_text(json.dumps(profile))
            self.assertFalse(load_robot_profile(path)[0]["bridge"]["forward_commands"])
            profile["footprint"]["lateral_margin_m"]=.08
            path.write_text(json.dumps(profile))
            loaded=load_robot_profile(path)[0]["footprint"]
            self.assertEqual(loaded["margin_m"],.15)
            self.assertEqual(loaded["lateral_margin_m"],.08)
            for value in (0,-.01,None,True,float("nan")):
                profile["footprint"]["lateral_margin_m"]=value
                path.write_text(json.dumps(profile))
                with self.assertRaisesRegex(ValueError,"lateral_margin_m"):
                    load_robot_profile(path)
            profile["footprint"]["lateral_margin_m"]=.08
            profile["bridge"].update(forward_commands=True, expected_output_topic="/verified/autonomy_input")
            path.write_text(json.dumps(profile))
            with self.assertRaisesRegex(ValueError, "pending"):
                load_robot_profile(path)
            profile["field_measurements"] = {key: True for key in profile["field_measurements"]}
            path.write_text(json.dumps(profile))
            self.assertTrue(load_robot_profile(path)[0]["bridge"]["forward_commands"])
            mount["ground_frame"] = "odom"
            Path(kit["mount"]).write_text(json.dumps(mount))
            with self.assertRaisesRegex(ValueError, "identity odom"):
                load_robot_profile(path)


if __name__ == "__main__":
    unittest.main()
