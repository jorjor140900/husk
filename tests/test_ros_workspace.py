"""Check fresh-workspace generation without a ROS install or real model weights."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

from tools.create_ros2_workspace import create_workspace, update_workspace_code


class WorkspaceTests(unittest.TestCase):
    def test_visualization_update_is_explicit_and_backed_up(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            workspace = root / "fresh"
            create_workspace(workspace, self.make_model_fixture(root))
            target = workspace / "src/husky_vision/road_navigation/visualization.py"
            target.write_text("# local drawing\n")
            update_workspace_code(workspace)
            self.assertEqual(target.read_text(), "# local drawing\n")
            result = update_workspace_code(workspace, update_visualization=True)
            backup = Path(result["source_backup"]) / "road_navigation/visualization.py"
            self.assertEqual(backup.read_text(), "# local drawing\n")
            self.assertIn("robot_axis=None", target.read_text())

    def make_model_fixture(self, folder):
        model = folder / "existing-model"
        model.mkdir()
        (model / "config.json").write_text(json.dumps({"num_labels": 2}))
        (model / "preprocessor_config.json").write_text("{}")
        # This fixture verifies copying only, not model inference.
        (model / "model.safetensors").write_bytes(b"test fixture, not a trained model")
        return model

    def test_package_is_independent_renamed_and_contains_no_old_ros_source(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model = self.make_model_fixture(root)
            destination = root / "fresh workspace"
            result = create_workspace(destination, model)
            package = Path(result["package"])
            self.assertEqual(package, destination / "src/husky_vision")
            self.assertEqual(ET.parse(package / "package.xml").findtext("name"), "husky_vision")
            self.assertFalse(any(destination.rglob("*.egg-info")))
            self.assertFalse(any(destination.rglob("legacy")))
            self.assertFalse(any(destination.rglob(".git")))
            self.assertFalse(any(path.is_symlink() for path in destination.rglob("*")))
            self.assertEqual([p.name for p in (destination / "src").iterdir()], ["husky_vision"])
            for name in ("setup.py", "setup.cfg", "package.xml", "launch/drivable_area.launch.py", "launch/ground_path.launch.py"):
                text = (package / name).read_text()
                self.assertIn("husky_vision", text)
                self.assertNotIn("husky_drivable_area", text)
            self.assertTrue((package / "road_navigation/segmentation.py").is_file())
            self.assertTrue((package / "config/checkerboard-9x6-25mm.pdf").read_bytes().startswith(b"%PDF"))
            self.assertTrue((package / "docs/ros2-metric-path.md").is_file())
            self.assertIn("husky_vision.ground_node:main", (package / "setup.py").read_text())
            self.assertIn("husky_vision.controller_node:main", (package / "setup.py").read_text())
            self.assertIn("husky_vision.control_self_test:main", (package / "setup.py").read_text())
            self.assertTrue((package / "husky_vision/control_report.py").is_file())
            self.assertTrue((package / "launch/control_demo.launch.py").is_file())
            self.assertTrue((package / "launch/lidar_demo.launch.py").is_file())
            self.assertTrue((package / "husky_vision/collision.py").is_file())
            self.assertIn("husky_vision.safety_node:main", (package / "setup.py").read_text())
            self.assertIn("husky_vision.vision_preview_node:main", (package / "setup.py").read_text())
            self.assertTrue((package / "launch/vision_preview.launch.py").is_file())
            self.assertIn("husky_vision.ground_check_node:main", (package / "setup.py").read_text())
            self.assertTrue((package / "launch/ground_check.launch.py").is_file())
            self.assertTrue((package / "husky_vision/ground_check.py").is_file())
            self.assertTrue((package / "docs/ros2-control.md").is_file())
            guide = (destination / "START_HERE.md").read_text()
            self.assertNotIn("~/husky_vision_ws", guide)
            self.assertNotIn("$HOME/husky_vision_ws", guide)
            self.assertIn(f"'{destination}/models/husky-segformer'", guide)
            copied_model = destination / "models/husky-segformer"
            self.assertEqual((copied_model / "model.safetensors").read_bytes(), (model / "model.safetensors").read_bytes())
            (model / "config.json").write_text("changed original")
            self.assertNotEqual((copied_model / "config.json").read_text(), "changed original")

    def test_existing_workspace_and_original_model_are_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model = self.make_model_fixture(root)
            destination = root / "existing-workspace"
            destination.mkdir()
            sentinel = destination / "my-code.py"
            sentinel.write_text("keep me")
            with self.assertRaises(FileExistsError):
                create_workspace(destination, model)
            self.assertEqual(sentinel.read_text(), "keep me")
            self.assertTrue((model / "model.safetensors").is_file())
            dangling = root / "old-link"
            dangling.symlink_to(root / "missing-target")
            with self.assertRaisesRegex(FileExistsError, "existing symlink"):
                create_workspace(dangling, model)
            self.assertTrue(dangling.is_symlink())
            self.assertFalse((root / "missing-target").exists())

    def test_missing_model_and_recursive_destination_do_not_create_workspace(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            destination = root / "fresh"
            with self.assertRaisesRegex(ValueError, "missing config.json"):
                create_workspace(destination, root / "missing-model")
            self.assertFalse(destination.exists())
            model = self.make_model_fixture(root)
            with self.assertRaisesRegex(ValueError, "outside the model"):
                create_workspace(model / "fresh", model)
            self.assertFalse((model / "fresh").exists())

    def test_copy_failure_cleans_staging_and_leaves_no_partial_workspace(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model = self.make_model_fixture(root)
            destination = root / "fresh"
            with mock.patch("tools.create_ros2_workspace.shutil.copytree", side_effect=OSError("disk full")):
                with self.assertRaisesRegex(OSError, "disk full"):
                    create_workspace(destination, model)
            self.assertFalse(destination.exists())
            self.assertEqual(list(root.iterdir()), [model])

    def test_update_backs_up_local_edits_and_preserves_model_and_config(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model = self.make_model_fixture(root)
            workspace = root / "fresh"
            create_workspace(workspace, model)
            package = workspace / "src/husky_vision"
            target = package / "husky_vision/nodes.py"
            target.write_text("# local ROS edits\n")
            config = package / "config.example.json"
            config.write_text('{"clearance_px": 12}')
            core = package / "road_navigation/detector.py"
            core.write_text("# local detector edits\n")
            result = update_workspace_code(workspace)
            self.assertEqual((Path(result["source_backup"]) / "husky_vision/nodes.py").read_text(), "# local ROS edits\n")
            self.assertIn("class DrivableAreaNode", target.read_text())
            self.assertIn('"cpu_threads"', target.read_text())
            self.assertNotIn("husky_drivable_area", (package / "launch/drivable_area.launch.py").read_text())
            self.assertEqual(config.read_text(), '{"clearance_px": 12}')
            self.assertEqual(core.read_text(), "# local detector edits\n")
            self.assertEqual((workspace / "models/husky-segformer/model.safetensors").read_bytes(), (model / "model.safetensors").read_bytes())

    def test_upgrade_adds_projection_to_an_older_export(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            workspace = root / "fresh"
            create_workspace(workspace, self.make_model_fixture(root))
            package = workspace / "src/husky_vision"
            added = ["husky_vision/ground_node.py", "husky_vision/ground_geometry.py", "husky_vision/calibration.py",
                     "launch/ground_path.launch.py", "config/ground_path.rviz", "config/camera_mount.example.json",
                     "config/checkerboard-9x6-25mm.pdf", "docs/ros2-metric-path.md"]
            for relative in added:
                (package / relative).unlink()
            (package / "setup.py").write_text("# old package metadata with local edits\n")
            result = update_workspace_code(workspace)
            self.assertTrue(all((package / relative).is_file() for relative in added))
            backup = Path(result["source_backup"])
            self.assertEqual((backup / "setup.py").read_text(), "# old package metadata with local edits\n")
            self.assertEqual(set(json.loads((backup / "created_files.json").read_text())), set(added))
            self.assertIn("ground_path_node", (package / "setup.py").read_text())

    def test_upgrade_adds_control_and_preserves_user_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            workspace = root / "fresh"
            create_workspace(workspace, self.make_model_fixture(root))
            package = workspace / "src/husky_vision"
            added = ["husky_vision/control.py", "husky_vision/controller_node.py", "husky_vision/control_simulator.py",
                     "launch/control_demo.launch.py", "launch/controller.launch.py", "config/controller.yaml",
                     "config/control_demo.rviz", "docs/ros2-control.md"]
            for relative in added:
                (package / relative).unlink()
            calibration = workspace / "camera_calibration"
            calibration.mkdir()
            (calibration / "camera-v2.yaml").write_text("user's calibrated intrinsics")
            (calibration / "mount.json").write_text("user's measured camera mount")
            result = update_workspace_code(workspace)
            self.assertEqual(set(json.loads((Path(result["source_backup"]) / "created_files.json").read_text())), set(added))
            self.assertTrue(all((package / relative).is_file() for relative in added))
            self.assertEqual((calibration / "camera-v2.yaml").read_text(), "user's calibrated intrinsics")
            self.assertEqual((calibration / "mount.json").read_text(), "user's measured camera mount")

    def test_update_rejects_an_unrelated_or_incomplete_workspace_before_writing(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaisesRegex(ValueError, "expected an existing"):
                update_workspace_code(root / "missing")
            self.assertFalse((root / "missing").exists())
            model = self.make_model_fixture(root)
            workspace = root / "fresh"
            create_workspace(workspace, model)
            package = workspace / "src/husky_vision"
            (package / "husky_vision/transport.py").unlink()
            original = (package / "husky_vision/nodes.py").read_bytes()
            with self.assertRaisesRegex(ValueError, "expected a regular"):
                update_workspace_code(workspace)
            self.assertEqual((package / "husky_vision/nodes.py").read_bytes(), original)
            self.assertFalse((workspace / ".source-backups").exists())


if __name__ == "__main__":
    unittest.main()
