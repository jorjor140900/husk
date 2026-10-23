"""Check fresh-workspace generation without a ROS install or real model weights."""

import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

from tools.create_ros2_workspace import create_workspace


class WorkspaceTests(unittest.TestCase):
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
            for name in ("setup.py", "setup.cfg", "package.xml", "launch/drivable_area.launch.py"):
                text = (package / name).read_text()
                self.assertIn("husky_vision", text)
                self.assertNotIn("husky_drivable_area", text)
            self.assertTrue((package / "road_navigation/segmentation.py").is_file())
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


if __name__ == "__main__":
    unittest.main()
