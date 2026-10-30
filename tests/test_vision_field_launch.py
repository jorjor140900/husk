"""Field launch uses the measured camera frame and only the image steering path."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock

from test_ground_geometry import mount_mapping


class FieldLaunchTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.device = root/"video7"
        self.device.touch()
        mount = root/"mount.json"
        mount.write_text(json.dumps(mount_mapping()))
        profile = dict(camera=dict(source="local_usb", model_path=str(root), mount_path=str(mount),
            calibration_path=str(root/"camera.yaml"), width=640,height=480,fps=15.))
        self.profile = root/"profile.json"
        self.profile.write_text(json.dumps(profile))
        class Configuration:
            def __init__(self, name): self.name=name
            def perform(self, context): return context[self.name]
        modules = {"ament_index_python.packages": NS(get_package_share_directory=lambda _:str(root)),
            "launch":NS(LaunchDescription=list),
            "launch.actions":NS(DeclareLaunchArgument=NS, IncludeLaunchDescription=lambda source,launch_arguments:NS(
                source=source,arguments=dict(launch_arguments)), OpaqueFunction=NS,
                SetEnvironmentVariable=lambda *args: NS(environment=args)),
            "launch.launch_description_sources":NS(PythonLaunchDescriptionSource=lambda path:path),
            "launch.substitutions":NS(LaunchConfiguration=Configuration),
            "launch_ros.actions":NS(Node=lambda **kw:NS(**kw))}
        path = Path(__file__).parents[1]/"launch/vision_field.launch.py"
        spec = importlib.util.spec_from_file_location("_field_launch_test", path)
        self.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict("sys.modules",modules): spec.loader.exec_module(self.module)
        self.context = dict(profile=str(self.profile),map="/site/map.yaml",camera_device=str(self.device),
                            allow_motion="false",supervised_trial="false",rviz="true",camera_only="false")

    def test_pipeline_has_one_camera_correct_frame_cuda_and_actual_controller_overlay_mode(self):
        nodes = self.module._nodes(self.context)
        self.assertEqual(nodes[0].environment, ("RMW_IMPLEMENTATION","rmw_cyclonedds_cpp"))
        camera = nodes[1]
        self.assertEqual(camera.executable,"webcam_node")
        self.assertEqual(camera.parameters[0]["camera_index"],7)
        self.assertEqual(camera.parameters[0]["frame_id"],"camera_optical_frame")
        perception, navigation = nodes[2:]
        self.assertEqual(perception.arguments["use_webcam"],"false")
        self.assertEqual(perception.arguments["device"],"cuda")
        self.assertEqual(perception.arguments["async_reference_preview"],"false")
        self.assertEqual(perception.arguments["max_frame_age_sec"],"0.5")
        self.assertEqual(navigation.arguments["steering_mode"],"vision")
        self.assertEqual(navigation.arguments["allow_motion"],"false")
        self.context["allow_motion"]="true"
        self.assertEqual(self.module._nodes(self.context)[3].arguments["allow_motion"],"true")

    def test_missing_or_non_video_device_cannot_silently_open_another_camera(self):
        self.device.unlink()
        with self.assertRaisesRegex(ValueError,"camera_device"):
            self.module._nodes(self.context)

    def test_camera_only_restores_legacy_preview_without_loading_navigation_or_map(self):
        self.context.update(camera_only="true",map="")
        nodes = self.module._nodes(self.context)
        self.assertEqual(len(nodes),3)
        self.assertEqual(nodes[2].arguments["async_reference_preview"],"true")
        self.assertEqual(nodes[2].arguments["reference_preview_fps"],"10.0")
        self.context["allow_motion"]="true"
        with self.assertRaisesRegex(ValueError,"allow_motion=false"):
            self.module._nodes(self.context)

    def test_navigation_cannot_silently_launch_without_map(self):
        self.context["map"]=""
        with self.assertRaisesRegex(ValueError,"navigation requires map"):
            self.module._nodes(self.context)
