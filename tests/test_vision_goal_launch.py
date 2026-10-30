"""Image steering selects one private command producer and preserves motion gates."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import cv2
import numpy as np
import yaml

from test_ground_geometry import camera_mapping,mount_mapping


class VisionLaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(__file__).resolve().parents[1]
        class Config:
            def __init__(self,name):self.name=name
            def perform(self,context):return context[self.name]
        stubs={"ament_index_python.packages":NS(get_package_share_directory=lambda _:str(cls.root)),
            "launch":NS(LaunchDescription=list),
            "launch.actions":NS(DeclareLaunchArgument=NS,OpaqueFunction=NS),
            "launch.conditions":NS(IfCondition=lambda value:value),
            "launch.substitutions":NS(LaunchConfiguration=Config),
            "launch_ros.actions":NS(Node=lambda **kwargs:NS(**kwargs))}
        spec=importlib.util.spec_from_file_location("_vision_launch_test",cls.root/"launch/navigation.launch.py")
        cls.module=importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,stubs):spec.loader.exec_module(cls.module)

    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);root=Path(temp.name)
        (root/"camera.yaml").write_text(yaml.safe_dump(camera_mapping()))
        (root/"mount.json").write_text(json.dumps(mount_mapping(height_m=.99)))
        cv2.imwrite(str(root/"map.pgm"),np.full((20,20),254,np.uint8))
        (root/"map.yaml").write_text(yaml.safe_dump(dict(image="map.pgm",resolution=.05,
            origin=[0,0,0],free_thresh=.196,occupied_thresh=.65,negate=0)))
        self.profile=dict(camera=dict(calibration_path=str(root/"camera.yaml"),mount_path=str(root/"mount.json")),
            robot=dict(base_frame="base_link",odom_frame="odom",external_permission_topic="/local_permission"),
            footprint=dict(front_m=.5,rear_m=.56,half_width_m=.33,margin_m=.15,lateral_margin_m=.08),
            bridge=dict(url="ws://127.0.0.1:9090",forward_commands=False),
            field_measurements=dict(geometry_checked=False,mux_and_base_timeout_checked=False,
                                    independent_permission_gate_checked=False))
        (root/"robot.json").write_text(json.dumps(self.profile))
        self.context=dict(profile=str(root/"robot.json"),map=str(root/"map.yaml"),allow_motion="false",supervised_trial="false",
            steering_mode="vision",ground_parent_frame="base_footprint",rviz="false",image_topic="/camera/image_raw")

    def nodes(self):
        nodes=self.module._nodes(self.context)
        generated=set()
        for node in nodes:
            for parameter in getattr(node,"parameters",[]):
                if isinstance(parameter,str) and parameter.startswith("/tmp/husky-nav2-"):
                    generated.add(parameter)
            for arg in getattr(node,"arguments",[]):
                if isinstance(arg,str) and arg.startswith("/tmp/husky-vision-rviz-"):
                    generated.add(arg)
        for path in generated:self.addCleanup(Path(path).unlink,missing_ok=True)
        return {node.executable:node for node in nodes if hasattr(node,"executable")}

    def test_vision_launch_uses_image_controller_and_map_only_as_direction_guide(self):
        nodes=self.nodes()
        self.assertIn("vision_goal_controller",nodes)
        self.assertNotIn("controller_server",nodes);self.assertNotIn("bt_navigator",nodes)
        self.assertFalse(nodes["robot_bridge"].parameters[0]["forward_commands"])
        self.assertFalse(nodes["navigation_gate"].parameters[0]["allow_motion"])
        self.assertIn(("/husky_nav/nav2_cmd_vel","/husky_nav/vision_cmd_vel"),nodes["navigation_gate"].remappings)
        self.assertTrue(nodes["goal_dispatch"].parameters[0]["guide_only"])
        self.assertEqual(nodes["vision_goal_controller"].parameters[0]["side_margin_m"],.08)
        rendered=yaml.safe_load(Path(nodes["planner_server"].parameters[0]).read_text())
        self.assertNotIn("vision_layer",rendered["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"])
        view=yaml.safe_load(Path(nodes["rviz2"].arguments[1]).read_text())
        displays={row["Name"]:row for row in view["Visualization Manager"]["Displays"]}
        self.assertFalse(displays["Goal preview plan"]["Enabled"])
        self.assertTrue(displays["Image AI controller reference"]["Enabled"])
        saved=json.loads(Path(self.context["profile"]).read_text())
        self.assertEqual(saved,self.profile)

    def test_legacy_nav2_mode_is_retained(self):
        self.context["steering_mode"]="nav2"
        nodes=self.nodes()
        self.assertNotIn("vision_goal_controller",nodes)
        self.assertFalse(nodes["goal_dispatch"].parameters[0]["guide_only"])
        rendered=yaml.safe_load(Path(nodes["planner_server"].parameters[0]).read_text())
        self.assertIn("vision_layer",rendered["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"])

    def test_trial_wiring_is_explicit_and_does_not_activate_for_normal_profiles(self):
        from husky_link_core.supervised_trial import MODE,LIMITS
        self.context["supervised_trial"]="true"
        with self.assertRaisesRegex(ValueError,"must match"):self.nodes()
        self.profile.update(operation_mode=MODE,trial_limits=LIMITS)
        Path(self.context["profile"]).write_text(json.dumps(self.profile))
        nodes=self.nodes()
        self.assertTrue(nodes["laser_safety"].parameters[-1]["supervised_trial"])
        self.assertTrue(nodes["navigation_gate"].parameters[0]["supervised_trial"])
        self.assertTrue(nodes["vision_goal_controller"].parameters[0]["supervised_trial"])
        self.context["steering_mode"]="nav2"
        with self.assertRaisesRegex(ValueError,"requires vision"):self.nodes()

    def test_motion_mode_still_rejects_unverified_robot_profile(self):
        self.context["allow_motion"]="true"
        with self.assertRaisesRegex(ValueError,"motion mode requires"):
            self.module._nodes(self.context)


if __name__=="__main__":
    unittest.main()
