import copy
import ast
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

from husky_drivable_area.navigation import mapping_scan_geometry, navigation_parameters
from husky_drivable_area.vision_grid import encode_segmentation, decode_segmentation, project_veto_grid
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
from road_navigation import RoadDetector
from test_ground_geometry import camera_mapping, mount_mapping
from test_ros_adapter import header
from tools.create_ros2_workspace import create_workspace, update_workspace_code

ROOT = Path(__file__).resolve().parents[1]


class MappingGeometryTests(unittest.TestCase):
    def test_real_legacy_vlp_scan_endpoints_corrected_without_relabeling_timing_or_no_returns(self):
        scan = SimpleNamespace(angle_min=-np.pi,angle_max=np.pi,angle_increment=.007,
            range_min=0.,range_max=200.,time_increment=0.,scan_time=0.,ranges=[2.]*896+[np.inf])
        before = copy.deepcopy(scan)
        values = mapping_scan_geometry(scan)
        self.assertAlmostEqual(values["angle_max"],-np.pi+896*.007)
        self.assertGreater(values["range_min"],0)
        self.assertEqual(scan.ranges,before.ranges)
        self.assertEqual(scan.time_increment,0)

    def test_bad_scan_rejected(self):
        scan = SimpleNamespace(angle_min=-np.pi,angle_max=np.pi,angle_increment=.007,
            range_min=0.,range_max=200.,time_increment=0.,scan_time=0.,ranges=[2.]*897)
        for value in (np.nan,-np.inf,-1.):
            scan.ranges[0] = value
            with self.assertRaises(ValueError):
                mapping_scan_geometry(scan)


class VisionGridTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration.from_mapping(camera_mapping())
        self.mount = CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,
            camera_x_m=-.26,camera_y_m=.21))

    def test_packet_preserves_stamp_and_marks_unanalyzed_horizon_unknown(self):
        frame = np.full((240,320,3),120,np.uint8)
        result = RoadDetector().process(frame,drivable_probability=np.ones((240,320)))
        packet, labels = decode_segmentation(encode_segmentation(result,header(),.3))
        self.assertEqual(packet["source_header"]["stamp"]["sec"],10)
        self.assertEqual(packet["raw_shape_hw"],[240,320])
        self.assertTrue((labels[:72] == 255).all())
        self.assertTrue((labels[200:] == 1).all())

    def test_packet_rejects_bad_dimensions_and_labels(self):
        for packet in ("[]",json.dumps(dict(protocol=1,valid=False)),"x"*2_000_001):
            with self.assertRaises((ValueError,KeyError)):
                decode_segmentation(packet)

    def test_ai_clear_and_unknown_never_become_free_space_in_map(self):
        for label in (1,255):
            costs, origin, detail = project_veto_grid(np.full((240,320),label,np.uint8),
                [480,640],self.camera,self.mount,np.eye(4))
            self.assertTrue((costs == -1).all())
            self.assertGreater(detail["observed_drivable_cells"],0) if label==1 else self.assertEqual(detail["observed_drivable_cells"],0)
        costs, _, detail = project_veto_grid(np.zeros((240,320),np.uint8),
            [480,640],self.camera,self.mount,np.eye(4))
        self.assertGreater(detail["blocked_cells"],0)
        self.assertTrue(set(np.unique(costs)).issubset({-1,100}))

    def test_captured_pose_changes_grid_origin_and_nonplanar_tf_rejected(self):
        pose = np.eye(4);pose[:2,3] = [10.,-2.]
        _, origin, _ = project_veto_grid(np.ones((240,320),np.uint8),[480,640],self.camera,self.mount,pose)
        np.testing.assert_allclose(origin,[7.,-5.])
        pose[2,2] = .5
        with self.assertRaises(ValueError):
            project_veto_grid(np.ones((240,320),np.uint8),[480,640],self.camera,self.mount,pose)


class NavigationConfigTests(unittest.TestCase):
    def test_measured_hull_ai_before_inflation_and_private_native_controller(self):
        profile = dict(robot=dict(base_frame="base_link",odom_frame="odom"),
            footprint=dict(front_m=.5,rear_m=.56,half_width_m=.33,margin_m=.15))
        template = yaml.safe_load((ROOT/"config/navigation.yaml").read_text())
        data = navigation_parameters(profile,template,"/tmp/site.yaml","/tmp/tree.xml")
        for name in ("global_costmap","local_costmap"):
            params = data[name][name]["ros__parameters"]
            self.assertLess(params["plugins"].index("vision_layer"),params["plugins"].index("inflation_layer"))
            self.assertIn("0.65",params["footprint"])
        self.assertFalse(data["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"])
        self.assertTrue(data["controller_server"]["ros__parameters"]["enable_stamped_cmd_vel"])
        self.assertLessEqual(data["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"],.05)
        self.assertTrue(data["controller_server"]["ros__parameters"]["FollowPath"]["use_collision_detection"])
        ET.parse(ROOT/"config/navigate_to_pose.xml")

    def test_navigation_plugin_export_is_opt_in_and_backs_up_edits(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);model = root/"model";model.mkdir()
            for name in ("config.json","preprocessor_config.json"):
                (model/name).write_text("{}")
            (model/"model.safetensors").write_bytes(b"fixture")
            workspace = root/"ws"
            create_workspace(workspace,model)
            self.assertFalse((workspace/"src/husky_vision_nav2").exists())
            update_workspace_code(workspace,with_navigation=True)
            target = workspace/"src/husky_vision_nav2/src/vision_mask_layer.cpp"
            self.assertTrue(target.is_file())
            target.write_text("local edits")
            result = update_workspace_code(workspace,with_navigation=True)
            self.assertEqual((Path(result["navigation_plugin"]["source_backup"])/"src/vision_mask_layer.cpp").read_text(),"local edits")

    def test_lateral_inflation_extent_is_separate_from_hull_and_ai_margin(self):
        hull=dict(front_m=.5,rear_m=.56,half_width_m=.33,margin_m=.15,lateral_margin_m=.08)
        profile=dict(robot=dict(base_frame="base_link",odom_frame="odom"),footprint=hull,
                     navigation=dict(inflation_side_clearance_m=.15))
        before=copy.deepcopy(profile)
        template=yaml.safe_load((ROOT/"config/navigation.yaml").read_text())
        data=navigation_parameters(profile,template,"/tmp/site.yaml","/tmp/tree.xml")
        self.assertEqual(profile,before)
        for name in ('global_costmap','local_costmap'):
            parameters=data[name][name]['ros__parameters']
            radius=parameters['inflation_layer']['inflation_radius']
            self.assertAlmostEqual(radius,.48)
            self.assertAlmostEqual(radius-hull['half_width_m'],.15)
            corners=np.array(ast.literal_eval(parameters['footprint']))
            np.testing.assert_allclose(corners,[[.65,.41],[.65,-.41],[-.71,-.41],[-.71,.41]])
        for value in (0,-.1,True,None,float('nan'),float('inf'),.01):
            profile['navigation']['inflation_side_clearance_m']=value
            with self.assertRaises(ValueError):navigation_parameters(profile,template,'/tmp/site.yaml','/tmp/tree.xml')

    def test_live_guide_obstacles_remain_default_and_unknown_source_is_rejected(self):
        from husky_drivable_area.navigation import guide_obstacle_source
        self.assertEqual(guide_obstacle_source({},True,False),'live_scan')
        self.assertEqual(guide_obstacle_source({},False,False),'live_scan')
        for value in ('none','false',None):
            with self.assertRaises(ValueError):guide_obstacle_source(dict(navigation=dict(guide_obstacle_source=value)),True,True)

    def test_side_margin_changes_only_width_and_matches_collision_filter(self):
        from husky_drivable_area.collision import CollisionConfig
        footprint=dict(front_m=.5,rear_m=.56,half_width_m=.33,margin_m=.15,lateral_margin_m=.08)
        profile=dict(robot=dict(base_frame="base_link",odom_frame="odom"),footprint=footprint)
        data=navigation_parameters(profile,yaml.safe_load((ROOT/"config/navigation.yaml").read_text()),"/tmp/site.yaml","/tmp/tree.xml")
        config=CollisionConfig(**footprint)
        for name in ("global_costmap","local_costmap"):
            params=data[name][name]["ros__parameters"]
            corners=np.array(ast.literal_eval(params["footprint"]))
            np.testing.assert_allclose(corners,[[.65,.41],[.65,-.41],[-.71,-.41],[-.71,.41]])
            signs=np.sign(corners)
            np.testing.assert_allclose(config.padded_corners(),corners+signs*config.sweep_step_m)
            self.assertAlmostEqual(params["inflation_layer"]["inflation_radius"],.81)
        for value in (0,-.1,True,None,float("nan")):
            footprint["lateral_margin_m"]=value
            with self.assertRaises(ValueError):navigation_parameters(profile,data,"/tmp/site.yaml","/tmp/tree.xml")
