"""A vegetation veto must survive model confidence, history and Nav2 encoding."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import cv2
import numpy as np

from road_navigation.detector import DetectorConfig, RoadDetector
from road_navigation.vegetation import green_vegetation_mask
from husky_drivable_area.vision_grid import encode_segmentation, decode_segmentation, project_veto_grid
from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount, ground_to_image
from tools.compare_vegetation_veto import compare
from test_ground_geometry import camera_mapping, mount_mapping
from tools.create_ros2_workspace import create_workspace, update_workspace_code


class VegetationTests(unittest.TestCase):
    def scene(self, mirrored=False):
        frame = np.full((240,320,3),140,np.uint8)
        frame[:,:80] = (30,120,40)
        frame[:,240:] = (50,95,55)  # a less saturated grass sample
        return frame[:,::-1].copy() if mirrored else frame

    def test_colour_filter_is_symmetric_and_preserves_neutral_paving(self):
        for brightness in (.5,1.,1.5):
            frame = np.clip(self.scene()*brightness,0,255).astype(np.uint8)
            blocked = green_vegetation_mask(frame)
            self.assertTrue((blocked[:,10:70]>0).all())
            self.assertTrue((blocked[:,250:310]>0).all())
            self.assertFalse(blocked[:,100:220].any())
            np.testing.assert_array_equal(green_vegetation_mask(frame[:,::-1].copy()),blocked[:,::-1])

    def test_ai_cannot_override_veto_in_raw_road_clearance_or_path_masks(self):
        frame = self.scene()
        result = RoadDetector(DetectorConfig(reject_green_vegetation=True)).process(frame,
            drivable_probability=np.ones(frame.shape[:2]))
        self.assertTrue(result.valid_image_path,result.reason)
        for mask in (result.raw_mask,result.road_mask,result.navigable_mask):
            self.assertFalse(mask[:,10:70].any())
            self.assertFalse(mask[:,250:310].any())
        self.assertTrue((result.path[:,0]>80).all())
        self.assertTrue((result.path[:,0]<240).all())
        self.assertTrue(result.metadata()["vegetation_veto"]["enabled"])

    def test_current_green_ground_cannot_be_restored_by_temporal_history(self):
        detector = RoadDetector(DetectorConfig(reject_green_vegetation=True,temporal_alpha=.1))
        frame = np.full((240,320,3),140,np.uint8)
        probability = np.ones(frame.shape[:2])
        detector.process(frame,drivable_probability=probability)
        frame[:] = (30,120,40)
        result = detector.process(frame,drivable_probability=probability)
        self.assertFalse(result.raw_mask.any())
        self.assertFalse(result.valid_image_path)

    def test_explicit_obstacles_still_veto_neutral_paving_and_default_is_unchanged(self):
        frame = self.scene();probability = np.ones(frame.shape[:2])
        legacy = RoadDetector().process(frame,drivable_probability=probability)
        self.assertTrue(legacy.raw_mask[150,40])
        obstacle = np.zeros(frame.shape[:2],np.uint8);obstacle[160:170,100:220] = 255
        result = RoadDetector(DetectorConfig(reject_green_vegetation=True)).process(frame,
            drivable_probability=probability,blocked_mask=obstacle)
        self.assertFalse(result.raw_mask[160:170,100:220].any())

    def test_veto_reaches_segmentation_packet_and_tiny_original_pixels_survive_downsampling(self):
        frame = np.full((480,640,3),140,np.uint8);frame[:,30:150] = (30,120,40)
        frame[400,320] = (30,120,40)
        result = RoadDetector(DetectorConfig(reject_green_vegetation=True,max_width=320,vegetation_dilation_px=0)).process(
            frame,drivable_probability=np.ones(frame.shape[:2]))
        self.assertEqual(result.raw_mask[200,160],0)
        header = NS(frame_id="camera",stamp=NS(sec=10,nanosec=123))
        packet, labels = decode_segmentation(encode_segmentation(result,header,.3))
        self.assertEqual(packet["source_header"]["stamp"]["nanosec"],123)
        self.assertTrue((labels[100:200,20:70] == 0).all())
        self.assertEqual(labels[200,160],0)
        self.assertEqual(labels[150,100],1)

    def test_dry_grass_and_green_paint_are_explicit_colour_heuristic_limits(self):
        dry = np.full((16,16,3),(70,110,145),np.uint8)
        paint = np.full((16,16,3),(30,120,40),np.uint8)
        self.assertFalse(green_vegetation_mask(dry).any())
        self.assertTrue(green_vegetation_mask(paint).all())

    def test_invalid_settings_rejected(self):
        for values in ({"reject_green_vegetation":1},{"vegetation_hue_min":100,"vegetation_hue_max":20},
                       {"vegetation_dilation_px":-1},{"vegetation_excess_green_min":float("nan")},
                       {"vegetation_support_radius_px":True},{"vegetation_support_radius_px":13},
                       {"vegetation_support_fraction":0},{"vegetation_support_fraction":True},
                       {"vegetation_support_fraction":float("nan")}):
            with self.assertRaises(ValueError):DetectorConfig(**values)

    def test_support_removes_isolated_specks_but_keeps_entire_connected_lawn(self):
        frame = self.scene()
        frame[170:172, 140:142] = (30,120,40)
        # A thin blade connected to a supported lawn must not be eroded away.
        frame[180,80:110] = (30,120,40)
        original = green_vegetation_mask(frame, dilation_px=0)
        supported = green_vegetation_mask(frame, support_radius_px=3, dilation_px=0)
        self.assertFalse(supported[170:172,140:142].any())
        expected = original.copy(); expected[170:172,140:142] = 0
        np.testing.assert_array_equal(supported, expected)
        np.testing.assert_array_equal(green_vegetation_mask(frame[:,::-1].copy(),
            support_radius_px=3,dilation_px=0), supported[:,::-1])

    def test_support_never_clears_a_model_rejection_or_explicit_obstacle(self):
        for fast in (False,True):
            frame = self.scene(); frame[170:172,140:142] = (30,120,40)
            probability = np.ones(frame.shape[:2]); probability[170,140] = 0
            obstacle = np.zeros(frame.shape[:2],np.uint8); obstacle[171,141] = 255
            result = RoadDetector(DetectorConfig(reject_green_vegetation=True,
                vegetation_support_radius_px=3,use_temporal=False,
                generate_reference_path=not fast)).process(frame,
                    drivable_probability=probability,blocked_mask=obstacle)
            self.assertFalse(result.vegetation_mask[170:172,140:142].any())
            self.assertEqual(result.raw_mask[170,140],0)
            self.assertEqual(result.raw_mask[171,141],0)
            self.assertEqual(result.raw_mask[170,141],255)
            self.assertTrue((result.raw_mask[100:,10:70] == 0).all())
            self.assertEqual(result.metadata()["vegetation_veto"]["support_radius_px"],3)
            _, labels = decode_segmentation(encode_segmentation(result,
                NS(frame_id="camera",stamp=NS(sec=1,nanosec=0)),.3))
            self.assertEqual(labels[170,140],0)
            self.assertEqual(labels[171,141],0)
            self.assertEqual(labels[170,141],1)

    def test_supported_config_and_offline_comparison_do_not_disable_lawn_veto(self):
        path=Path(__file__).resolve().parents[1]/"config/outdoor-supported-green-preview.json"
        config=DetectorConfig(**json.loads(path.read_text()))
        self.assertTrue(config.reject_green_vegetation)
        self.assertFalse(config.use_temporal)
        self.assertFalse(config.generate_reference_path)
        frame=self.scene();frame[170:172,140:142]=(30,120,40)
        old,new,_,report=compare(frame,config)
        self.assertGreater(report["removed_colour_only_pixels"],0)
        self.assertEqual(report["added_pixels"],0)
        self.assertFalse(report["ready_for_control"])
        np.testing.assert_array_equal(old[:,10:70],new[:,10:70])
        np.testing.assert_array_equal(old[:,250:310],new[:,250:310])
        json.dumps(report,allow_nan=False)

    def test_colour_speck_no_longer_marks_ground_cell_but_model_obstacle_still_does(self):
        camera=CameraCalibration.from_mapping(camera_mapping())
        mount=CameraMount.from_mapping(mount_mapping(height_m=.99,pitch_down_deg=20,
            camera_x_m=-.26,camera_y_m=.21))
        frame=np.full((480,640,3),140,np.uint8)
        frame[:,:80]=(30,120,40);frame[:,560:]=(30,120,40)
        point=np.array([[2.025,.025]])  # an actual .05 m ground-grid centre
        uv,visible=ground_to_image(point,camera,mount)
        self.assertTrue(visible.all())
        u,v=np.rint(uv[0]).astype(int)
        frame[v,u]=(30,120,40)
        grids=[]
        for radius,model_obstacle in ((0,False),(3,False),(3,True)):
            probability=np.ones(frame.shape[:2]);probability[v,u]=0 if model_obstacle else 1
            result=RoadDetector(DetectorConfig(reject_green_vegetation=True,use_temporal=False,
                generate_reference_path=False,vegetation_support_radius_px=radius)).process(
                    frame,drivable_probability=probability)
            _,labels=decode_segmentation(encode_segmentation(result,
                NS(frame_id="camera",stamp=NS(sec=1,nanosec=0)),.3))
            grid,origin,_=project_veto_grid(labels,[480,640],camera,mount,np.eye(4))
            x,y=np.floor((point[0]-origin)/.05).astype(int)
            grids.append((grid,int(grid[y,x])))
        self.assertEqual([cost for _,cost in grids],[100,-1,100])
        # Only the speck's cells change; broad green areas still veto ground.
        self.assertGreater(np.count_nonzero(grids[1][0]==100),100)
        self.assertFalse((grids[1][0]==0).any(),"vision must never clear map/LiDAR costs")

    def test_export_opt_in_backs_up_detector_without_changing_visualization(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);model=root/"model";model.mkdir()
            for name in ("config.json","preprocessor_config.json"):(model/name).write_text("{}")
            (model/"model.safetensors").write_bytes(b"fixture")
            ws=root/"ws";create_workspace(ws,model)
            detector=ws/"src/husky_vision/road_navigation/detector.py"
            visualization=ws/"src/husky_vision/road_navigation/visualization.py"
            detector.write_text("local detector");visualization.write_text("local thin lines")
            update_workspace_code(ws)
            self.assertEqual(detector.read_text(),"local detector")
            result=update_workspace_code(ws,update_detector=True)
            self.assertEqual((Path(result["source_backup"])/"road_navigation/detector.py").read_text(),"local detector")
            self.assertEqual(visualization.read_text(),"local thin lines")
            self.assertTrue((detector.parent/"vegetation.py").is_file())
