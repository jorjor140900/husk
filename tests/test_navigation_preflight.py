"""Preparation without a map must not be mistaken for live/physical readiness."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import cv2
import numpy as np
import yaml

from husky_drivable_area.navigation import saved_map_metadata
from husky_drivable_area.navigation_preflight import preparation_checks, evaluate_live, main, check_native_library
from test_ground_geometry import camera_mapping, mount_mapping

ROOT = Path(__file__).resolve().parents[1]


class NavigationPreparationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root/"camera.yaml").write_text(yaml.safe_dump(camera_mapping()))
        (self.root/"mount.json").write_text(json.dumps(mount_mapping()))
        self.profile = dict(camera=dict(calibration_path=str(self.root/"camera.yaml"),
            mount_path=str(self.root/"mount.json"), width=640, height=480),
            robot=dict(odom_frame="odom", base_frame="base_link",external_permission_topic="/husky_link/external_permission"),
            bridge=dict(url="ws://127.0.0.1:9090",forward_commands=False),
            footprint=dict(front_m=.5,rear_m=.56,half_width_m=.33,margin_m=.15))
        self.profile_file = self.root/"robot.json"
        self.profile_file.write_text(json.dumps(self.profile))
        (self.root/"lib").mkdir()
        (self.root/"lib/libvision_mask_layer.so").write_bytes(b"package-presence fixture only")
        self.map_file = self.root/"site.yaml"

    def prepare(self):
        return preparation_checks(self.profile_file,self.map_file,package_lookup=lambda _:self.root,root=ROOT,
                                  native_probe=lambda _: "synthetic native-probe fixture",
                                  typesupport_probe=lambda *args: "synthetic transport-probe fixture")

    def test_typesupport_fastcdr_failure_cannot_hide_behind_successful_manager_check(self):
        def broken(prefix, library):
            if "geographic_msgs" in library:
                raise ValueError("undefined symbol: eprosima_fastcdr_Cdr_serialize")
            return "resolves"
        with mock.patch.dict("os.environ",{"RMW_IMPLEMENTATION":"rmw_fastrtps_cpp"}):
            _, _, checks, setup = preparation_checks(self.profile_file,self.map_file,
                package_lookup=lambda _:self.root, root=ROOT,native_probe=lambda _:"manager resolves",
                typesupport_probe=broken)
        self.assertTrue(checks["nav2_native_runtime"]["ok"])
        self.assertTrue(checks["nav2_msgs_transport_cpp"]["ok"])
        self.assertFalse(checks["geographic_msgs_transport_c"]["ok"])
        self.assertFalse(checks["geographic_msgs_transport_cpp"]["ok"])
        self.assertFalse(all(checks[name]["ok"] for name in setup))

    def test_transport_probe_uses_selected_cyclone_introspection_without_changing_rmw(self):
        probe = mock.Mock(return_value="introspection resolves")
        with mock.patch.dict("os.environ",{"RMW_IMPLEMENTATION":"rmw_cyclonedds_cpp"}):
            preparation_checks(self.profile_file,self.map_file,package_lookup=lambda _:self.root,
                root=ROOT,native_probe=lambda _:"manager resolves",typesupport_probe=probe)
        self.assertEqual(probe.call_count,4)
        self.assertTrue(all("__rosidl_typesupport_introspection_" in call.args[1] for call in probe.call_args_list))

    def test_native_check_rejects_undefined_symbols_even_when_ldd_returns_zero(self):
        (self.root/"lib/libnav2_lifecycle_manager_core.so").touch()
        with mock.patch("husky_drivable_area.navigation_preflight.subprocess.run",return_value=NS(
                returncode=0,stdout="undefined symbol: diagnostic_updater_Updater (libnav2.so)",stderr="")):
            with self.assertRaisesRegex(ValueError,"ABI error"):
                check_native_library(self.root)

    def test_native_check_rejects_missing_dependency_and_reports_resolved_path(self):
        (self.root/"lib/libnav2_lifecycle_manager_core.so").touch()
        with mock.patch("husky_drivable_area.navigation_preflight.subprocess.run",return_value=NS(
                returncode=0,stdout="libdiagnostic_updater.so => not found",stderr="")):
            with self.assertRaises(ValueError):check_native_library(self.root)
        with mock.patch("husky_drivable_area.navigation_preflight.subprocess.run",return_value=NS(
                returncode=0,stdout="libdiagnostic_updater.so => /opt/ros/jazzy/lib/libdiagnostic_updater.so",stderr="")):
            self.assertIn("/opt/ros/jazzy",check_native_library(self.root))

    def write_map(self):
        image = np.full((10,20),254,np.uint8)
        cv2.imwrite(str(self.root/"site.pgm"),image)
        data = dict(image="site.pgm",resolution=.05,origin=[-1.,-2.,0.],negate=0,free_thresh=.196,occupied_thresh=.65)
        self.map_file.write_text(yaml.safe_dump(data))
        return data

    def test_no_map_is_pending_but_does_not_block_offline_preparation(self):
        _, metadata, checks, setup = self.prepare()
        self.assertTrue(all(checks[key]["ok"] for key in setup))
        self.assertIsNone(metadata)
        self.assertFalse(checks["saved_map"]["ok"])
        output = self.root/"reports"
        with mock.patch("husky_drivable_area.navigation_preflight.preparation_checks",return_value=self.prepare()):
            result = main(["--profile",str(self.profile_file),"--output-dir",str(output)])
        self.assertEqual(result,0)
        report = json.loads(next(output.glob("*.json")).read_text())
        self.assertTrue(report["software_prepared"])
        self.assertFalse(report["planning_inputs_ready"])
        self.assertFalse(report["ready_for_autonomous_control"])

    def test_missing_compiled_plugin_and_camera_size_mismatch_are_visible(self):
        (self.root/"lib/libvision_mask_layer.so").unlink()
        self.profile["camera"]["width"] = 320
        self.profile_file.write_text(json.dumps(self.profile))
        _, _, checks, _ = self.prepare()
        self.assertFalse(checks["package_husky_vision_nav2"]["ok"])
        self.assertFalse(checks["camera_configuration"]["ok"])

    def test_uninstalled_packages_and_incomplete_footprint_do_not_pass(self):
        self.profile["footprint"]["front_m"] = None
        self.profile_file.write_text(json.dumps(self.profile))
        def absent(_):
            raise LookupError("package not found")
        _, _, checks, _ = preparation_checks(self.profile_file,self.map_file,package_lookup=absent,root=ROOT)
        self.assertFalse(checks["navigation_configuration"]["ok"])
        self.assertFalse(checks["package_nav2_amcl"]["ok"])

    def test_invalid_tf_names_and_self_referencing_permission_do_not_pass_configuration(self):
        for key,value in (("base_frame",123),("base_frame","bad frame"),
                          ("external_permission_topic","/husky_control/motion_permitted")):
            changed = copy.deepcopy(self.profile)
            changed["robot"][key] = value
            self.profile_file.write_text(json.dumps(changed))
            self.assertFalse(self.prepare()[2]["navigation_configuration"]["ok"])

    def test_saved_map_resolves_relative_image_and_reports_original_origin(self):
        self.write_map()
        result = saved_map_metadata(self.map_file)
        self.assertEqual(result["image"],str(self.root/"site.pgm"))
        self.assertEqual((result["width"],result["height"]),(20,10))
        self.assertEqual(result["origin"],[-1.,-2.,0.])

    def test_missing_image_and_invalid_coordinate_metadata_rejected(self):
        data = self.write_map()
        (self.root/"site.pgm").unlink()
        with self.assertRaises(ValueError):saved_map_metadata(self.map_file)
        for key,value in (("resolution",0),("origin",[float("nan"),0,0]),("occupied_thresh",.1)):
            bad = {**data,key:value}
            self.map_file.write_text(yaml.safe_dump(bad))
            with self.assertRaises(ValueError):saved_map_metadata(self.map_file)


class LiveNavigationReadinessTests(unittest.TestCase):
    def setUp(self):
        self.profile = dict(robot=dict(base_frame="base_link",odom_frame="odom"))
        header = lambda frame:NS(frame_id=frame,stamp=NS(sec=10,nanosec=0))
        status = lambda data:NS(data=json.dumps(data))
        zero = lambda:NS(x=0.,y=0.,z=0.)
        origin = NS(position=zero(),orientation=NS(x=0.,y=0.,z=0.,w=1.))
        info = NS(width=10,height=10,resolution=.1,origin=origin)
        self.latest = {key:(value,100.) for key,value in dict(
            link=status(dict(connected=True,reason="ok",enabled=False,forward_commands=False,
                             receiver=dict(armed=False),decode_errors={})),
            odom=NS(header=header("odom"),child_frame_id="base_link"),
            scan=NS(header=header("velodyne"),ranges=[2.]*897,angle_min=-np.pi,angle_max=np.pi,
                    angle_increment=.007,range_min=0.,range_max=200.,time_increment=0.,scan_time=0.),
            vision=status(dict(valid=True,source_stamp_ns=10*10**9)),
            mask=NS(header=header("odom"),info=copy.deepcopy(info),data=[-1]*100),
            map=NS(header=header("map"),info=info,data=[0]*100),
            gate=status(dict(allow_motion=False,enabled=False,request_allowed=False,linear_x=0.,angular_z=0.)),
            guarded=NS(linear=zero(),angular=zero()),
            delivery={key:dict(unique_samples=20,p95_ms=100.,max_ms=120.) for key in ("odom","scan","vision")}
        ).items()}
        self.publishers = {topic:1 for topic in ("/map","/husky_link/odom","/husky_nav/mapping_scan",
            "/husky_nav/vision_mask","/husky_nav/guarded_request")}
        self.lifecycles = dict(map_server=3,amcl=3,planner_server=3)
        self.transforms = dict(map_to_base=True,odom_to_scan=True,odom_to_camera=True)
        self.metadata = dict(width=10,height=10,resolution=.1,origin=[0.,0.,0.])

    def evaluate(self):
        return evaluate_live(self.profile,self.latest,self.publishers,self.lifecycles,self.transforms,True,10*10**9,100.,self.metadata)

    def test_nav2_inputs_pass_without_old_yellow_reference_path_or_permission_true(self):
        self.assertTrue(all(check["ok"] for check in self.evaluate().values()))
        self.assertNotIn("path",self.latest)
        self.assertNotIn("permission",self.latest)

    def test_stale_ai_capture_fails_even_with_fresh_status_delivery(self):
        self.latest["vision"] = (NS(data=json.dumps(dict(valid=True,source_stamp_ns=9*10**9))),100.)
        self.assertFalse(self.evaluate()["live_vision"]["ok"])

    def test_static_map_stamp_is_not_expired_but_wrong_map_origin_is_rejected(self):
        self.latest["map"][0].header.stamp.sec = 0
        self.assertTrue(self.evaluate()["live_saved_map"]["ok"])
        self.latest["map"][0].info.origin.position.x = 10.
        self.assertFalse(self.evaluate()["live_saved_map"]["ok"])

    def test_duplicate_publishers_inactive_amcl_and_missing_tf_are_pending(self):
        self.publishers["/husky_link/odom"] = 2
        self.lifecycles["amcl"] = 2
        self.transforms["map_to_base"] = "localization missing"
        checks = self.evaluate()
        self.assertFalse(checks["single_publisher_/husky_link/odom"]["ok"])
        self.assertFalse(checks["lifecycle_amcl"]["ok"])
        self.assertFalse(checks["tf_map_to_base"]["ok"])

    def test_motion_enabled_gate_nonzero_request_and_bridge_forwarding_do_not_pass_preview_check(self):
        self.latest["guarded"][0].linear.x = .05
        self.assertFalse(self.evaluate()["planning_only_gate"]["ok"])
        self.latest["link"] = (NS(data=json.dumps(dict(connected=True,reason="ok",enabled=True,
            forward_commands=True,receiver=dict(armed=False)))),100.)
        self.assertFalse(self.evaluate()["live_bridge_observe_only"]["ok"])

    def test_one_lucky_fresh_frame_does_not_hide_poor_delivery_or_missing_samples(self):
        self.latest["delivery"][0]["vision"]["p95_ms"] = 900.
        self.latest["delivery"][0]["scan"]["unique_samples"] = 1
        checks = self.evaluate()
        self.assertFalse(checks["delivery_budget_vision"]["ok"])
        self.assertFalse(checks["delivery_budget_scan"]["ok"])

    def test_ai_cannot_claim_free_space_and_missing_map_stays_pending(self):
        self.latest["mask"][0].data[0] = 0
        self.metadata = None
        checks = self.evaluate()
        self.assertFalse(checks["live_vision_grid"]["ok"])
        self.assertFalse(checks["live_saved_map"]["ok"])
