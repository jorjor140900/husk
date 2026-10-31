import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import cv2
import numpy as np

from husky_drivable_area.charuco import CharucoSession
from husky_drivable_area.transport import LatestInput, fill_image
from test_ros_adapter import header
from test_ground_geometry import camera_mapping


class CharucoNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        modules = {
            "rclpy": SimpleNamespace(), "rclpy.node": SimpleNamespace(Node=object),
            "rclpy.executors": SimpleNamespace(ExternalShutdownException=RuntimeError),
            "rclpy.qos": SimpleNamespace(QoSProfile=object, ReliabilityPolicy=object),
            "sensor_msgs": SimpleNamespace(),
            "sensor_msgs.msg": SimpleNamespace(Image=SimpleNamespace, CompressedImage=SimpleNamespace),
            "std_srvs": SimpleNamespace(), "std_srvs.srv": SimpleNamespace(Trigger=object),
        }
        path = Path(__file__).resolve().parents[1]/"husky_drivable_area/charuco_node.py"
        spec = importlib.util.spec_from_file_location("husky_drivable_area._charuco_test", path)
        cls.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict("sys.modules", modules):
            spec.loader.exec_module(cls.module)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.node = object.__new__(self.module.CharucoNode)
        self.node.session, self.node.latest = CharucoSession(), LatestInput()
        self.node.expected, self.node.saved = (480, 640), False
        self.node.folder = Path(self.temp.name)
        self.node.last_capture = float("-inf")
        self.node.publisher, self.node.get_logger = mock.Mock(), mock.Mock()
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))

    def test_raw_view_saved_and_overlay_preserves_source_header(self):
        image = cv2.cvtColor(self.node.session.board.generateImage((640,480), marginSize=25), cv2.COLOR_GRAY2BGR)
        source_header = header()
        self.node.latest.put(fill_image(SimpleNamespace(), image, source_header))
        self.node._tick()
        self.assertEqual(len(self.node.session.views), 1)
        saved = cv2.imread(str(self.node.folder/"view-001.png"))
        np.testing.assert_array_equal(saved, image)
        observations = json.loads((self.node.folder/"observations.json").read_text())
        self.assertEqual(len(observations["views"][0]["ids"]), 24)
        output = self.node.publisher.publish.call_args.args[0]
        self.assertIs(output.header, source_header)

    def test_stale_input_is_not_used_for_calibration(self):
        frame = np.full((480,640,3), 120, np.uint8)
        self.node.latest.put(fill_image(SimpleNamespace(), frame, header(sec=1)))
        self.node._tick()
        self.assertEqual(len(self.node.session.views), 0)
        self.assertEqual(list(self.node.folder.iterdir()), [])

    def test_service_rejects_incomplete_then_saves_once_without_overwriting(self):
        result = self.node._calibrate(None, SimpleNamespace())
        self.assertFalse(result.success)
        self.assertFalse((self.node.folder/"camera.yaml").exists())
        report = dict(training_rms_px=.2, held_out_rms_px=[.3], warnings=[])
        self.node.session.calibrate = mock.Mock(return_value=(camera_mapping(), report))
        result = self.node._calibrate(None, SimpleNamespace())
        self.assertTrue(result.success)
        before = (self.node.folder/"camera.yaml").read_bytes()
        self.assertFalse(self.node._calibrate(None, SimpleNamespace()).success)
        self.assertEqual((self.node.folder/"camera.yaml").read_bytes(), before)
