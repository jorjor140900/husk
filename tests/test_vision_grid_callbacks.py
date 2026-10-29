"""A slightly late capture TF is retried, never replaced by latest TF."""
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import numpy as np


class VisionGridCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        def grid():
            return NS(header=NS(stamp=NS()),info=NS(origin=NS(position=NS(),orientation=NS())))
        stubs={"rclpy":NS(),"rclpy.node":NS(Node=object),
               "rclpy.time":NS(Time=lambda **kwargs:NS(**kwargs)),
               "rclpy.qos":NS(QoSProfile=object,DurabilityPolicy=object,ReliabilityPolicy=object),
               "nav_msgs.msg":NS(OccupancyGrid=grid),"std_msgs.msg":NS(String=NS),
               "tf2_ros":NS(Buffer=object,TransformListener=object,TransformException=RuntimeError)}
        source=Path(__file__).resolve().parents[1]/"husky_drivable_area/vision_grid_node.py"
        spec=importlib.util.spec_from_file_location("husky_drivable_area._vision_grid_test",source)
        cls.module=importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,stubs):spec.loader.exec_module(cls.module)

    def setUp(self):
        self.node=object.__new__(self.module.VisionGridNode)
        self.node.pending=None
        self.node.max_age=.5
        self.node.odom_frame="odom"
        self.node.mount=NS(camera_frame="camera",ground_frame="ground")
        self.node.calibration=object()
        self.node.output=mock.Mock()
        self.node.status=mock.Mock()
        self.node.buffer=mock.Mock()
        self.node.buffer.lookup_transform.return_value=NS(transform=NS(
            rotation=NS(x=0.,y=0.,z=0.,w=1.),translation=NS(x=0.,y=0.,z=.132)))
        self.now_ns=10_100_000_000
        self.node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=self.now_ns),clock_type="ros")
        self.packet=dict(source_header=dict(frame_id="camera",stamp=dict(sec=10,nanosec=0)),raw_shape_hw=[480,640])
        self.labels=np.ones((48,64),np.uint8)
        self.decode=mock.patch.object(self.module,"decode_segmentation",return_value=(self.packet,self.labels))
        self.decode.start();self.addCleanup(self.decode.stop)
        self.project=mock.patch.object(self.module,"project_veto_grid",return_value=(
            np.full((4,4),-1,np.int8),[0.,0.],dict(observed_drivable_cells=5)))
        self.projection=self.project.start();self.addCleanup(self.project.stop)

    def status(self):
        return json.loads(self.node.status.publish.call_args.args[0].data)

    def receive(self):
        self.node.receive(NS(data="packet"))

    def test_future_tf_is_retried_at_original_capture_stamp_without_redecoding(self):
        self.node.buffer.lookup_transform.side_effect=RuntimeError("future by 13ms")
        self.receive()
        self.assertEqual(self.status()["reason"],"waiting_for_capture_tf")
        self.assertFalse(self.status()["valid"])
        self.node.output.publish.assert_not_called()
        self.projection.assert_not_called()
        self.node.buffer.lookup_transform.side_effect=None
        self.now_ns+=20_000_000
        self.node.retry_projection()
        self.assertTrue(self.status()["valid"])
        self.assertIsNone(self.node.pending)
        self.node.output.publish.assert_called_once()
        stamp=self.node.output.publish.call_args.args[0].header.stamp
        self.assertEqual((stamp.sec,stamp.nanosec),(10,0))
        for call in self.node.buffer.lookup_transform.call_args_list:
            self.assertEqual(call.args[:2],("odom","ground"))
            self.assertEqual(call.args[2].nanoseconds,10_000_000_000)
        self.module.decode_segmentation.assert_called_once()

    def test_new_packet_replaces_waiting_packet_and_no_old_grid_is_published(self):
        self.node.buffer.lookup_transform.side_effect=RuntimeError("future")
        self.receive()
        newer=dict(source_header=dict(frame_id="camera",stamp=dict(sec=10,nanosec=50_000_000)),raw_shape_hw=[480,640])
        self.module.decode_segmentation.return_value=(newer,self.labels)
        self.receive()
        self.node.buffer.lookup_transform.side_effect=None
        self.node.retry_projection()
        message=self.node.output.publish.call_args.args[0]
        self.assertEqual(message.header.stamp.nanosec,50_000_000)
        self.node.output.publish.assert_called_once()

    def test_missing_tf_expires_without_projection_or_latest_tf_fallback(self):
        self.node.buffer.lookup_transform.side_effect=RuntimeError("missing")
        self.receive()
        self.now_ns=10_501_000_000
        self.node.buffer.lookup_transform.side_effect=None
        self.node.retry_projection()
        self.node.output.publish.assert_not_called()
        self.projection.assert_not_called()
        self.assertIsNone(self.node.pending)
        self.assertFalse(self.status()["valid"])
        self.assertEqual(self.status()["reason"],"stale_segmentation_waiting_for_tf")

    def test_invalid_new_packet_cancels_pending_valid_packet(self):
        self.node.buffer.lookup_transform.side_effect=RuntimeError("future")
        self.receive()
        self.module.decode_segmentation.side_effect=ValueError("invalid source")
        self.receive()
        self.node.buffer.lookup_transform.side_effect=None
        self.node.retry_projection()
        self.assertIsNone(self.node.pending)
        self.node.output.publish.assert_not_called()
        self.assertFalse(self.status()["valid"])

    def test_projection_that_crosses_original_deadline_cannot_publish(self):
        original=self.projection.return_value
        def delayed(*args):
            self.now_ns=10_501_000_000
            return original
        self.projection.side_effect=delayed
        self.receive()
        self.assertFalse(self.status()["valid"])
        self.assertEqual(self.status()["reason"],"vision_projection_too_old")
        self.node.output.publish.assert_not_called()

    def test_watchdog_remains_invalid_when_capture_or_receipt_expires(self):
        self.receive()
        self.assertTrue(self.status()["valid"])
        self.now_ns=10_600_000_000
        self.node.watchdog()
        self.assertFalse(self.status()["valid"])
        self.assertEqual(self.status()["reason"],"vision_timeout")
