"""Timed-cloud parsing and actual geometric motion compensation."""
from types import SimpleNamespace as NS
import unittest

import numpy as np

from husky_drivable_area.vlp16 import TimedCloud, deskew_returns, read_timed_cloud


def message(endian="<", padded=False):
    names = ["x", "y", "z", "ring", "time"]
    formats = [endian+f for f in ("f4", "f4", "f4", "u2", "f4")]
    dtype = np.dtype({"names": names, "formats": formats, "offsets": [0, 4, 8, 12, 14], "itemsize": 18})
    array = np.zeros(2, dtype=dtype)
    array["x"], array["ring"], array["time"] = [1, 2], [0, 15], [-.1, .001]
    data = array.tobytes()
    if padded:
        data = data[:18]+b"PAD!"+data[18:]+b"PAD!"
    return NS(header=NS(stamp=NS(sec=100, nanosec=0), frame_id="/velodyne"),
              width=1 if padded else 2, height=2 if padded else 1,
              point_step=18, row_step=22 if padded else 36, is_bigendian=endian == ">", data=data,
              fields=[NS(name=n, offset=o, datatype=t, count=1) for n, o, t in
                      zip(names, [0, 4, 8, 12, 14], [7, 7, 7, 4, 7])])


class Vlp16Tests(unittest.TestCase):
    def test_preserves_signed_and_small_positive_times_both_byte_orders_and_padding(self):
        for endian in ("<", ">"):
            for padding in (False, True):
                cloud = read_timed_cloud(message(endian, padding))
                np.testing.assert_allclose(cloud.offsets_sec, [-.1, .001])
                np.testing.assert_allclose(cloud.xyz, [[1, 0, 0], [2, 0, 0]])
                self.assertEqual(cloud.rings.tolist(), [0, 15])
                self.assertEqual(cloud.frame_id, "velodyne")

    def test_rejects_missing_time_bad_layout_truncation_and_invalid_values(self):
        for kind in ("missing", "overlap", "truncated", "stamp", "ring", "nan", "time"):
            m = message()
            if kind == "missing": m.fields.pop()
            if kind == "overlap": m.fields[-1].offset = 0
            if kind == "truncated": m.data = m.data[:-1]
            if kind == "stamp": m.header.stamp.nanosec = 1_000_000_000
            if kind in ("ring", "nan", "time"):
                raw = bytearray(m.data)
                if kind == "ring": raw[12:14] = np.uint16(16).tobytes()
                if kind == "nan": raw[0:4] = np.float32(np.nan).tobytes()
                if kind == "time": raw[14:18] = np.float32(.1).tobytes()
                m.data = raw
            with self.subTest(kind=kind), self.assertRaises(ValueError): read_timed_cloud(m)

    def test_translation_maps_a_stationary_world_target_to_one_reference_position(self):
        epoch = 1_792_989_224_000_000_000
        # Robot moves +x 1m/s. An earlier return measures farther to the same wall.
        cloud = TimedCloud(epoch+100_000_000, "velodyne", np.array([[5.1, 0, 0], [5, 0, 0]]),
                           np.array([0, 0]), np.array([-.1, 0]))
        points = deskew_returns(cloud, np.array([epoch, epoch+100_000_000]),
                                [[0, 0, 0], [.1, 0, 0]], np.eye(4))
        np.testing.assert_allclose(points, [[5, 0, 0], [5, 0, 0]], atol=1e-12)

    def test_yaw_wrap_and_sensor_offset_are_compensated_at_each_return(self):
        epoch = 100_000_000_000
        yaws = np.deg2rad([179, 180, -179])
        transform = np.eye(4); transform[:3, 3] = [-.232, 0, .882]
        world_target = np.array([[-4., 1.]]*3)
        points = []
        for yaw, target in zip(yaws, world_target):
            c, s = np.cos(yaw), np.sin(yaw)
            base = target @ np.array([[c, -s], [s, c]])
            points.append([base[0]+.232, base[1], 0])
        cloud = TimedCloud(epoch+100_000_000, "velodyne", np.array(points), np.array([0, 0, 0]), np.array([-.1, -.05, 0]))
        output = deskew_returns(cloud, np.array([epoch, epoch+100_000_000]),
                                [[0, 0, yaws[0]], [0, 0, yaws[-1]]], transform)
        np.testing.assert_allclose(output[0], output[1], atol=1e-12)
        np.testing.assert_allclose(output[0], output[2], atol=1e-12)
        self.assertAlmostEqual(output[0, 2], .882)

    def test_requires_entire_time_interval_including_positive_point_offsets(self):
        cloud = read_timed_cloud(message())
        for stamps, poses, error in (
            ([99_950_000_000, 100_020_000_000], [[0, 0, 0]]*2, "bracket"),
            ([99_800_000_000, 100_000_000_000], [[0, 0, 0]]*2, "bracket"),
            ([99_800_000_000, 100_100_000_000], [[0, 0, 0]]*2, "gap"),
            ([100_000_000_000, 99_800_000_000], [[0, 0, 0]]*2, "trajectory"),
        ):
            with self.subTest(error=error), self.assertRaisesRegex(ValueError, error):
                deskew_returns(cloud, np.array(stamps), poses, np.eye(4))

    def test_rejects_nonrigid_transform_instead_of_assuming_identity(self):
        cloud = read_timed_cloud(message())
        transform = np.eye(4); transform[0, 0] = 2
        with self.assertRaisesRegex(ValueError, "transform"):
            deskew_returns(cloud, np.arange(99_800_000_000, 100_100_000_001, 20_000_000),
                           np.zeros((16, 3)), transform)


if __name__ == "__main__": unittest.main()
