"""Read-only check of a mount against independently measured floor points."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np

from .ground_geometry import CameraCalibration, CameraMount, ground_to_image
from .mount_fit import serve_points


def check_floor_points(calibration, mount, pixels, expected):
    pixels, expected = np.asarray(pixels, dtype=float), np.asarray(expected, dtype=float)
    if (pixels.ndim != 2 or pixels.shape[1] != 2 or pixels.shape != expected.shape or
            not 1 <= len(pixels) <= 1000 or not np.isfinite(pixels).all() or not np.isfinite(expected).all()):
        raise ValueError("need matching finite pixel XY and measured ground XY points")
    if (pixels < 0).any() or (pixels[:, 0] > calibration.width-1).any() or (pixels[:, 1] > calibration.height-1).any():
        raise ValueError("point outside calibrated raw image")
    normalized = cv2.undistortPoints(pixels.reshape(-1, 1, 2), calibration.matrix, calibration.distortion).reshape(-1, 2)
    rays = np.column_stack([normalized, np.ones(len(pixels))]) @ mount.rotation.T
    if not np.isfinite(rays).all() or (rays[:, 2] >= -1e-6).any():
        raise ValueError("selected point does not intersect the floor in front of the camera")
    actual = (mount.origin-mount.height_m/rays[:, 2, None]*rays)[:, :2]
    projected, visible = ground_to_image(expected, calibration, mount)
    return {"expected_xy_m": expected.tolist(), "estimated_xy_m": actual.tolist(),
            "error_xy_cm": ((actual-expected)*100).tolist(),
            "distance_error_cm": (np.linalg.norm(actual-expected, axis=1)*100).tolist(),
            "observed_pixels_xy": pixels.tolist(), "expected_pixels_xy": projected.tolist(),
            "expected_points_visible": visible.tolist(),
            "mount_changed": False, "verified_geometry": False, "ready_for_control": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("image", "camera-calibration", "camera-mount"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--near-x", type=float, default=1.0)
    parser.add_argument("--far-x", type=float, default=1.5)
    parser.add_argument("--left-y", type=float, default=.2)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mount-check"))
    args = parser.parse_args(argv)
    calibration, mount = CameraCalibration.load(args.camera_calibration), CameraMount.load(args.camera_mount)
    image = cv2.imread(str(args.image.expanduser()))
    if image is None or image.shape[:2] != (calibration.height, calibration.width):
        parser.error("use the original raw image at the calibrated resolution")
    expected = np.array([[args.near_x, 0.], [args.far_x, 0.], [args.far_x, args.left_y]])
    if not np.isfinite(expected).all() or not args.far_x > args.near_x or args.left_y <= 0:
        parser.error("require finite far-x > near-x and left-y > 0")

    def submit(pixels):
        report = check_floor_points(calibration, mount, pixels, expected)
        report.update({"labels": ["E", "F", "G"], "mount": asdict(mount),
                       "note": "Independent point check only; no mount fitting or robot commands."})
        folder = args.output_dir.expanduser() / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        folder.mkdir(parents=True, exist_ok=False)
        (folder / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
        cv2.imwrite(str(folder / "raw.png"), image)
        return {"saved_to": str(folder.absolute()), **report}
    return serve_points(image, ["E", "F", "G"],
                        "คลิกฐานสัมผัสพื้น: E ใกล้กลาง → F ไกลกลาง → G ไกลซ้าย", submit)
