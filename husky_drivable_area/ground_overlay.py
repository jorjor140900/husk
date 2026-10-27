"""Project the robot's ground centerline across the camera view."""

import cv2
import numpy as np

from .ground_geometry import ground_to_image


def robot_axis_pixels(calibration, mount, processed_shape, input_shape, source_frame):
    if source_frame != mount.camera_frame:
        raise ValueError("overlay source camera frame differs from measured mount")
    if tuple(input_shape) != (calibration.height, calibration.width):
        raise ValueError("overlay camera resolution differs from calibration")
    h, w = processed_shape
    if min(h, w) < 16:
        raise ValueError("invalid overlay image size")
    # Include the floor behind the robot origin so rear-mounted cameras can
    # display the same centerline down to the bottom of the image. This is a
    # drawing guide only; it does not extend the yellow path or command reverse.
    distances = np.r_[np.linspace(-5, 5, 4096), np.geomspace(5.01, 1000, 1024)]
    ground = np.column_stack([distances, np.zeros(len(distances))])
    raw, visible = ground_to_image(ground, calibration, mount)
    # A polynomial distortion model can fold rays outside its calibrated view
    # back into image bounds. Such a point is not visible just because its
    # forward-distorted coordinates happen to land in the image. Require the
    # inverse camera mapping to recover the actual ray (central lens branch).
    indices = np.flatnonzero(visible)
    if len(indices):
        world = np.column_stack([ground[indices], np.zeros(len(indices))])
        camera = (world-mount.origin) @ mount.rotation
        normalized = camera[:, :2]/camera[:, 2, None]
        recovered = cv2.undistortPoints(raw[indices].reshape(-1, 1, 2),
                                      calibration.matrix, calibration.distortion).reshape(-1, 2)
        error_px = np.linalg.norm((recovered-normalized)*
                                  [calibration.matrix[0, 0], calibration.matrix[1, 1]], axis=1)
        visible[indices] &= np.isfinite(error_px) & (error_px <= .5)
    pixels = (raw + 0.5) * [w / calibration.width, h / calibration.height] - 0.5
    visible &= ((pixels[:, 0] >= 0) & (pixels[:, 0] <= w-1) &
                (pixels[:, 1] >= 0) & (pixels[:, 1] <= h-1))
    pixels[~visible] = np.nan
    # Keep dense samples at view boundaries (within a pixel of the edge), but
    # reduce the cached curve elsewhere to keep per-frame drawing inexpensive.
    boundaries = np.flatnonzero(np.diff(visible.astype(int)))
    keep = np.unique(np.r_[np.arange(0, len(pixels), 8), boundaries,
                           boundaries+1, len(pixels)-1])
    return pixels[keep]
