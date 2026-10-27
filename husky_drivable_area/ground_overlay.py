"""Project the robot's forward ground centerline into the processed image."""

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
    # Only the forward centerline (ground Y=0), never a fabricated image axis.
    ground = np.column_stack([np.linspace(0, 5, 512), np.zeros(512)])
    raw, visible = ground_to_image(ground, calibration, mount)
    pixels = (raw + 0.5) * [w / calibration.width, h / calibration.height] - 0.5
    visible &= ((pixels[:, 0] >= 0) & (pixels[:, 0] <= w-1) &
                (pixels[:, 1] >= 0) & (pixels[:, 1] <= h-1))
    pixels[~visible] = np.nan
    return pixels
