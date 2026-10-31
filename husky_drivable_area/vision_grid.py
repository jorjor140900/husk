"""Bounded segmentation packets and calibrated, veto-only ground grids."""
import base64
import json
import math

import cv2
import numpy as np

from .ground_geometry import ground_to_image


def encode_segmentation(result, header, horizon_fraction):
    labels = (result.raw_mask > 0).astype(np.uint8)
    labels[:int(labels.shape[0]*horizon_fraction)] = 255  # not analyzed, not blocked
    ok, encoded = cv2.imencode(".png", labels)
    if not ok:
        raise ValueError("segmentation_png_failed")
    return json.dumps(dict(protocol=1, valid=True, source_header=dict(frame_id=header.frame_id,
        stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec)),
        raw_shape_hw=list(result.input_shape), shape_hw=list(labels.shape),
        png_b64=base64.b64encode(encoded).decode("ascii")), allow_nan=False)


def decode_segmentation(text):
    if not isinstance(text, str) or len(text) > 2_000_000:
        raise ValueError("oversized_segmentation_packet")
    packet = json.loads(text)
    if not isinstance(packet, dict) or packet.get("protocol") != 1 or packet.get("valid") is not True:
        raise ValueError("invalid_segmentation_packet")
    for key in ("raw_shape_hw", "shape_hw"):
        size = packet[key]
        if (not isinstance(size, list) or len(size) != 2 or
                any(type(v) is not int or not 16 <= v <= 4096 for v in size) or np.prod(size) > 1_000_000):
            raise ValueError("invalid_segmentation_dimensions")
    encoded = base64.b64decode(packet["png_b64"], validate=True)
    if len(encoded) < 24 or encoded[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("segmentation_requires_png")
    width, height = int.from_bytes(encoded[16:20], "big"), int.from_bytes(encoded[20:24], "big")
    if [height, width] != packet["shape_hw"]:
        raise ValueError("segmentation_png_dimensions_differ")
    labels = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_UNCHANGED)
    if labels is None or labels.ndim != 2 or labels.dtype != np.uint8 or not np.isin(labels, [0, 1, 255]).all():
        raise ValueError("invalid_segmentation_labels")
    return packet, labels


def project_veto_grid(labels, raw_shape, calibration, mount, ground_to_odom, resolution=.05, extent=6.):
    if tuple(raw_shape) != (calibration.height, calibration.width):
        raise ValueError("mask_capture_resolution_differs_from_calibration")
    transform = np.asarray(ground_to_odom, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0,0,0,1]) or
            not np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3), atol=1e-6) or
            abs(np.linalg.det(transform[:3,:3])-1) > 1e-6 or abs(transform[2,2]-1) > 1e-3):
        raise ValueError("mask_requires_planar_capture_time_tf")
    if (not math.isfinite(resolution) or not math.isfinite(extent) or
            not .02 <= resolution <= .2 or not 1 <= extent <= 12):
        raise ValueError("invalid_vision_grid_size")
    size = int(math.ceil(extent/resolution))
    origin = np.floor((transform[:2,3]-extent/2)/resolution)*resolution
    yy, xx = np.mgrid[:size, :size]
    centers = origin+np.column_stack([xx.reshape(-1)+.5, yy.reshape(-1)+.5])*resolution
    known = np.ones(len(centers), bool)
    blocked = np.zeros(len(centers), bool)
    for offset in ((0,0), (-.5,-.5), (-.5,.5), (.5,-.5), (.5,.5)):
        ground = (centers+np.array(offset)*resolution-transform[:2,3])@transform[:2,:2]
        chunks = [ground_to_image(ground[i:i+10000], calibration, mount) for i in range(0,len(ground),10000)]
        raw = np.concatenate([chunk[0] for chunk in chunks])
        visible = np.concatenate([chunk[1] for chunk in chunks])
        indices = np.flatnonzero(visible)
        sampled = np.full(len(centers), 255, np.uint8)
        if len(indices):
            # Reject polynomial distortion foldbacks outside the actual lens view.
            camera = (np.column_stack([ground[indices], np.zeros(len(indices))])-mount.origin)@mount.rotation
            expected = camera[:,:2]/camera[:,2,None]
            recovered = cv2.undistortPoints(raw[indices].reshape(-1,1,2), calibration.matrix, calibration.distortion).reshape(-1,2)
            consistent = np.linalg.norm((expected-recovered)*[calibration.matrix[0,0],calibration.matrix[1,1]],axis=1) <= .5
            indices = indices[consistent]
            pixels = (raw[indices]+.5)*[labels.shape[1]/calibration.width, labels.shape[0]/calibration.height]-.5
            u, v = np.rint(pixels).astype(int).T
            inside = (u >= 0)&(u < labels.shape[1])&(v >= 0)&(v < labels.shape[0])
            sampled[indices[inside]] = labels[v[inside],u[inside]]
        known &= sampled == 1
        blocked |= sampled == 0
    # No free-space values: AI can veto, but cannot erase map/LiDAR obstacles
    # or claim unknown parts of the map are drivable.
    costs = np.where(blocked, 100, -1).astype(np.int8).reshape(size,size)
    return costs, origin, dict(observed_drivable_cells=int(known.sum()), blocked_cells=int(blocked.sum()),
                               unobserved_or_no_veto_cells=int((~blocked).sum()))
