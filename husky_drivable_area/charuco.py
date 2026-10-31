"""ChArUco detection, diverse-view capture and raw pinhole calibration."""

from pathlib import Path
import json

import cv2
import numpy as np

from .ground_geometry import CameraCalibration


class CharucoSession:
    def __init__(self, squares_x=7, squares_y=5, square_m=.024, marker_m=.017,
                 dictionary="DICT_5X5_100", min_views=20):
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
            raise ValueError("ChArUco requires OpenCV >=4.7 with aruco; use the project venv with OpenCV 4.10")
        if not (3 <= squares_x <= 30 and 3 <= squares_y <= 30 and
                np.isfinite([square_m, marker_m]).all() and 0 < marker_m < square_m and min_views >= 12):
            raise ValueError("invalid ChArUco dimensions or minimum view count")
        if not dictionary.startswith("DICT_") or not isinstance(getattr(cv2.aruco, dictionary, None), int):
            raise ValueError("unknown ArUco dictionary")
        self.board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_m, marker_m,
                                           cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)))
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.object_corners = self.board.getChessboardCorners()
        self.extent = np.array([[0, 0], [squares_x*square_m, 0],
                                [squares_x*square_m, squares_y*square_m], [0, squares_y*square_m]], np.float32)
        self.config = dict(squares_x=squares_x, squares_y=squares_y, square_m=square_m,
                           marker_m=marker_m, dictionary=dictionary)
        self.min_views, self.size, self.views = min_views, None, []

    def detect(self, frame):
        corners, ids, _, _ = self.detector.detectBoard(frame)
        return corners, ids

    def add(self, frame, corners, ids):
        size = (frame.shape[1], frame.shape[0])
        if self.size is not None and size != self.size:
            raise ValueError("camera resolution changed during calibration")
        if corners is None or ids is None or len(ids) < 12:
            return "need_at_least_12_charuco_corners"
        pixels = np.asarray(corners, np.float32).reshape(-1, 2)
        ids = np.asarray(ids).reshape(-1)
        if (len(pixels) != len(ids) or not np.issubdtype(ids.dtype, np.integer) or
                len(set(ids.tolist())) != len(ids) or (ids < 0).any() or
                (ids >= len(self.object_corners)).any() or not np.isfinite(pixels).all()):
            raise ValueError("invalid detected ChArUco correspondences")
        if (pixels < 0).any() or (pixels[:, 0] >= size[0]).any() or (pixels[:, 1] >= size[1]).any():
            raise ValueError("detected corner outside raw image")
        hull = cv2.convexHull(pixels)
        if cv2.contourArea(hull) < .02*size[0]*size[1]:
            return "move_board_closer"
        x, y, w, h = cv2.boundingRect(hull)
        gray = cv2.cvtColor(frame[y:y+h, x:x+w], cv2.COLOR_BGR2GRAY)
        if cv2.Laplacian(gray, cv2.CV_64F).var() < 30:
            return "hold_board_still_or_improve_focus"
        homography, _ = cv2.findHomography(self.object_corners[ids, :2], pixels, 0)
        if homography is None:
            return "invalid_board_pose"
        quad = cv2.perspectiveTransform(self.extent.reshape(1, 4, 2), homography).reshape(4, 2)
        if not np.isfinite(quad).all() or not cv2.isContourConvex(quad.astype(np.float32)):
            return "invalid_board_pose"
        scale = np.hypot(*size)
        if any(np.sqrt(np.mean(np.sum((quad-v["quad"])**2, axis=1)))/scale < .035 for v in self.views):
            return "move_or_tilt_board_for_new_view"
        if len(self.views) >= 80:
            return "capture_limit_reached"
        sides = np.linalg.norm(quad-np.roll(quad, -1, axis=0), axis=1)
        if (sides <= 1).any():
            return "invalid_board_pose"
        tilted = max(abs(np.log(sides[0]/sides[2])), abs(np.log(sides[1]/sides[3]))) >= .06
        self.size = size
        self.views.append(dict(pixels=pixels.copy(), ids=ids.copy(), quad=quad, tilted=bool(tilted)))
        return "captured"

    def progress(self):
        coverage = 0.
        if self.views:
            all_points = np.concatenate([v["pixels"] for v in self.views])
            coverage = cv2.contourArea(cv2.convexHull(all_points))/np.prod(self.size)
        tilts = sum(v["tilted"] for v in self.views)
        reasons = []
        if len(self.views) < self.min_views:
            reasons.append(f"views {len(self.views)}/{self.min_views}")
        if coverage < .35:
            reasons.append("move board toward image edges")
        if tilts < 3:
            reasons.append(f"tilted views {tilts}/3")
        return dict(views=len(self.views), coverage=float(coverage), tilted_views=tilts,
                    ready_to_calibrate=not reasons, pending=reasons)

    def calibrate(self):
        progress = self.progress()
        if not progress["ready_to_calibrate"]:
            raise ValueError("; ".join(progress["pending"]))
        training = [v for i, v in enumerate(self.views) if i % 5 != 4]
        held_out = [v for i, v in enumerate(self.views) if i % 5 == 4]
        objects = [self.object_corners[v["ids"]] for v in training]
        images = [v["pixels"].reshape(-1, 1, 2) for v in training]
        rms, matrix, distortion, _, _ = cv2.calibrateCamera(objects, images, self.size, None, None)
        mapping = dict(image_width=self.size[0], image_height=self.size[1], camera_name="charuco_camera",
                       distortion_model="plumb_bob", camera_matrix=dict(rows=3, cols=3, data=matrix.reshape(-1).tolist()),
                       distortion_coefficients=dict(rows=1, cols=5, data=distortion.reshape(-1).tolist()),
                       rectification_matrix=dict(rows=3, cols=3, data=np.eye(3).reshape(-1).tolist()),
                       projection_matrix=dict(rows=3, cols=4, data=np.column_stack([matrix, np.zeros(3)]).reshape(-1).tolist()))
        CameraCalibration.from_mapping(mapping)
        if not np.isfinite(rms):
            raise ValueError("nonfinite calibration residual")
        held_errors = []
        for view in held_out:
            points = self.object_corners[view["ids"]]
            ok, rvec, tvec = cv2.solvePnP(points, view["pixels"], matrix, distortion)
            if not ok:
                raise ValueError("held-out pose fit failed")
            predicted, _ = cv2.projectPoints(points, rvec, tvec, matrix, distortion)
            held_errors.append(float(np.sqrt(np.mean(np.sum((predicted.reshape(-1, 2)-view["pixels"])**2, axis=1)))))
        warnings = []
        if rms > 1 or max(held_errors) > 1.5:
            warnings.append("large pixel residual: inspect blur, board flatness, dictionary and corner coverage")
        return mapping, dict(board=self.config, **progress, training_views=len(training), held_out_views=len(held_out),
                             training_rms_px=float(rms), held_out_rms_px=held_errors, warnings=warnings,
                             verified_ground_geometry=False, ready_for_control=False)


def save_calibration(folder, mapping, report):
    import yaml
    folder = Path(folder)
    # Reserve output exclusively. Active camera calibration is never overwritten.
    with (folder / "camera.yaml").open("x") as target:
        yaml.safe_dump(mapping, target, sort_keys=False)
    (folder / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False))
