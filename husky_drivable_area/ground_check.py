"""Draw measured ground coordinates on raw camera images, without AI or ROS."""

import cv2
import numpy as np

from .ground_geometry import ground_to_image


DISTANCES_M = (0.10, 0.25, 0.50, 0.75, 1.0, 1.5, 2.0, 3.0)
BANNER_HEIGHT = 116


class GroundCheckOverlay:
    """A fixed calibration/mount grid; restart after changing the camera rig.

    Measurement origin is the point on the floor directly below the lens.
    Grid axes follow the ground frame, including when camera offsets are nonzero.
    Intrinsics always refer to the raw image, before adding the display banner.
    """

    def __init__(self, calibration, mount):
        self.calibration, self.mount = calibration, mount
        self.origin = np.array([mount.camera_x_m, mount.camera_y_m])
        self.lines = []
        for x in DISTANCES_M:
            self._line(np.column_stack([np.full(121, x), np.linspace(-0.6, 0.6, 121)]))
        for y in (-0.2, 0.0, 0.2):
            self._line(np.column_stack([np.linspace(0.0, 3.0, 301), np.full(301, y)]))
        self.targets = []
        for x in DISTANCES_M:
            for y in (0.0, 0.2, -0.2):
                ground = self.origin + [x, y]
                pixels, visible = ground_to_image([ground], calibration, mount)
                if visible[0]:
                    label = f"{'C' if y == 0 else 'L' if y > 0 else 'R'}{round(x*100):03d}"
                    self.targets.append({"id": label, "relative_xy_m": [x, y],
                                         "ground_xy_m": ground.tolist(), "raw_pixel_uv": pixels[0].tolist()})

    def _line(self, relative):
        pixels, visible = ground_to_image(relative + self.origin, self.calibration, self.mount)
        # Keep each visible run separate: never join over the camera plane,
        # an out-of-view interval, or distortion that leaves/re-enters the image.
        edges = np.diff(np.r_[False, visible, False].astype(int))
        for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            if stop-start >= 2:
                self.lines.append(np.rint(pixels[start:stop]).astype(np.int32))

    def metadata(self):
        return {"check_only": True, "ready_for_control": False, "verified_traversability": False,
                "physical_geometry_verified": False, "projection_model": "flat_ground",
                "measurement_origin": "floor_below_lens; axes_follow_ground_frame",
                "measurement_origin_ground_xy_m": self.origin.tolist(),
                "camera_height_m": self.mount.height_m, "camera_pitch_down_deg": self.mount.pitch_down_deg,
                "visible_targets": self.targets}

    def draw(self, image):
        if (image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or
                image.shape[:2] != (self.calibration.height, self.calibration.width)):
            raise ValueError("raw camera resolution/format differs from calibration; keep the calibrated capture size")
        result = image.copy()
        for line in self.lines:
            cv2.polylines(result, [line], False, (150, 150, 70), 1, cv2.LINE_AA)
        for target in self.targets:
            u, v = np.rint(target["raw_pixel_uv"]).astype(int)
            center = target["relative_xy_m"][1] == 0
            color = (0, 230, 255) if center else (255, 220, 100)
            cv2.drawMarker(result, (u, v), color, cv2.MARKER_CROSS, 15, 2)
            label = f"{target['id']} X={target['relative_xy_m'][0]:.2f}m" if center else target["id"]
            (width, height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            position = (max(0, min(u+10, result.shape[1]-width-2)), max(height+2, min(v-8, result.shape[0]-3)))
            cv2.putText(result, label, position, cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(result, label, position, cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        visible = ", ".join(f"{t['relative_xy_m'][0]:.2f}" for t in self.targets if t["relative_xy_m"][1] == 0)
        return add_banner(result, ["GROUND CHECK | FLAT FLOOR | NO MOTOR OUTPUT",
            "Origin: floor below lens. X=forward, Y=left (ground axes).",
            f"h={self.mount.height_m:.3f}m | pitch={self.mount.pitch_down_deg:.1f}deg | raw {self.calibration.width}x{self.calibration.height}",
            "C: Y=0 | L: Y=+0.20m | R: Y=-0.20m. Match measured floor marks.",
            "Visible center X (m): " + (visible or "NONE - check pose/calibration/view")])


def add_banner(image, lines, *, error=False):
    result = cv2.copyMakeBorder(image, BANNER_HEIGHT, 0, 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
    for row, line in enumerate(lines[:5]):
        cv2.putText(result, line, (8, 19+row*21), cv2.FONT_HERSHEY_SIMPLEX, 0.43,
                    (80, 80, 255) if error else (240, 240, 240), 1, cv2.LINE_AA)
    return result
