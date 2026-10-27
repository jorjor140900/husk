# Fit the camera mount from measured floor points

`fit_camera_mount` opens a local browser to select four points in the original
raw image. It uses the raw-camera intrinsics/distortion and all six mount pose
parameters. The detector, ROS topics and robot commands are not used or changed.

Record the image with the camera fixed in its actual mount and unchanged focus
and capture mode. Measure coordinates from the same robot ground-frame origin:
X forward, Y left, Z up. The four points must lie on a flat floor. Use distinct
corners of a measured rectangle, with its sides parallel to the robot axes.
For the field measurements supplied on 2026-10-27:

```bash
ros2 run husky_vision fit_camera_mount \
  --image "$HOME/husky_vision_ws/robot/ground-calibration.png" \
  --camera-calibration "$HOME/husky_vision_ws/camera_calibration/camera-v2.yaml" \
  --camera-mount "$HOME/husky_vision_ws/robot/robot-mount.json" \
  --near-x 1.27 --far-x 1.87 --left-y 0.375 --right-y=-0.225 \
  --fix-origin
```

Select A far-left, B far-right, C near-left, D near-right. Browser resizing
preserves original pixel coordinates. Undo/reset are available before saving.
If no browser opens, open the printed localhost URL on the same computer.
This works with headless OpenCV; it does not require `cv2.imshow` or Qt.

The session saves `report.json`, `mount.candidate.json`, and `raw.png` under a
new `outputs/mount-fit/<timestamp>` directory, then exits. Nothing replaces the
active mount. The report includes both planar PnP candidates when available,
reprojection residuals, an ambiguity flag, and the previous measured mount.
Large disagreement with measured height/offsets must be resolved rather than
assuming the fit is physically correct. A small residual at the four fitted
points is not independent validation: check extra measured floor points before
applying a candidate or using the ground path for control.

`--fix-origin` retains the measured height and XYZ offsets and fits only roll,
pitch and yaw. A large remaining pixel residual indicates that the measurements,
point correspondence, or camera calibration need checking; do not force a
translation change just to reduce that residual. Omitting the option returns
the unrestricted planar PnP solutions for comparison.
