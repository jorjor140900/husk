# ChArUco lens calibration from the existing ROS camera

Keep the USB webcam node running in the actual raw capture mode (640x480 here).
Keep focus/zoom unchanged. Stop AI, ground projection and controllers during
calibration. Mount the printed sheet flat on a rigid backing; move the sheet,
leaving the camera mount fixed. Measure the printed square, not the PDF setting.

The supplied board is 7 by 5 **squares**, each 24 mm, with 17 mm markers from
`DICT_5X5_100`. There are 24 internal ChArUco corners. The standard board uses
marker IDs starting at zero. Watch the corner overlay to confirm the printed
layout matches; a different dictionary/ID layout must not be calibrated as this
board. OpenCV 4.10 from requirements-ros2.txt supplies the needed aruco API;
headless OpenCV works because the viewer is ROS image_view. No contrib-wheel
replacement or separate GUI toolkit is required.

After exporting/updating and building the ROS package:

```bash
ros2 run husky_vision charuco_calibration --ros-args \
  -p image_topic:=/camera/image_raw \
  -p squares_x:=7 -p squares_y:=5 \
  -p square_m:=0.024 -p marker_m:=0.017 \
  -p dictionary:=DICT_5X5_100
```

In a sourced second terminal:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/charuco/overlay -p image_transport:=compressed \
  -p autosize:=false -p width:=960 -p height:=720
```

Frames are captured automatically at new positions/angles, with at least 12
detected ChArUco corners. Hold the board still briefly after each move. Capture
near/far, center, all sides/corners of the image, and several tilted poses about
both axes. Keep the board sharp, evenly lit and most of its corners visible.
Identical poses, small boards and blurred regions are rejected with an on-image
reason. The current frame is latest-only; raw capture stamps are retained on
the overlay, and old input is rejected. No camera image is resized for fitting.

Readiness requires 20 distinct views, corner-hull coverage of at least 35% of the
image, and at least three perspective-tilted views. If the counter stalls, read
the `Need` line and move/tilt the board as directed. At most 80 views are retained;
restart the node to start a fresh session if necessary.

When the overlay reads READY, use a third sourced terminal:

```bash
ros2 service call /charuco/calibrate std_srvs/srv/Trigger '{}'
```

The session saves raw views and observations under a new
`~/husky_vision_ws/camera_calibration/charuco-<timestamp>/` folder. Successful
calibration adds `camera.yaml` and `report.json`, prints their path and pixel
errors, and freezes capture. A new session is required for a second calibration.
Existing camera-v2.yaml and mount files are preserved. No TF or motor command
is published, and no active profile is changed.

Calibration uses OpenCV calibrateCamera with the indexed board corners, avoiding
a dependency on the optional calibrateCameraCharuco function. Every fifth view
is held out from intrinsic fitting; its pose/reprojection residual is measured
afterward. Inspect warnings for training RMS >1 pixel or held-out RMS >1.5
pixels. These checks test lens consistency; they do not validate meter distances
on the floor. Refit the mount with the chosen new intrinsics, then check the
independent E/F/G measurements before using the projection for control.
