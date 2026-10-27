# Projected robot centerline in the camera overlay

The blue line can use raw-camera calibration and the measured mount to project
the robot's ground centerline (`X=-5..5 m, Y=0, Z=0`) into the image.
It accounts for lens distortion, camera translation/rotation and the detector's
processed resolution. A left-mounted camera normally sees this line to the
right of the image center; its image X varies with distance.

Pass both `camera_calibration` and `camera_mount` to `drivable_area.launch.py`.
These must be the same files used by `ground_path.launch.py`. The source frame
and raw capture resolution must match. `robot_pipeline.launch.py` passes the
profile's camera files automatically.

For an existing exported workspace, update with
`tools/create_ros2_workspace.py --workspace ~/husky_vision_ws --update-code --update-visualization`
and rebuild. The additional flag backs up and updates only the drawing module
alongside the usual ROS updates; detector/model/user calibration stay preserved.

The legend reads `Blue=robot Y=0` in calibrated mode. The visible blue curve and
lookahead marker use the projection; `dx` compares the yellow path to this curve
at the same image row, in processed-image pixels. No marker or dx is invented
when that row is outside the visible centerline. Invisible sections are not
connected, and a calibration/frame mismatch hides the blue line with a warning.
The drawing includes the floor behind the robot origin, allowing a rear-mounted,
downward-facing camera to show the guide down to the bottom image edge. It keeps
the same thickness and processed resolution; it does not extend a driving path.

Without these parameters, the existing configured image axis remains available
and is labeled `Blue=assumed axis`. The legacy `reference_geometry` metadata
explicitly identifies that configured alignment assumption.

This display uses measured inputs and a flat-ground assumption; it does not
verify their physical accuracy. It does not move the detector seed, change the
yellow path, alter controller commands, or check the robot footprint. Physical
floor-marker checks and the control permission/obstacle checks remain required.
