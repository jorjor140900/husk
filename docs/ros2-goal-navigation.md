# Goal navigation with a map, Nav2 and AI floor constraints

The destination architecture is now distinct from the earlier yellow-line
experiment. Nav2 selects a route to a map-frame goal and its Regulated Pure
Pursuit controller produces the only navigation request. SegFormer supplies a
calibrated local ground **veto** to both Nav2 costmaps before footprint inflation.
The yellow image path remains a diagnostic and does not command the robot in
this mode.

```text
manual survey: real ROS1 scan + odom/TF -> ROS2 SLAM -> saved map
navigation: saved map + AMCL -> map/odom/base pose + goal -> Nav2 route/controller
camera -> SegFormer labels -> capture-time odom grid -> AI costmap layer
Nav2 stamped request -> disabled-first navigation gate -> existing collision filter
  -> private stamped output -> ROS1 gateway lease/watchdog -> verified mux input
```

## Install on the Jazzy computer

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

In the Git checkout:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py --workspace ~/husky_vision_ws --update-code --with-navigation
```

This explicitly adds the companion C++ package `src/husky_vision_nav2` and backs
up local edits. Models, calibration and measured robot profiles are preserved.
Build both packages in the workspace:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision husky_vision_nav2
source install/local_setup.bash
```

Use the same ROS/workspace sources in every new terminal.

## Prepare now, before a real map is available

You can finish software preparation while the robot/site is unavailable. Run:

```bash
ros2 run husky_vision navigation_preflight \
  --profile "$HOME/husky_vision_ws/robot/robot.json"
```

This default mode checks installed Nav2/SLAM packages, the compiled AI plugin,
camera calibration/capture dimensions, mount, measured footprint and robot
frame configuration. It also checks relocations of the native Nav2 lifecycle
manager library: a package being installed does not establish ABI compatibility.
It never connects to the robot, starts SLAM, publishes a
goal, or changes a profile. The expected map path is `maps/site1.yaml`; use
`--map /path/to/site.yaml` when you choose a different name.

Before mapping, a normal result is:

```text
PENDING: saved_map | ... site1.yaml ...
Software prepared: True
Planning inputs ready: False | Autonomous control verified: False
```

A missing map does not fail **preparation** if all software/configuration checks
pass. Missing packages, an unbuilt AI plugin or incomplete measured values do
fail it. The report is saved under `outputs/navigation-preflight/`. Do not create
a blank substitute map or set unfinished field flags true to make this pass.

When a real map is saved, the later sequence is already prepared:

1. Stop the mapping launch, old ground/controller nodes and standalone
   `robot_link`; keep the gateway and tunnel. The navigation launch owns its
   own bridge and measured ground/camera TF.
2. Keep one USB webcam and AI detector running with the final calibration/mount.
3. Start `navigation.launch.py` with the saved map and `allow_motion:=false` as
   shown below, then use **2D Pose Estimate** at the actual robot location.
4. Check that laser observations line up with the saved walls, then run the live
   preview check:

```bash
ros2 run husky_vision navigation_preflight \
  --profile "$HOME/husky_vision_ws/robot/robot.json" \
  --map "$HOME/husky_vision_ws/maps/site1.yaml" \
  --live --duration 10
```

Live mode only subscribes and calls lifecycle **GetState**. It checks the active
map server/AMCL/planner and planning action, real capture-time TF, source ages,
AI veto grid, saved/live map geometry, publisher counts, and the disabled zero
command gate. At least three distinct captures and a delivery p95 within the
300 ms odometry/scan and 500 ms vision budgets are needed; one lucky fresh frame
cannot hide a slow stream. The check does not depend on the legacy yellow
reference path and does not require a permission signal to be set true.

`Planning inputs ready: True` means the observed preview inputs met those checks.
It does not verify physical map accuracy or safe driving. Always inspect wall
alignment yourself. The command exits with status 2 when a required live check
is pending and keeps `Autonomous control verified: False` in every mode.

Then select the destination using **2D Goal Pose** and inspect the preview path.
The existing goal dispatcher accepts the map destination directly; no extra
calibration or invented destination coordinates are needed during preparation.

## 1. Survey and save a map

Keep the ROS1 gateway and SSH tunnel running. On Jazzy keep exactly one
`robot_link.launch.py base_frame:=base_link forward_commands:=false` running.
The robot gateway remains `allow_robot_output:=false`; keyboard teleop continues
through the robot's existing `/kb_teleop/cmd_vel` mux input. Stop the old AI
controller/robot_pipeline and any SLAM/AMCL session first. AI and camera views
can be stopped to free CPU during mapping.

Check the real scan is arriving, then launch:

```bash
ros2 topic hz /husky_link/scan
# Ctrl+C after observing live data
QT_QPA_PLATFORM=xcb ros2 launch husky_vision mapping.launch.py
```

This launch contains no robot command publisher. Drive manually, slowly, through
the allowed area using the already tested ROS1 keyboard teleop. Visit corridors
and junctions and return through previously seen places for loop closure. Watch
the map align with walls; a topic existing alone does not prove valid mapping.

Save from a separate terminal:

```bash
mkdir -p ~/husky_vision_ws/maps
ros2 run nav2_map_server map_saver_cli -f "$HOME/husky_vision_ws/maps/site1" --ros-args -p save_map_timeout:=10.0
```

Keep both `site1.yaml` and `site1.pgm` (or the image filename written in YAML).
Inspect coverage and wall alignment before using the map for localization.

The mapping adapter derives `angle_max` from the actual range array and replaces
a zero lower metadata bound with positive float epsilon; it keeps all measured
ranges, original timestamps, timing fields and no-return infinities unchanged.
It does **not** deskew VLP16, infer free-space from absent returns or certify an
obstacle scan. Its output `/husky_nav/mapping_scan` is for SLAM/AMCL/planning only.
The final collision filter still receives the original `/husky_link/scan` and
rejects the previously observed unsupported VLP16 metadata. The runtime VLP16
obstacle/coverage integration remains a real prerequisite for physical autonomy.

## 2. Start saved-map goal planning

Stop the mapping launch and standalone robot_link before starting navigation.
SLAM and AMCL must never both publish `map -> odom`. Also stop the earlier
ground_path/controller and manual `base_footprint -> husky_vision_ground` alias;
the navigation launch owns that alias and camera mount TF. Keep the gateway
and SSH tunnel.

Start the USB webcam and AI with the final calibration/mount as before. After
replugging/reopening the webcam, retain the calibrated focus (the tested camera
uses autofocus off, focus_absolute=120, zoom=0):

```bash
v4l2-ctl -d /dev/video2 --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d /dev/video2 --set-ctrl=focus_absolute=120
```

The detector must be the current version: it publishes a segmentation packet
only while subscribed, with original capture header, raw dimensions and a PNG
label image. It uses the full thresholded segmentation, not the seed-connected
yellow path, so destination planning can select a different branch.

Check `robot/robot.json` uses the chosen `camera-active.yaml`, current mount,
`odom`/`base_link`, real footprint (0.50 m front, 0.56 m rear, 0.33 m half-width
for this robot, plus its configured margin) and loopback bridge URL. Preview
does not require the unfinished motion-check flags to be set true.

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision navigation.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot.json" \
  map:="$HOME/husky_vision_ws/maps/site1.yaml" \
  allow_motion:=false
```

The launch starts one robot bridge using the **new private** final command
topic, map server, AMCL, Nav2 planner, AI mask projection, command gate and
collision filter. Controller/navigator servers are added only in motion mode.
It does not start another webcam or AI model.
The ground parent defaults to the real ROS1 `base_footprint` TF verified on this
Husky; if that TF is absent, resolve the actual ground anchor rather than adding
an identity odom transform. Only one publisher may own each ground/camera child.

In RViz:

1. Click **2D Pose Estimate** at the robot's actual position and drag its heading.
2. Check that scan points align with the saved walls as AMCL localizes.
3. Click **2D Goal Pose** on an allowed, mapped destination and drag the desired heading.
4. In planning-only mode, the dispatcher calls Nav2 `ComputePathToPose` and draws
   `/husky_nav/goal_plan`; it never starts a drive action. The status becomes
   `path_ready_preview_only`. A different goal cancels/replaces the old request.

```bash
ros2 topic echo /husky_nav/goal_status --once --full-length
ros2 topic echo /husky_nav/vision_status --once --full-length
ros2 topic echo /husky_nav/control_status --once --full-length
```

Goal planning needs live real TF/scan/odometry and fresh AI masks. If a server is
not ready, initialize pose and check its lifecycle/vision status before retrying
the goal. Unknown map cells are not traversed by the global planner.

## What AI changes and how commands are isolated

The AI grid is frozen into odom at **image capture time**. Cells with observed
non-drivable samples become cost 100; other cells stay unknown/no-veto (-1), not
free. Center and corner samples are used for each grid cell, and invalid rays or
distortion foldbacks are discarded. Unanalyzed image rows above the detector's
ROI remain unknown. Color labels alone do not certify traversability.

The C++ `VisionMaskLayer` adds only lethal costs before Nav2's inflation layer.
It never clears map/LiDAR obstacles or fills unknown map cells with free space.
Bounds include both previous and current mask extents, so moving a mask does not
leave stale ghost obstacles. A missing/old mask or unavailable TF makes the
costmap non-current; the downstream gate independently blocks stale vision.

The Nav2 controller publishes `TwistStamped` only on `/husky_nav/nav2_cmd_vel`.
It does not publish a global `/cmd_vel`. The navigation gate starts disabled;
planning-only launches reject attempts to enable it. Its output is the private
`/husky_nav/guarded_request`, consumed by the final collision filter. Only
`/husky_nav/filtered_cmd_vel_stamped` reaches the navigation-mode bridge. Old
yellow-path controller outputs cannot enter this chain. Do not run two bridge
clients/receivers or two competing motion pipelines.

## Physical motion is a separate field step

`allow_motion:=true` switches RViz goals to native `NavigateToPose`, but requires
the strict measured profile, explicit bridge forwarding, verified field-check
flags and a configured real robot-local deadman. The command gate still starts
disabled and requires fresh independent permission, vision, odometry, map TF
and original stamped Nav2 commands. The existing local receiver must be manually
armed and allowed to write to the verified mux input. Nothing arms automatically.

The VLP16 runtime obstacle integration, local permission source and physical
clearance/stopping checks are still pending for this robot. Do not mark those
fields true just because a map or a planned path looks correct. Completing this
software architecture does not certify those untested hardware conditions.

## Startup problems observed on the field computer

If RViz reports `xcb` loaded from a `cv2/qt/plugins` directory, OpenCV has
overridden the native GUI plugin path. The navigation launch clears OpenCV's Qt
plugin/font path variables **for RViz only**, preserving the camera/AI process
environment. After updating code, rebuild and restart the navigation launch.

An `undefined symbol` error in `libnav2_lifecycle_manager_core.so` can indicate
a mismatched native dependency even when all package-presence checks pass.
On the field machine Nav2 was newer while `diagnostic_updater` remained at
4.2.6 with 4.2.7 available. Inspect the actual loaded path and package versions:

```bash
ldd /opt/ros/jazzy/lib/libnav2_lifecycle_manager_core.so | grep -E 'diagnostic|not found'
apt-cache policy ros-jazzy-diagnostic-updater ros-jazzy-nav2-lifecycle-manager
```

When the ROS installation's diagnostic updater is behind the available version,
stop navigation and update that package on the Jazzy computer:

```bash
sudo apt update
sudo apt install --only-upgrade ros-jazzy-diagnostic-updater
```

Then rerun `navigation_preflight` and check `nav2_native_runtime`. If the loaded
dependency comes from another workspace, inspect the sourced environment before
retrying. Keep the robot stationary and the navigation launch in planning-only
mode while resolving startup failures.

Another field failure came from dynamically loaded Nav2 typesupport:
`libgeographic_msgs__rosidl_typesupport_fastrtps_cpp.so` could not resolve
`eprosima::fastcdr::Cdr::serialize`, and the C typesupport failed similarly.
This is outside the manager library's direct dependency graph, so checking
only the AMCL executable or manager core does not detect it. Preflight now
also checks both C/C++ transport libraries for `nav2_msgs` and `geographic_msgs`
using the selected RMW (Fast DDS by default, introspection for Cyclone DDS).
It does not switch RMW or alter library paths to hide the error.

Inspect the loaded Fast CDR path and package versions:

```bash
ldd /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_fastrtps_cpp.so | grep -E 'fastcdr|not found'
apt-cache policy ros-jazzy-fastcdr ros-jazzy-geographic-msgs
```

For outdated packages loaded from the ROS installation, stop navigation and
update the affected pair, then verify relocations again:

```bash
sudo apt update
sudo apt install --only-upgrade ros-jazzy-fastcdr ros-jazzy-geographic-msgs
ldd -r /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_fastrtps_cpp.so 2>&1 | grep -E 'undefined symbol|not found'
```

An empty final grep result means no such relocation errors were reported for
that library. Restart navigation to load the updated libraries; an existing
process may retain libraries from before the update. If dependencies load from
`/usr/local` or another workspace, inspect that environment first instead of
assuming the apt packages are the libraries actually in use.

## Software validation and its limits

The implementation was built against ROS 2 Jazzy/Nav2, including the exported
`husky_vision` and `husky_vision_nav2` packages. Validation includes 221 Python
tests and four native C++ layer tests covering obstacle preservation, previous
mask bounds, stale/invalid masks and frame transforms.

An isolated native Nav2 integration used synthetic map/TF/segmentation inputs:
the planner followed a straight route, detoured around an AI veto, then returned
to the straight route after that veto disappeared. A separate native
`NavigateToPose`/Regulated Pure Pursuit check verified original stamped private
requests, zero output from a disabled gate, no global `/cmd_vel` publisher, and
controller stop when AI data became stale. The navigator's built-in `goal_pose`
subscriber is remapped so a single RViz goal reaches only our dispatcher.

The preparation/live preflight adds thirteen tests, including preparation
without a map and rejection of stale AI, duplicate publishers, incomplete
configuration and inactive localization. Its ROS2 subscriptions, lifecycle
GetState calls and capture-time TF observations were also exercised in an
isolated Jazzy container with synthetic data and no robot connection.

These fixtures do not connect a robot bridge and do not test physical goal
arrival. Real SLAM map quality, AMCL localization, VLP16 motion coverage, local
permission and stopping behavior still require the field checks above.
