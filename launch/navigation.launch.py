"""Saved-map goals + AI costmaps; private commands, planning-only by default."""
from pathlib import Path
import json
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    from husky_drivable_area.navigation import navigation_parameters, saved_map_metadata, guide_obstacle_source
    from husky_drivable_area.ground_geometry import CameraCalibration, CameraMount
    from husky_drivable_area.robot_profile import load_robot_profile, expanded_path
    share = Path(get_package_share_directory("husky_drivable_area"))
    # The layer must actually be installed; absence cannot silently disable AI.
    get_package_share_directory("husky_vision_nav2")
    filename = Path(LaunchConfiguration("profile").perform(context)).expanduser()
    profile = json.loads(filename.read_text())
    for key in ("calibration_path", "mount_path"):
        profile["camera"][key] = str(expanded_path(profile["camera"][key]))
    allow = LaunchConfiguration("allow_motion").perform(context).lower() == "true"
    trial = LaunchConfiguration("supervised_trial").perform(context).lower() == "true"
    from husky_link_core.supervised_trial import trial_mode, profile_trial_limits, GOAL_LIMITS, EXTENDED_GOAL_LIMITS, LONG_GOAL_LIMITS
    trial_mode(profile, trial)
    limits=profile_trial_limits(profile) if trial else None
    extended=trial and limits in (EXTENDED_GOAL_LIMITS,LONG_GOAL_LIMITS)
    goal_session=trial and (limits==GOAL_LIMITS or extended)
    trial_parameters=dict(supervised_trial=trial,trial_extended_session=extended)
    reference_parameters=dict(trial_goal_session=goal_session,trial_extended_duration_sec=limits['max_duration_sec'] if extended else 300.)
    mode = LaunchConfiguration("steering_mode").perform(context)
    if mode not in ("nav2", "vision"):
        raise ValueError("steering_mode must be nav2 or vision")
    vision = mode == "vision"
    guide_source=guide_obstacle_source(profile,vision,trial)
    if trial and not vision:raise ValueError("supervised_trial requires vision steering")
    if allow:
        if profile.get("bridge", {}).get("forward_commands") is not True:
            raise ValueError("motion mode requires a completed, explicitly enabled field robot profile")
        profile, mount = load_robot_profile(filename, allow_supervised_trial=trial)
    else:
        mount = CameraMount.load(profile["camera"]["mount_path"])
    camera, robot, bridge = profile["camera"], profile["robot"], profile["bridge"]
    CameraCalibration.load(camera["calibration_path"])
    map_file = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    saved_map_metadata(map_file)
    template = yaml.safe_load((share/"config/navigation.yaml").read_text())
    params = navigation_parameters(profile,template,map_file,share/"config/navigate_to_pose.xml")
    if vision:
        # The global route is a coarse map/LiDAR direction guide only. Current
        # image labels constrain the actual local steering reference directly.
        costmap=params["global_costmap"]["global_costmap"]["ros__parameters"]
        costmap["plugins"].remove("vision_layer")
        if guide_source=="saved_map_only":
            costmap["plugins"].remove("obstacle_layer")
    with tempfile.NamedTemporaryFile("w",prefix="husky-nav2-",suffix=".yaml",delete=False) as target:
        yaml.safe_dump(params,target)
        parameters = target.name
    base, odom = robot["base_frame"], robot["odom_frame"]
    rviz_file=str(share/"config/navigation.rviz")
    if vision:
        view=yaml.safe_load(Path(rviz_file).read_text())
        for display in view["Visualization Manager"]["Displays"]:
            if display.get("Name") in ("Goal preview plan","Navigation plan"):
                display["Enabled"]=False
        with tempfile.NamedTemporaryFile("w",prefix="husky-vision-rviz-",suffix=".rviz",delete=False) as target:
            yaml.safe_dump(view,target);rviz_file=target.name
    parent = LaunchConfiguration("ground_parent_frame").perform(context)
    if parent == mount.ground_frame:
        raise ValueError("ground parent and child must differ")
    nodes = [
        Node(package="husky_drivable_area",executable="robot_bridge",namespace="husky_link",output="screen",parameters=[{
            "bridge_url":bridge["url"],"forward_commands":allow,"base_frame":base,**trial_parameters,
            "expected_output_topic":bridge.get("expected_output_topic") or "",
            "command_topic":"/husky_nav/filtered_cmd_vel_stamped"}]),
        Node(package="husky_drivable_area",executable="mapping_scan",output="screen"),
        Node(package="tf2_ros",executable="static_transform_publisher",name="navigation_ground_alias",
             arguments=["--x","0","--y","0","--z","0","--roll","0","--pitch","0","--yaw","0",
                        "--frame-id",parent,"--child-frame-id",mount.ground_frame]),
        Node(package="husky_drivable_area",executable="vision_grid",output="screen",parameters=[{
            "camera_calibration":camera["calibration_path"],"camera_mount":camera["mount_path"],"odom_frame":odom}]),
        Node(package="husky_drivable_area",executable="field_permission",namespace="husky_control",output="screen",
             parameters=[{"external_topic":robot["external_permission_topic"]}]),
        Node(package="husky_drivable_area",executable="navigation_gate",output="screen",
             parameters=[{"allow_motion":allow,"base_frame":base,"odom_frame":odom,"guide_obstacle_source":guide_source,**trial_parameters}],
             remappings=[("/husky_nav/nav2_cmd_vel","/husky_nav/vision_cmd_vel"),
                         ("/husky_nav/vision_status","/husky_nav/vision_control_status")] if vision else []),
        Node(package="husky_drivable_area",executable="goal_dispatch",output="screen",
             parameters=[{"allow_motion":allow,"guide_only":vision}]),
        Node(package="husky_drivable_area",executable="laser_safety",namespace="husky_control",output="screen",
             parameters=[str(share/"config/laser_safety.yaml"),profile["footprint"],
                         {"base_frame":base,"odom_frame":odom,"max_linear_mps":.05,"max_angular_rps":.2,**trial_parameters,**reference_parameters}],
             remappings=[("odom","/husky_link/odom"),("scan","/husky_link/scan"),
                         ("control/requested_cmd_vel","/husky_nav/guarded_request"),
                         ("control/cmd_vel","/husky_nav/filtered_cmd_vel"),
                         ("control/cmd_vel_stamped","/husky_nav/filtered_cmd_vel_stamped")]),
    ]
    if vision:
        nodes.append(Node(package="husky_drivable_area",executable="vision_goal_controller",output="screen",parameters=[{
            "camera_calibration":camera["calibration_path"],"camera_mount":camera["mount_path"],
            "base_frame":base,"odom_frame":odom,"half_width_m":profile["footprint"]["half_width_m"],
            "side_margin_m":profile["footprint"].get("lateral_margin_m",profile["footprint"]["margin_m"]),
            "image_topic":LaunchConfiguration("image_topic").perform(context),**trial_parameters,**reference_parameters}]))
    if camera.get("publish_camera_tf",True):
        q = mount.quaternion_xyzw
        nodes.append(Node(package="tf2_ros",executable="static_transform_publisher",name="navigation_camera_mount",
            arguments=["--x",str(mount.camera_x_m),"--y",str(mount.camera_y_m),"--z",str(mount.height_m),
                       "--qx",str(q[0]),"--qy",str(q[1]),"--qz",str(q[2]),"--qw",str(q[3]),
                       "--frame-id",mount.ground_frame,"--child-frame-id",mount.camera_frame]))
    for package, executable, name in (("nav2_map_server","map_server","map_server"),
            ("nav2_amcl","amcl","amcl"),("nav2_controller","controller_server","controller_server"),
            ("nav2_planner","planner_server","planner_server"),("nav2_bt_navigator","bt_navigator","bt_navigator")):
        if (not allow or vision) and name in ("controller_server", "bt_navigator"):
            continue
        remappings = []
        if name == "controller_server":
            remappings = [("cmd_vel", "/husky_nav/nav2_cmd_vel")]
        elif name == "bt_navigator":
            # Native Nav2 also subscribes to goal_pose. Only our dispatcher
            # should consume RViz goals, otherwise one click sends two actions.
            remappings = [("goal_pose", "/husky_nav/native_goal_input")]
        nodes.append(Node(package=package,executable=executable,name=name,output="screen",parameters=[parameters],
                          remappings=remappings))
    for name, managed in (("localization",["map_server","amcl"]),
                          ("navigation",["controller_server","planner_server","bt_navigator"] if allow and not vision else ["planner_server"])):
        nodes.append(Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_"+name,
                          output="screen",parameters=[{"autostart":True,"node_names":managed,"use_sim_time":False}]))
    nodes.append(Node(package="rviz2",executable="rviz2",arguments=["-d",rviz_file],
                      # Importing GUI OpenCV above can replace these variables
                      # with its bundled Qt. Native RViz must use system Qt.
                      additional_env={"QT_QPA_PLATFORM_PLUGIN_PATH":"", "QT_QPA_FONTDIR":""},
                      condition=IfCondition(LaunchConfiguration("rviz"))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile",description="Measured robot/robot.json"),
        DeclareLaunchArgument("map",description="Previously saved map YAML"),
        DeclareLaunchArgument("allow_motion",default_value="false"),
        DeclareLaunchArgument("supervised_trial",default_value="false"),
        DeclareLaunchArgument("steering_mode",default_value="nav2",description="vision = image/AI steering, map goal guidance only"),
        DeclareLaunchArgument("image_topic",default_value="/camera/image_raw"),
        DeclareLaunchArgument("ground_parent_frame",default_value="base_footprint",description="Real robot TF ground anchor"),
        DeclareLaunchArgument("rviz",default_value="true"),OpaqueFunction(function=_nodes)])
