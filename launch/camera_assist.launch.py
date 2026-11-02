"""Camera + segmentation + held-forward assist. No map, Nav2, scan or RViz."""
import json
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    from husky_drivable_area.ground_geometry import CameraMount
    from husky_drivable_area.robot_profile import expanded_path,load_robot_profile
    from husky_link_core.supervised_trial import trial_mode,profile_trial_limits,GOAL_LIMITS,EXTENDED_GOAL_LIMITS,LONG_GOAL_LIMITS,TURN_GOAL_LIMITS
    value=lambda key:LaunchConfiguration(key).perform(context)
    share=Path(get_package_share_directory('husky_drivable_area'))
    filename=expanded_path(value('profile'));profile=json.loads(filename.read_text())
    allow=value('allow_motion').lower()=='true'
    trial_mode(profile,True)
    limits=profile_trial_limits(profile)
    extended=limits in (EXTENDED_GOAL_LIMITS,LONG_GOAL_LIMITS,TURN_GOAL_LIMITS)
    if allow:profile,mount=load_robot_profile(filename,allow_supervised_trial=True)
    else:mount=CameraMount.load(expanded_path(profile['camera']['mount_path']))
    camera,robot,bridge=profile['camera'],profile['robot'],profile['bridge']
    if camera.get('source')!='local_usb':raise ValueError('camera assist expects the calibrated local USB camera')
    device=expanded_path(value('camera_device'));match=re.fullmatch(r'video([0-9]+)',device.name)
    if not device.exists() or match is None:raise ValueError('camera_device must resolve to the connected /dev/videoN')
    if allow and bridge.get('forward_commands') is not True:raise ValueError('motion requires an explicitly enabled supervised profile')
    parent=value('ground_parent_frame')
    if parent==mount.ground_frame:raise ValueError('ground parent and child must differ')
    for key in ('calibration_path','mount_path','model_path'):camera[key]=str(expanded_path(camera[key]))
    base,odom=robot['base_frame'],robot['odom_frame']
    common=dict(supervised_trial=True,trial_extended_session=extended)
    sessions=dict(trial_goal_session=limits==GOAL_LIMITS or extended,
        trial_extended_duration_sec=float(limits['max_duration_sec']) if extended else 300.,
        trial_turning_session=limits==TURN_GOAL_LIMITS)
    perception=dict(use_webcam='false',image_topic='/camera/image_raw',device='cuda',
        model_path=camera['model_path'],model_width='256',model_height='160',cpu_threads='2',
        camera_calibration=camera['calibration_path'],camera_mount=camera['mount_path'],
        config_file=str(share/'config/outdoor-supported-green-stable.json'),
        show_nav2_path='false',async_reference_preview='false',max_frame_age_sec='0.5')
    nodes=[SetEnvironmentVariable('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp'),
        Node(package='husky_drivable_area',executable='webcam_node',output='screen',parameters=[dict(
            camera_index=int(match.group(1)),width=camera['width'],height=camera['height'],fps=camera['fps'],
            frame_id=mount.camera_frame,image_topic='/camera/image_raw')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/drivable_area.launch.py')),
            launch_arguments=perception.items()),
        Node(package='husky_drivable_area',executable='robot_bridge',namespace='husky_link',output='screen',parameters=[dict(
            bridge_url=bridge['url'],forward_commands=allow,base_frame=base,
            expected_output_topic=bridge.get('expected_output_topic') or '',
            command_topic='/husky_assist/filtered_cmd_vel_stamped',**common)]),
        Node(package='tf2_ros',executable='static_transform_publisher',name='assist_ground_alias',arguments=[
            '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0',
            '--frame-id',parent,'--child-frame-id',mount.ground_frame]),
        Node(package='husky_drivable_area',executable='field_permission',namespace='husky_control',output='screen',
            parameters=[dict(external_topic=robot['external_permission_topic'])]),
        Node(package='husky_drivable_area',executable='camera_assist',output='screen',parameters=[dict(
            allow_motion=allow,camera_calibration=camera['calibration_path'],camera_mount=camera['mount_path'],
            base_frame=base,odom_frame=odom,speed_mps=min(.15,limits['max_linear_mps']),max_angular_rps=min(.15,limits['max_angular_rps']),
            half_width_m=profile['footprint']['half_width_m'],
            side_margin_m=profile['footprint'].get('lateral_margin_m',profile['footprint']['margin_m']),
            reference_hold_sec=float(value('reference_hold_sec')))]),
        Node(package='husky_drivable_area',executable='laser_safety',namespace='husky_control',output='screen',
            parameters=[str(share/'config/laser_safety.yaml'),profile['footprint'],dict(base_frame=base,odom_frame=odom,
                subscribe_scan=False,pause_on_vision_gap=True,**common,**sessions)],remappings=[
                ('odom','/husky_link/odom'),('control/requested_cmd_vel','/husky_assist/guarded_request'),
                ('control/cmd_vel','/husky_assist/filtered_cmd_vel'),('control/cmd_vel_stamped','/husky_assist/filtered_cmd_vel_stamped'),
                ('/husky_nav/control_status','/husky_assist/gate_status'),('/husky_nav/reset_trial','/husky_assist/reset_trial')])]
    if not allow:
        # A laptop-only preview needs neither a robot connection nor fake TF.
        nodes=[nodes[0],nodes[1],nodes[2],nodes[6]]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile',description='Existing supervised robot profile, e.g. robot-nojoy-junction.json'),
        DeclareLaunchArgument('allow_motion',default_value='false'),
        DeclareLaunchArgument('camera_device',default_value='/dev/v4l/by-id/usb-Generic_Web_Camera_200901010001-video-index0'),
        DeclareLaunchArgument('ground_parent_frame',default_value='base_footprint'),
        DeclareLaunchArgument('reference_hold_sec',default_value='2.0'),OpaqueFunction(function=_nodes)])
