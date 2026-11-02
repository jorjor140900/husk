"""Standalone assistance must not start mapping, goal or LaserScan consumers."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest import mock
from test_vision_field_launch import FieldLaunchTests
from husky_link_core.supervised_trial import MODE,TURN_GOAL_LIMITS


class CameraAssistLaunchTests(unittest.TestCase):
    def setUp(self):
        fixture=FieldLaunchTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        root=Path(fixture.temporary.name)
        self.context=dict(profile=str(fixture.profile),camera_device=str(fixture.device),allow_motion='false',
            ground_parent_frame='base_footprint',reference_hold_sec='2.0')
        profile=json.loads(fixture.profile.read_text())
        profile.update(operation_mode=MODE,trial_limits=TURN_GOAL_LIMITS,
            robot=dict(base_frame='base_link',odom_frame='odom',external_permission_topic='/husky_link/external_permission'),
            footprint=dict(half_width_m=.33,margin_m=.15,lateral_margin_m=.08),
            bridge=dict(url='ws://127.0.0.1:9090',forward_commands=True,expected_output_topic='/cmd_vel'))
        fixture.profile.write_text(json.dumps(profile))
        class Configuration:
            def __init__(self,name):self.name=name
            def perform(self,context):return context[self.name]
        stubs={'ament_index_python.packages':NS(get_package_share_directory=lambda _:str(root)),
            'launch':NS(LaunchDescription=list),'launch.actions':NS(DeclareLaunchArgument=NS,OpaqueFunction=NS,
                IncludeLaunchDescription=lambda source,launch_arguments:NS(source=source,arguments=dict(launch_arguments)),
                SetEnvironmentVariable=lambda *args:NS(environment=args)),
            'launch.launch_description_sources':NS(PythonLaunchDescriptionSource=lambda path:path),
            'launch.substitutions':NS(LaunchConfiguration=Configuration),'launch_ros.actions':NS(Node=lambda **kw:NS(**kw))}
        source=Path(__file__).parents[1]/'launch/camera_assist.launch.py'
        spec=importlib.util.spec_from_file_location('_assist_launch_test',source)
        self.module=importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules,stubs):spec.loader.exec_module(self.module)

    def test_camera_only_graph_has_no_map_goal_rviz_nav2_or_scan_consumer(self):
        rows=self.module._nodes(self.context)
        nodes={n.executable:n for n in rows if hasattr(n,'executable')}
        self.assertEqual(set(nodes),{'webcam_node','camera_assist'})
        self.assertFalse(nodes['camera_assist'].parameters[0]['allow_motion'])
        perception=next(n for n in rows if hasattr(n,'source'))
        self.assertIn('drivable_area.launch.py',perception.source)
        self.assertEqual(perception.arguments['show_nav2_path'],'false')


if __name__=='__main__':unittest.main()
