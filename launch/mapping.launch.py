"""Manual-survey SLAM, with no controller or robot command publisher."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("husky_drivable_area"))
    slam = Path(get_package_share_directory("slam_toolbox"))
    rviz = share/"config/navigation.rviz"
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="husky_drivable_area", executable="mapping_scan", output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/"launch/online_async_launch.py")),
            launch_arguments={"use_sim_time": "false", "slam_params_file": str(share/"config/slam.yaml")}.items()),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(rviz)],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
