"""Install only the camera-steering-assist executables and their assets."""
from glob import glob
from setuptools import find_packages, setup

setup(
    name="husky_drivable_area", version="0.1.0",
    packages=find_packages(include=("road_navigation", "road_navigation.*", "husky_drivable_area", "husky_drivable_area.*", "husky_link_core", "husky_link_core.*")),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/husky_drivable_area"]),
        ("share/husky_drivable_area", ["package.xml", "README.md"]),
        ("share/husky_drivable_area/launch", glob("launch/*.launch.py")),
        ("share/husky_drivable_area/config", glob("config/*")),
        ("share/husky_drivable_area/docs", glob("docs/*.md")),
        ("share/husky_drivable_area/ros1/husky_vision_bridge", ["ros1/husky_vision_bridge/package.xml", "ros1/husky_vision_bridge/CMakeLists.txt", "ros1/husky_vision_bridge/setup.py"]),
        ("share/husky_drivable_area/ros1/husky_vision_bridge/launch", glob("ros1/husky_vision_bridge/launch/*.launch")),
        ("share/husky_drivable_area/ros1/husky_vision_bridge/scripts", glob("ros1/husky_vision_bridge/scripts/*.py")),
        ("share/husky_drivable_area/ros1/husky_vision_bridge/src/husky_link_core", glob("husky_link_core/*.py")),
    ],
    install_requires=["setuptools", "numpy"], zip_safe=True,
    maintainer="husk maintainers", maintainer_email="jorjor140900@users.noreply.github.com",
    description="SegFormer drivable-area perception and keyboard-held steering assistance for Husky A200",
    license="Unlicensed",
    entry_points={"console_scripts": [
        "webcam_node = husky_drivable_area.nodes:camera_main",
        "drivable_area_node = husky_drivable_area.nodes:detector_main",
        "camera_assist = husky_drivable_area.camera_assist_node:main",
        "camera_assist_teleop = husky_drivable_area.assist_teleop:main",
        "laser_safety = husky_drivable_area.safety_node:main",
        "robot_bridge = husky_drivable_area.robot_bridge_node:main",
        "field_permission = husky_drivable_area.permission_node:main",
        "prepare_operator_profile = husky_drivable_area.operator_profile:main",
    ]},
)
