from glob import glob

from setuptools import find_packages, setup


setup(
    name="husky_drivable_area",
    version="0.1.0",
    packages=find_packages(include=("road_navigation", "road_navigation.*", "husky_drivable_area", "husky_drivable_area.*")),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/husky_drivable_area"]),
        ("share/husky_drivable_area", ["package.xml"]),
        ("share/husky_drivable_area/launch", glob("launch/*.launch.py")),
        ("share/husky_drivable_area/config", ["config.example.json"]),
        ("share/husky_drivable_area/config", glob("config/*")),
        ("share/husky_drivable_area/docs", ["docs/ros2-metric-path.md", "docs/ros2-control.md"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="husk maintainers",
    maintainer_email="jorjor140900@users.noreply.github.com",
    description="Drivable perception, calibrated projection and experimental path control",
    license="Unlicensed",
    entry_points={"console_scripts": [
        "drivable_area_node = husky_drivable_area.nodes:detector_main",
        "webcam_node = husky_drivable_area.nodes:camera_main",
        "ground_path_node = husky_drivable_area.ground_node:main",
        "import_camera_calibration = husky_drivable_area.calibration:main",
        "path_controller = husky_drivable_area.controller_node:main",
        "control_simulator = husky_drivable_area.control_simulator:main",
        "control_self_test = husky_drivable_area.control_self_test:main",
        "laser_safety = husky_drivable_area.safety_node:main",
        "vision_control_preview = husky_drivable_area.vision_preview_node:main",
        "ground_geometry_check = husky_drivable_area.ground_check_node:main",
    ]},
)
