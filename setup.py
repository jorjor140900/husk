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
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="husk maintainers",
    maintainer_email="jorjor140900@users.noreply.github.com",
    description="Drivable masks and image-pixel references; no motor control",
    license="Unlicensed",
    entry_points={"console_scripts": [
        "drivable_area_node = husky_drivable_area.nodes:detector_main",
        "webcam_node = husky_drivable_area.nodes:camera_main",
    ]},
)
