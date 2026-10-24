"""Standalone road detection entry point. Run --help; ROS is not required.

The original ROS 2 implementation is preserved in legacy/RoadDetection_ros2.py.
"""

from road_navigation.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
