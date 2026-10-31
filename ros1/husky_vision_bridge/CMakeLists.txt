cmake_minimum_required(VERSION 3.0.2)
project(husky_vision_bridge)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/robot_gateway.py scripts/robot_inventory.py scripts/operator_console.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
