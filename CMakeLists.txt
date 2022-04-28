cmake_minimum_required(VERSION 3.0.2)
project(tracking_pid)
find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/Params.cfg
)
catkin_package()
include_directories()
catkin_install_python(PROGRAMS
  scripts/tracking_pid_node.py
  scripts/vicon_to_odom_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
