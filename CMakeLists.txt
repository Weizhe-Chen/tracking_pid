cmake_minimum_required(VERSION 3.0.2)
project(tracking_pid)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED
  rospy
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  )
catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# generate_dynamic_reconfigure_options(
#   cfg/Params.cfg
#   )
# catkin_install_python(PROGRAMS
#   scripts/tracking_pid_node.py
#   scripts/vicon_to_odom_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )

add_executable(pose_tracking_node src/pose_tracking_node.cpp)
target_link_libraries(pose_tracking_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS pose_tracking_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
