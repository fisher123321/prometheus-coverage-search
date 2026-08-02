cmake_minimum_required(VERSION 3.0.2)
project(prometheus_two_uav_coverage_search)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  laser_geometry
  pcl_ros
  visualization_msgs
  tf2
  tf2_ros
  octomap_server
  prometheus_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_path(NLOPT_INCLUDE_DIR nlopt.hpp HINTS /usr/local/include /home/wjy/nlopt/build REQUIRED)
find_library(NLOPT_LIBRARY nlopt HINTS /usr/local/lib /home/wjy/nlopt/build REQUIRED)

add_message_files(FILES
  SwarmState.msg
  SwarmMapChunk.msg
  SwarmMapRequest.msg
  SwarmFrontier.msg
  SwarmFrontierArray.msg
  SwarmBid.msg
  SwarmBidArray.msg
  SwarmTask.msg
  SwarmTaskArray.msg
  SwarmTrajectory.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES two_uav_coverage_lib
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs laser_geometry pcl_ros visualization_msgs tf2 tf2_ros octomap_server prometheus_msgs message_runtime
  DEPENDS PCL
)

include_directories(
  SYSTEM
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})

# Local copy of the complete single-UAV coverage stack.  The coordination
# nodes below are additions; they do not depend on a separately running node.
add_library(two_uav_coverage_lib
  src/two_uav_astar_2d.cpp
  src/two_uav_coverage_map.cpp
  src/two_uav_frontier_finder.cpp
  src/two_uav_hierarchical_grid.cpp
  src/two_uav_coverage_search_manager.cpp
  src/two_uav_coverage_search_planning.cpp
  src/two_uav_coverage_search_trajectory.cpp
)
add_dependencies(two_uav_coverage_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_uav_coverage_lib ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${NLOPT_LIBRARY})

add_executable(two_uav_coverage_search_node src/two_uav_coverage_search_node.cpp)
add_dependencies(two_uav_coverage_search_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_uav_coverage_search_node two_uav_coverage_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(two_uav_depth_cloud_downsample_node src/two_uav_depth_cloud_downsample_node.cpp)
add_dependencies(two_uav_depth_cloud_downsample_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_uav_depth_cloud_downsample_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(two_uav_octomap_server_node src/two_uav_octomap_server_node.cpp)
add_dependencies(two_uav_octomap_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_uav_octomap_server_node ${catkin_LIBRARIES})

add_executable(two_uav_fov_visualizer src/two_uav_fov_visualizer.cpp)
add_dependencies(two_uav_fov_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_uav_fov_visualizer ${catkin_LIBRARIES})

add_executable(two_uav_swarm_bridge src/two_uav_swarm_bridge.cpp)
add_dependencies(two_uav_swarm_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_uav_swarm_bridge ${catkin_LIBRARIES} zmq zmqpp)

add_executable(two_uav_coordinator src/two_uav_coordinator.cpp)
add_dependencies(two_uav_coordinator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_uav_coordinator ${catkin_LIBRARIES})

install(TARGETS two_uav_coverage_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS two_uav_coverage_search_node two_uav_depth_cloud_downsample_node two_uav_octomap_server_node two_uav_fov_visualizer
  two_uav_swarm_bridge two_uav_coordinator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ launch config docs rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/check_clock_offset.sh scripts/configure_chrony_peer.sh
  scripts/start_swarm_bridge.sh scripts/start_two_uav_coverage.sh
  scripts/start_two_uav_sim.sh scripts/arm_two_uav_sim.sh scripts/start_two_uav_coverage_sim.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
