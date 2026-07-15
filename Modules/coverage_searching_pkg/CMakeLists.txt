cmake_minimum_required(VERSION 2.8.3)
project(prometheus_coverage_search)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
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
  prometheus_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  SYSTEM
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(coverage_search_lib
  src/astar_2d.cpp
  src/coverage_map.cpp
  src/frontier_finder.cpp
  src/hierarchical_grid.cpp
  src/coverage_search_manager.cpp
  src/coverage_search_planning.cpp
  src/coverage_search_trajectory.cpp
)
target_link_libraries(coverage_search_lib ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(coverage_search_node src/coverage_search_node.cpp)
add_dependencies(coverage_search_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_search_node coverage_search_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(depth_cloud_downsample_node src/depth_cloud_downsample_node.cpp)
add_dependencies(depth_cloud_downsample_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_cloud_downsample_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
