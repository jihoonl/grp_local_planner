cmake_minimum_required(VERSION 2.8.3)
project(grp_local_planner)

find_package(catkin REQUIRED roscpp base_local_planner nav_core costmap_2d cmake_modules geometry_msgs nav_msgs tf gaussian_process)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grp_local_planner_ros
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
  

add_library(grp_local_planner_ros
  src/grp_local_planner_ros.cpp
)

target_link_libraries(grp_local_planner_ros
  ${catkin_LIBRARIES}
)

install(TARGETS grp_local_planner_ros 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  grp_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
