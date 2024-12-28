cmake_minimum_required(VERSION 3.8)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fast_planner_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  fast_planner_msgs
  quadrotor_msgs
  visualization_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  cv_bridge
  Eigen3
  PCL
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(kino_replan_fsm SHARED
  src/kino_replan_fsm_node.cpp src/planner_manager.cpp)
set_target_properties(kino_replan_fsm PROPERTIES
  COMPILE_DEFINITIONS "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(kino_replan_fsm ${dependencies})
rclcpp_components_register_nodes(kino_replan_fsm "fast_planner::KinoReplanFSM")
ament_export_libraries(kino_replan_fsm)
ament_export_dependencies(${dependencies})

add_library(bspline_tracker_node SHARED
  src/bspline_tracker_node.cpp)
set_target_properties(bspline_tracker_node PROPERTIES
  COMPILE_DEFINITIONS "COMPOSITION_BUILDING_DLL"
) 
ament_target_dependencies(bspline_tracker_node ${dependencies})
rclcpp_components_register_nodes(bspline_tracker_node "fast_planner::BsplineTrackerNode")
ament_export_libraries(bspline_tracker_node)
ament_export_dependencies(${dependencies})

install(TARGETS
  kino_replan_fsm bspline_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

ament_package()
