cmake_minimum_required(VERSION 3.8)
project(traj_utils)
cmake_policy(SET CMP0069 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(poly_traj REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies 
    Eigen3
    bspline
    bspline_opt
    poly_traj
    visualization_msgs
    geometry_msgs
)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

add_library(traj_utils SHARED
    src/planning_visualization.cpp
)
ament_target_dependencies(traj_utils ${dependencies})
target_link_libraries(traj_utils ${Eigen3_LIBRARIES})
ament_export_targets(traj_utils HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS traj_utils
    EXPORT traj_utils
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_package()