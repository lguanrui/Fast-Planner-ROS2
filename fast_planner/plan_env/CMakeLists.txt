cmake_minimum_required(VERSION 3.8)
project(plan_env)
cmake_policy(SET CMP0069 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies 
    rclcpp
    std_msgs
    nav_msgs
    visualization_msgs
    cv_bridge
    message_filters
    Eigen3
    PCL
    pcl_conversions
    OpenCV
)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env SHARED
    src/sdf_map.cpp 
    #src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
)
ament_target_dependencies(plan_env ${dependencies})
target_link_libraries(plan_env ${PCL_LIBRARIES})
#ament_export_targets(plan_env HAS_LIBRARY_TARGET)

#add_executable(obj_generator src/obj_generator.cpp)
#ament_target_dependencies(obj_generator ${dependencies})
#target_link_libraries(obj_generator plan_env)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS plan_env 
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

#install(TARGETS obj_generator 
#    LIBRARY DESTINATION lib
#    ARCHIVE DESTINATION lib
#    RUNTIME DESTINATION bin
#    INCLUDES DESTINATION include
#)

ament_package()