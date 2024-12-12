cmake_minimum_required(VERSION 3.8)
project(fast_planner_msgs)
cmake_policy(SET CMP0069 NEW)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# We need c++14 for ROS2, thus just require it for everybody
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
    std_msgs
    geometry_msgs
    rosidl_default_generators
    nav_msgs
)

set(message_list
  "msg/Bspline.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
                          ${message_list}                          
                          ${service_list}
                          ${action_list}
                          DEPENDENCIES ${dependencies} )

ament_export_dependencies(${dependencies})

ament_package()

