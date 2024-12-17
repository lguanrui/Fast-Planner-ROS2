cmake_minimum_required(VERSION 3.8)
project(bspline)
cmake_policy(SET CMP0069 NEW)

# We need c++14 for ROS2, thus just require it for everybody
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") #C++ compile option
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
# Enable debug flags (use if you want to debug in gdb)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
    Eigen3
)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(bspline SHARED src/non_uniform_bspline.cpp)
ament_target_dependencies(bspline ${dependencies})
ament_export_targets(bspline HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS bspline 
  EXPORT bspline 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()