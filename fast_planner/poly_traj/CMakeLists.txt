# Minimum CMake version required
cmake_minimum_required(VERSION 3.8)

# Project name
project(poly_traj)

# Set CMake policy
cmake_policy(SET CMP0069 NEW)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# List of dependencies
set(dependencies 
  Eigen3
)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIR})

# Add library target
add_library(poly_traj SHARED src/polynomial_traj.cpp)

# Link dependencies to the target
ament_target_dependencies(poly_traj ${dependencies})
ament_export_targets(poly_traj HAS_LIBRARY_TARGET)

# Install the library
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS poly_traj
  EXPORT poly_traj
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Export include directories and dependencies
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

# Package the project
ament_package()
