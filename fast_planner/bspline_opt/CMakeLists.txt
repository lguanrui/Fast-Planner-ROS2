# Minimum CMake version required
cmake_minimum_required(VERSION 3.8)

# Project name
project(bspline_opt)

# Set CMake policy
cmake_policy(SET CMP0069 NEW)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)
set(NLOPT_INCLUDE_DIR "/usr/local/include")  
set(NLOPT_LIBRARY "/usr/local/lib/libnlopt.so")  

# List of dependencies
set(dependencies 
    plan_env
    Eigen3
)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIR} ${NLOPT_INCLUDE_DIR})

# Add library target
add_library(bspline_opt SHARED src/bspline_optimizer.cpp)

# Link dependencies to the target
ament_target_dependencies(bspline_opt ${dependencies})
target_link_libraries(bspline_opt ${NLOPT_LIBRARY})
ament_export_targets(bspline_opt HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

# Install the library
install(DIRECTORY include/ DESTINATION include)
install(
    TARGETS bspline_opt
    EXPORT bspline_opt
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