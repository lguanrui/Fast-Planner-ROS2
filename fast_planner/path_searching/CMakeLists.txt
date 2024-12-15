cmake_minimum_required(VERSION 3.8)
project(path_searching)
cmake_policy(SET CMP0069 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)

set(dependencies 
    Eigen3
    plan_env
)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

add_library(path_searching SHARED
    src/kinodynamic_astar.cpp 
    #src/astar.cpp
    #src/topo_prm.cpp
)
ament_target_dependencies(path_searching ${dependencies})
target_link_libraries(path_searching ${PCL_LIBRARIES})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS path_searching 
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_package()