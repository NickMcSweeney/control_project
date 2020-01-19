cmake_minimum_required(VERSION 2.8.3)
project(r2d2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(pid_controller SHARED src/pid_controller.cpp)
target_link_libraries(pid_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(r2d2_controller SHARED src/r2d2_controller.cpp)
target_link_libraries(r2d2_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
