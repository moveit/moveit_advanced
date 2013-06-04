cmake_minimum_required(VERSION 2.8.3)
project(moveit_advanced_perception_tests)
find_package(catkin REQUIRED COMPONENTS moveit_core)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

## Declare things to be passed to other projects
#catkin_package()

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_subdirectory(mesh_filter)
