cmake_minimum_required(VERSION 2.8.3)
project(collision_detection_distance_field)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(
  catkin
  REQUIRED
  COMPONENTS
    console_bridge
    eigen
    eigen_conversions
    eigen_stl_containers
    geometric_shapes
    geometry_msgs
    moveit_core
    moveit_msgs
    rostime
    shape_msgs
    shape_tools
    std_msgs
    visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    collision_detection_distance_field
  CATKIN_DEPENDS
    console_bridge
    eigen_conversions
    eigen_stl_containers
    geometric_shapes
    moveit_msgs
    shape_tools
  DEPENDS
    Boost
    Eigen
)

###########
## Build ##
###########

include_directories(
  SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
)

link_directories(
    ${Boost_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

add_library(
  collision_detection_distance_field
    src/collision_common_distance_field.cpp
    src/collision_robot_distance_field.cpp
    src/collision_world_distance_field.cpp
    src/workarea.cpp
    src/params.cpp
    src/sphere_collision.cpp
    src/df_intragroup_collision.cpp
    src/static_distance_field.cpp
)

# This is needed to ensure that messages are done being built before this is built
add_dependencies(
  collision_detection_distance_field
    moveit_msgs_gencpp
)

target_link_libraries(
  collision_detection_distance_field
    moveit_collision_detection
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    collision_detection_distance_field
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

#############
## Testing ##
#############

add_subdirectory(test)
