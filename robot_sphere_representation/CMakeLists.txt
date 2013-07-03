cmake_minimum_required(VERSION 2.8.3)
project(robot_sphere_representation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin
  REQUIRED COMPONENTS
    mesh_core
    moveit_core
    moveit_setup_assistant
    roscpp
    srdfdom
    std_msgs
)

## System dependencies are found with CMake's conventions
find_package(
  Boost
  REQUIRED COMPONENTS
    system
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    moveit_robot_sphere_representation
  CATKIN_DEPENDS
    mesh_core
    moveit_core
    moveit_setup_assistant
    roscpp
    srdfdom
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(MOVEIT_LIB_NAME moveit_robot_sphere_representation)
add_library(
  ${MOVEIT_LIB_NAME}
    src/robot_sphere_representation.cpp
    src/link_sphere_representation.cpp
    src/sphere_rep.cpp
    src/method_enums.cpp
    src/body_bounding_sphere.cpp
)

target_link_libraries(
  ${MOVEIT_LIB_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(
  TARGETS
    ${MOVEIT_LIB_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  
install(
  DIRECTORY
    include/
  DESTINATION
    include
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_sphere_representation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
