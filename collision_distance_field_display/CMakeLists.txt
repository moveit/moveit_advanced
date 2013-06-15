cmake_minimum_required(VERSION 2.8.3)
project(collision_distance_field_display)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread date_time system filesystem)

# Ogre
pkg_check_modules(OGRE OGRE)
link_directories( ${OGRE_LIBRARY_DIRS} )

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)


# Find ROS packages
find_package(catkin REQUIRED COMPONENTS
  collision_detection_distance_field
  moveit_ros_visualization
  moveit_ros_robot_interaction
  eigen_conversions
  geometric_shapes
  interactive_markers
  class_loader
  rviz
  tf
  robot_sphere_representation
  roscpp
  rosconsole
)

set(LIBNAME collision_distance_field_display)

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
    ${LIBNAME}
  CATKIN_DEPENDS
    moveit_ros_visualization
    moveit_ros_robot_interaction
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


qt4_wrap_cpp(
  MOC_SOURCES
    include/collision_distance_field_display/collision_distance_field_display.h
    include/collision_distance_field_display/df_link.h
    include/collision_distance_field_display/per_link_object.h
)


add_library(
  ${LIBNAME}
    src/collision_distance_field_display_plugin.cpp
    src/collision_distance_field_display.cpp
    src/df_link.cpp
    src/per_link_data.cpp
    src/sphere_rep_properties.cpp
    src/per_link_object.cpp
    src/dfexamine.cpp
    src/cylinders_display.cpp
    src/spheres_display.cpp
    src/points_display.cpp
    src/color_cast.cpp
    ${MOC_SOURCES}
)

target_link_libraries(
  ${LIBNAME}
    ${catkin_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES}
)



install(
  DIRECTORY
    include/
  DESTINATION
    include)

install(
  TARGETS
    ${LIBNAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



## Declare a cpp library
# add_library(collision_distance_field_display
#   src/${PROJECT_NAME}/collision_distance_field_display.cpp
# )

## Declare a cpp executable
# add_executable(collision_distance_field_display_node src/collision_distance_field_display_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(collision_distance_field_display_node collision_distance_field_display_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(collision_distance_field_display_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(
  FILES
    collision_distance_field_display_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY
    icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS collision_distance_field_display collision_distance_field_display_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_collision_distance_field_display.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
