cmake_minimum_required(VERSION 2.8.3)
project(moveit_advanced_perception)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

include(${PROJECT_SOURCE_DIR}/cmake/moveit_find_sse.cmake)
moveit_check_for_sse()

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
if (OPENGL_FOUND)
  set(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES})
endif(OPENGL_FOUND)

find_package(Boost REQUIRED thread)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_perception
  roscpp
  image_transport
  pluginlib
  )

find_package(OpenMP)
find_package(octomap REQUIRED)

catkin_package(
   INCLUDE_DIRS
     mesh_filter/include
     depth_image_octomap_updater/include
     ${OCTOMAP_INCLUDE_DIRS}
   LIBRARIES
     moveit_mesh_filter
     moveit_depth_image_octomap_updater_core
     ${OCTOMAP_LIBRARIES}
   CATKIN_DEPENDS
     moveit_ros_perception
     image_transport
     )

include_directories(mesh_filter/include
                    depth_image_octomap_updater/include
                    ${catkin_INCLUDE_DIRS} 
                    )
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
		    ${GLEW_INCLUDE_DIR}
		    ${GLUT_INCLUDE_DIR}
 		    )
 
 link_directories(${Boost_LIBRARY_DIRS})
 link_directories(${catkin_LIBRARY_DIRS})
 
add_subdirectory(mesh_filter)
add_subdirectory(depth_image_octomap_updater)

install(FILES
    depth_image_octomap_updater_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
