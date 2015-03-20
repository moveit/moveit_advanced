cmake_minimum_required(VERSION 2.8.3)
project(moveit_workspace_analysis)


find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    moveit_workspace_analysis
  CATKIN_DEPENDS 
    moveit_msgs
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    Boost
    Eigen
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIR}
                           )

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(moveit_workspace_analysis SHARED
   src/workspace_analysis.cpp
)

add_dependencies(moveit_workspace_analysis ${catkin_EXPORTED_TARGETS})

target_link_libraries(moveit_workspace_analysis ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(workspace_analysis_node src/main.cpp)
add_executable(workspace_reader_node src/reader.cpp)

target_link_libraries(workspace_analysis_node
  moveit_workspace_analysis ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(workspace_reader_node
  moveit_workspace_analysis ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(
  TARGETS
    moveit_workspace_analysis
    workspace_analysis_node
    workspace_reader_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


