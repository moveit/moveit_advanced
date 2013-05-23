
catkin_add_gtest(
  test_collision_detection_distance_field
    test_collision_detection_distance_field.cpp
)

target_link_libraries(
  test_collision_detection_distance_field
    collision_detection_distance_field
    ${Boost_LIBRARIES}
)

find_package(moveit_resources QUIET)
set(MOVEIT_RESOURCES_DIR "${moveit_resources_SOURCE_DIR}")
set(TEST_RESOURCES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources")
configure_file("${TEST_RESOURCES_DIR}/config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/resources/config.h")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

