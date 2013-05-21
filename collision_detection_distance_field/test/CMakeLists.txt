
catkin_add_gtest(
  test_collision_detection_distance_field
    test_collision_detection_distance_field.cpp
)

target_link_libraries(
  test_collision_detection_distance_field
    collision_detection_distance_field
    ${Boost_LIBRARIES}
)

