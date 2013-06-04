catkin_add_gtest(mesh_filter_test src/mesh_filter_test.cpp)

target_link_libraries(mesh_filter_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_mesh_filter)

