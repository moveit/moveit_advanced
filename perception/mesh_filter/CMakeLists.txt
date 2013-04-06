set(MOVEIT_LIB_NAME moveit_mesh_filter)

add_library(${MOVEIT_LIB_NAME}
  src/mesh_filter_base.cpp
  src/sensor_model.cpp
  src/stereo_camera_model.cpp
  src/gl_renderer.cpp
  src/gl_mesh.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${gl_LIBS} glut GLEW)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
