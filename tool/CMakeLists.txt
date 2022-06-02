set(V4L2_CAMREA_SHM_FILES
  v4l2_camera_shm8k_node
  v4l2_camera_shm512k_node
  v4l2_camera_shm1m_node
  v4l2_camera_shm2m_node
  v4l2_camera_shm4m_node
  v4l2_camera_shm8m_node
  v4l2_camera_image8k_subscriber
  v4l2_camera_image512k_subscriber
  v4l2_camera_image1m_subscriber
  v4l2_camera_image2m_subscriber
  v4l2_camera_image4m_subscriber
  v4l2_camera_image8m_subscriber
)

foreach(V4L2_CAMREA_SHM_EXE ${V4L2_CAMREA_SHM_FILES})
  add_executable(${V4L2_CAMREA_SHM_EXE} ${V4L2_CAMREA_SHM_EXE}.cpp)
  target_include_directories(${V4L2_CAMREA_SHM_EXE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${V4L2_CAMREA_SHM_EXE} v4l2_camera)
  ament_target_dependencies(${V4L2_CAMREA_SHM_EXE} "rclcpp" "shm_msgs")
  target_compile_options(${V4L2_CAMREA_SHM_EXE} PRIVATE -Werror)
  install(TARGETS ${V4L2_CAMREA_SHM_EXE} DESTINATION lib/${PROJECT_NAME})
endforeach()
