add_executable(slam slam.cpp
  intrinsic_matrix.cpp
  rgbd_image.cpp
  dense_tracker.cpp
  param_reader.cpp
  )

# OpenCV library
find_package(OpenCV REQUIRED)
target_link_libraries(slam ${OpenCV_LIBS})
target_link_libraries(slam yaml-cpp)
