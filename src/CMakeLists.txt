add_executable(slam slam.cpp
  intrinsic_matrix.cpp
  rgbd_image.cpp
  dense_tracker.cpp
  param_reader.cpp
  keyframe_graph.cpp
  keyframe_tracker.cpp
  local_tracker.cpp
  point_selection.cpp
  local_map.cpp
  least_squares.cpp
  math_sse.cpp
  )

# OpenCV library
find_package(OpenCV REQUIRED)
target_link_libraries(slam ${OpenCV_LIBS})
target_link_libraries(slam yaml-cpp
  g2o_core
  g2o_types_slam3d
  g2o_solver_csparse
  g2o_stuff
  g2o_csparse_extension
  ${CSPARSE_LIBRARY})
