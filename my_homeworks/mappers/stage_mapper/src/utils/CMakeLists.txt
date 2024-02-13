add_library(mapper_utils SHARED
  canvas.cpp canvas.h
  geometry_utils_fd.h)

target_link_libraries(mapper_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})
