add_library(odom_laser_utils SHARED
  canvas.cpp canvas.h
  geometry_utils_fd.h)

target_link_libraries(odom_laser_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})
