add_library(odom_laser_library SHARED
  mapper.cpp mapper.h)

target_link_libraries(odom_laser_library ${catkin_LIBRARIES})
