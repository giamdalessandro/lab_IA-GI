add_library(mapOdom_library SHARED
  mapOdom.cpp mapOdom.h)

target_link_libraries(mapOdom_library ${catkin_LIBRARIES})
