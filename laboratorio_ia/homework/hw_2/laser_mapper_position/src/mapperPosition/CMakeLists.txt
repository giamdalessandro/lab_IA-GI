add_library(mapper_position_library SHARED
  mapper.cpp mapper.h)

target_link_libraries(mapper_position_library ${catkin_LIBRARIES})
