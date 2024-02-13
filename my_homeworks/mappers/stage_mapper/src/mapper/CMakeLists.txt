add_library(mapper_library SHARED
  mapper.cpp mapper.h)

target_link_libraries(mapper_library ${catkin_LIBRARIES})
