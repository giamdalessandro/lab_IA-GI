add_executable(laser_mapper_node laser_mapper_node.cpp)
target_link_libraries(laser_mapper_node mapper_library mapper_utils ${catkin_LIBRARIES})

