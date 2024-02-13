cmake_minimum_required(VERSION 2.8.3)
project(video_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport sensor_msgs)

# Find Opencv
find_package(OpenCV REQUIRED)
include_directories(INCLUDE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
 add_executable(${PROJECT_NAME}_node src/video_publisher_node.cpp)
add_dependencies(video_publisher_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(video_publisher_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(video_reader_node src/video_reader_node.cpp)
add_dependencies(video_reader_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(video_reader_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )



# Mark executables and/or libraries for installation
install(TARGETS video_publisher_node video_reader_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

