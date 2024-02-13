cmake_minimum_required(VERSION 2.8.3)
project(action_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  actionlib
  actionlib_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
#add_message_files(
#  DIRECTORY msg
#  FILES start_msg.msg stop_msg.msg
#)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES countdown.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/action_server.cpp	
# )

## Declare a C++ executable
#add_executable(${PROJECT_NAME}_node action_server_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

