cmake_minimum_required(VERSION 2.8.3)
project(inspection_stop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(inspection_stop_node src/inspection_stop_node.cpp src/guidance.cpp)
target_link_libraries(inspection_stop_node ${catkin_LIBRARIES})