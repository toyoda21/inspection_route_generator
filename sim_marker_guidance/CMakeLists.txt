cmake_minimum_required(VERSION 2.8.3)
project(marker_guidance)

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

add_executable(marker_guidance_node src/marker_guidance_node.cpp src/guidance.cpp)
target_link_libraries(marker_guidance_node ${catkin_LIBRARIES})

