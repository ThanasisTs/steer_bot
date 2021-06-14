cmake_minimum_required(VERSION 3.0.2)
project(steer_bot_motion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES steer_bot_motion
#  CATKIN_DEPENDS geometry_msgs roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

