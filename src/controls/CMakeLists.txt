cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

