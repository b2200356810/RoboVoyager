cmake_minimum_required(VERSION 3.0.2)
project(video_streaming)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  VideoStreamInfo.msg
)

add_service_files(
  FILES
  SetVideoStreamSettings.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
