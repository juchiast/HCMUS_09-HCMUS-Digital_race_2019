cmake_minimum_required(VERSION 2.8.3)
project(cds_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  LandMark.msg
  Lane.msg
  SignDetected.msg
  System.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cds_msgs
 CATKIN_DEPENDS std_msgs 
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)