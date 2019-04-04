cmake_minimum_required(VERSION 2.8.3)
project(object_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cds_msgs
  roscpp
  std_msgs
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES object_detector
 CATKIN_DEPENDS cds_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/object_detector.cpp
)

add_executable(${PROJECT_NAME}_node src/object_detector_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
