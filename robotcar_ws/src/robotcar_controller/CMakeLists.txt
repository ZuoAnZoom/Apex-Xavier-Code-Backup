cmake_minimum_required(VERSION 2.8.3)
project(robotcar_controller)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  robotcar_map
  robotcar_general
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(
  controller
  src/controller.cpp
)

add_dependencies(
  controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  controller
  general_lib
  ${catkin_LIBRARIES}
)

add_library(
  controller_lib
  src/controller.cpp
)

add_dependencies(
  controller_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  controller_lib
  general_lib
  ${catkin_LIBRARIES}
)
