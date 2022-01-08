cmake_minimum_required(VERSION 2.8.3)
project(robotcar_bringup)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE Debug)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  robotcar_general
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime

  INCLUDE_DIRS
  include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  bringup
  src/main.cpp
  src/bringup.cpp
)

add_dependencies(
  bringup
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  bringup
  general_lib
  ${catkin_LIBRARIES}
)

add_library(
  bringup_lib
  src/bringup.cpp
)

add_dependencies(
  bringup_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  bringup_lib
  general_lib
  ${catkin_LIBRARIES}
)
