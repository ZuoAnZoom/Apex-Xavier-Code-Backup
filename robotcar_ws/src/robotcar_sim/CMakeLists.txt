cmake_minimum_required(VERSION 2.8.3)
project(robotcar_sim)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE Debug)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
