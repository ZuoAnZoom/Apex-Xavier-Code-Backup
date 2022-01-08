cmake_minimum_required(VERSION 2.8.3)
project(robotcar_teleop)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

