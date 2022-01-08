cmake_minimum_required(VERSION 2.8.3)
project(robotcar_general)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)

catkin_package(
  DEPENDS EIGEN3
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(
  general_lib
  src/curve_fitter.cpp
  src/print_helper.cpp
)

add_dependencies(
  general_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  general_lib
  ${catkin_LIBRARIES} 
)
