cmake_minimum_required(VERSION 2.8.3)
project(robotcar_collect_data)
set( CMAKE_BUILD_TYPE "Debug" )

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  #OpenCV
  cv_bridge
  robotcar_general
  robotcar_map
)

find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(
  extract_compressed_image
  src/extract_compressed_image.cpp
)

add_executable(
  get_vo_data
  src/get_vo_data.cpp
)

add_executable(
  get_lane_data
  src/get_lane_data.cpp
)

add_dependencies(
  get_vo_data
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  get_lane_data
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  extract_compressed_image
  general_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  get_vo_data
  general_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  get_lane_data
  general_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
