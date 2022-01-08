cmake_minimum_required(VERSION 2.8.3)
project(robotcar_motion_planner)

add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE Debug)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  map_msgs
  nav_msgs
  geometry_msgs
  robotcar_general
  robotcar_map
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  tf
  std_msgs
  map_msgs
  nav_msgs
  geometry_msgs
  message_runtime

  # DEPENDS
  # EIGEN3
  # Boost

  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_executable(
  state_lattice_motion_planner
  src/state_lattice/test_state_lattice_planner.cpp
  src/state_lattice/state_lattice_motion_planner.cpp
  src/state_lattice/bvp_solver.cpp
  src/state_lattice/frame_transformer.cpp
)

add_dependencies(
  state_lattice_motion_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  state_lattice_motion_planner 
  general_lib
  map_lib
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(
  motion_planner_lib
  src/state_lattice/test_state_lattice_planner.cpp
  src/state_lattice/state_lattice_motion_planner.cpp
  src/state_lattice/bvp_solver.cpp
  src/state_lattice/frame_transformer.cpp
)

add_dependencies(
  motion_planner_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  motion_planner_lib
  general_lib
  map_lib
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)
