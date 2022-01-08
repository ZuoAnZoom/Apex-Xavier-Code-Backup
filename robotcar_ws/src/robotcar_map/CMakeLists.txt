cmake_minimum_required(VERSION 2.8.3)
# 程序包名
project(robotcar_map)

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
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  message_generation
  robotcar_general
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
# find_library(PQXX_LIB pqxx REQUIRED)
# find_library(PQ_LIB pq REQUIRED)

# 添加自定义消息
add_message_files(
   FILES
   lanes.msg
   obstacles.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  tf
  std_msgs
  map_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  laser_geometry
  message_runtime
  pcl_ros

  DEPENDS
  PCL
  EIGEN3
  Boost

  # 用于提供头文件给其他包引用
  INCLUDE_DIRS
  include
)

# 指定头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(
  ${EIGEN3_DEFINITIONS}
)

# 生成库
add_library(
  map_lib
#  src/global_map.cpp
  src/map.cpp
  src/static_map.cpp
  src/semantic_map.cpp
  src/inflation_map.cpp
  src/map_manager.cpp
)

# 为生成的库添加依赖项
add_dependencies(
  map_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 生成可执行程序
add_executable(
  map
#  src/global_map.cpp
  src/map.cpp
  src/static_map.cpp
  src/semantic_map.cpp
  src/inflation_map.cpp
  src/map_manager.cpp
  src/test_map.cpp
)

# 为可执行程序添加依赖项
add_dependencies(
  map
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 为项目链接动态库
target_link_libraries(
  map
  general_lib
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
#  ${PQXX_LIB}
#  ${PQ_LIB}
#  ${PROJ_LIB}
)

# 为项目生成的库链接动态库
target_link_libraries(
  map_lib
  general_lib
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
#  ${PQXX_LIB}
#  ${PQ_LIB}
#  ${PROJ_LIB}
)
