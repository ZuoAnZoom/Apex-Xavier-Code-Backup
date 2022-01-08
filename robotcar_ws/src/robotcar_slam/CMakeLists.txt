cmake_minimum_required( VERSION 2.8 )
project (robotcar_slam)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_C_COMPILER "gcc" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O0" )
set( CMAKE_C_FLAGS "-std=c11" )
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

option(USE_ROS "use ros" OFF)
IF(USE_ROS)
    message(STATUS "Build with ROS!")
    add_definitions(-D_ROS_)
ELSE()
    message(STATUS "Build without ROS!")
ENDIF(USE_ROS)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# Eigen
find_package(Eigen3 3 REQUIRED )

# OpenCV
find_package( OpenCV 4 REQUIRED )

# Sophus 
find_package( Sophus REQUIRED )

# G2O
find_package( G2O REQUIRED )

#Pangolin
find_package(Pangolin REQUIRED )

# ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  image_transport
)

# ROS
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

####################添加头文件
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

# ROS
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

####################添加库文件
# 生成内部库
add_library( slam_lib SHARED
    src/Camera.cpp
    src/Param.cpp
    src/Frame.cpp
    src/MapPoint.cpp
    src/Map.cpp
    
    src/ObjDetector_mod.cpp
    src/ObjSegment_mod.cpp
    src/Mapping_mod.cpp
    src/Odometry_mod.cpp
    src/Optimization_mod.cpp
    src/LoopClosing_mod.cpp
    src/ORBextractor_mod.cpp
    src/DirectTracker_mod.cpp
    src/MotionClassifier_mod.cpp

    src/System.cpp
    src/Transform.cpp
    src/Viewer.cpp
)

# 内部库链接上外部库
set( THIRD_PARTY_LIBS 
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)
target_link_libraries( slam_lib
    ${THIRD_PARTY_LIBS}
    ${catkin_LIBRARIES}
    fmt
)

#####################添加可执行程序
add_executable( slam src/test/run_vo.cpp )
#可执行程序链接上内部库(已包含外部库)
target_link_libraries(slam slam_lib)
target_link_libraries(slam ${catkin_LIBRARIES})

add_executable(rectify_test src/test/rectify_test.cpp)
target_link_libraries( rectify_test  slam_lib )

add_executable(stereo_calib src/test/stereo_calib.cpp)
target_link_libraries( stereo_calib  slam_lib )

IF(USE_ROS)
    add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(myslam ${catkin_LIBRARIES})
ENDIF(USE_ROS)
