#! /bin/bash

source ~/workspace/robotcar_ws/devel/setup.bash

# roslaunch miivii_gmsl_ros 1_node_with_1_camera.launch
roslaunch miivii_gmsl_ros 2_nodes_with_8_cameras.launch
# roslaunch miivii_gmsl_ros 1_node_with_4_cameras.launch
# 等等
#roslaunch miivii_gmsl_ros 1_node_with_1_camera.launch
