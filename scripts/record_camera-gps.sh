#!/bin/bash

set -e

fps_A=$(cat ~/workspace/robotcar_ws/src/gmsl_camera/launch/1_node_with_4_cameras.launch | grep fps)

echo -e "[INFO] Group_A cameras info are below:"
echo -e "[INFO]   Camera 1: front Left"
echo -e "[INFO]   Camera 2: front Right"
echo -e "[INFO]   Camera 3: front Fish eye"
echo -e "[INFO]   Camera 4: back  Fish eye"
echo -e "[INFO]$fps_A"
echo -e "[INFO] Please MAKE SURE you don't need to change the camera FPS !"


cd ~/workspace/DATA_RECORD/
echo -e "[INFO] cameras image will saved at $PWD"


echo -e "[INFO] Will record Group A. Camera 1-4."
rosbag record /miivii_gmsl_ros_A/camera1 /miivii_gmsl_ros_A/camera2 /miivii_gmsl_ros_A/camera3 /miivii_gmsl_ros_A/camera4 /gps_info


exit 0
