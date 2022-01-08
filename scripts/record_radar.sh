#!/bin/bash

set -e
set -x

fps_A=$(cat ~/workspace/robotcar_ws/src/miivii_gmsl_camera/miivii_gmsl_ros/launch/1_node_with_4_cameras.launch | grep fps)

echo -e "[INFO] Group_A cameras info are below:"
echo -e "[INFO]   Camera 1: front Left"
echo -e "[INFO]   Camera 2: front Right"
echo -e "[INFO]$fps_A"

cd ~/workspace/DATA_RECORD/radar_data
echo -e "[INFO] cameras image and front_radar data will saved at $PWD"

echo -e "[INFO] Will record Group A. Camera 1-2. Front_Radar"
rosbag record -o Front_camera /miivii_gmsl_ros_A/camera1/compressed  /miivii_gmsl_ros_A/camera2/compressed & rosbag record -o Radar /ars_40X/clusters /ars_40X/objects /radar_status

exit 0
