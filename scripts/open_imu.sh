#!/bin/bash

set -e

gnome-terminal --tab --title="imu-publish" -- bash -c \
"
roslaunch razor_imu_9dof razor-pub.launch
"

sleep 5

gnome-terminal --tab --title="imu-visual" -- bash -c \
"
cd /home/nvidia/workspace/robotcar_sensor/src/visual_Razor9DOF
python3 display_3D_visualization.py
"


exit 0
