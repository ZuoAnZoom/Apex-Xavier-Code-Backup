#!/bin/bash

cd ~/Desktop/scripts
./open_camera.sh

rosrun camera_calibration cameracalibrator.py  --size 10x7 --approximate 0.01 --square 0.082 left:=/miivii_gmsl_ros_A/camera1 right:=/miivii_gmsl_ros_A/camera2 left_camera:=/miivii_gmsl_ros_A right_camera:=/miivii_gmsl_ros_A

#size--棋盘格内点size
#square--棋盘格大小（m）
#left/right---输入图片话题
#left/right_camera---相机节点名
