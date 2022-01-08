#!/bin/bash

cd ~/Desktop/scripts
./open_camera.sh

# 大纸板作标定板
#rosrun camera_calibration cameracalibrator.py  --size 8x6 --square 0.071 image:=/miivii_gmsl_ros_A/camera3 camera:=/miivii_gmsl_ros_A

# 书本作标定板
#rosrun camera_calibration cameracalibrator.py  --size 8x6 --square 0.024 image:=/miivii_gmsl_ros_A/camera1 camera:=/miivii_gmsl_ros_A

# 亚克力标定板
rosrun camera_calibration cameracalibrator.py  --size 10x7 --square 0.082 image:=/miivii_gmsl_ros_A/camera2 camera:=/miivii_gmsl_ros_A
