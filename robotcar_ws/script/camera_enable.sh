#! /bin/bash

cd /opt/miivii/features/gmsl_camera

# 注意/dev和尺寸的修改，n*1280×720，n为摄像头数量
./bin/4cameras_egl_demo -d /dev/video0 -s 1280x720
