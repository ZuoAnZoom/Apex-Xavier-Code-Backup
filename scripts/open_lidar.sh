#!/bin/bash

set -e

# open c16
gnome-terminal --tab --title="lslidar-c16" -- bash -c \
"
source ~/workspace/lslidar_c16_ws/devel/setup.bash;
roslaunch lslidar_c16_decoder lslidar_c16.launch ls_distance_unit:=1.0 ls_use_gps_ts:=false ls_frame_id:=livox_frame;
"

sleep 10 # wait for roscore

# open ptp sync
gnome-terminal --tab --title="ptp-master-clock" -- bash -c "echo <YOUR-PASSWORD> | sudo -S ptpd -M -i eth0 -C"
# open mid-70
gnome-terminal --tab --title="livox-mid70" -- bash -c \
"
source ~/workspace/ws_livox/devel/setup.bash;
roslaunch livox_ros_driver livox_lidar.launch msg_frame_id:=livox_frame;
"

# open rviz
gnome-terminal --tab --title="rviz" -- bash -c \
"
source ~/workspace/ws_livox/devel/setup.bash;
source ~/workspace/lslidar_c16_ws/devel/setup.bash;
rviz;
"


exit 0
