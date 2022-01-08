#!/bin/bash

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can1 type can bitrate 500000 berr-reporting on loopback off
sudo ip link set up can1
sudo ip -details -statistics link show can1

# Show can1 data
#sudo candump can1

# Shut down can1 --> can B
#sudo ifconfig can1 down

source /home/nvidia/workspace/radar_ws/devel/setup.bash

echo -e "[INFO] Do you want to show obstacle_array in Object mode ? [y/n]"
read ans

if [[ "$ans"=="y" ]]; then
	echo -e "[INFO] roslaunch ars_40X ars_40X.launch obstacle_array:=true visualize:=true"
	roslaunch ars_40X ars_40X.launch obstacle_array:=true visualize:=true
elif [[ "$ans"=="n" ]]; then
	echo -e "[INFO] roslaunch ars_40X ars_40X.launch obstacle_array:=false visualize:=true"
	roslaunch ars_40X ars_40X.launch obstacle_array:=false visualize:=true
fi


# modify the radar output_type
# 0 none
# 1 object
# 2 cluster

#rosservice call /set_ouput_type 2



exit 0

