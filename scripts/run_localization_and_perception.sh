#!/bin/bash

set -e

PASSWORD=<YOUR-PASSWORD>
echo $PASSWORD | sudo -S jetson_clocks

gnome-terminal --tab --title="run_localization_and_perception" -- bash -c \
"
source ~/workspace/robotcar_ws/devel/setup.bash
source ~/catkin_ws2/devel/setup.bash --extend
roslaunch robotcar_general localization_and_perception.launch
"

sleep 4

gnome-terminal --tab --title="rviz" -- bash -c \
"
source ~/workspace/robotcar_ws/devel/setup.bash
source ~/catkin_ws2/devel/setup.bash --extend
rviz
"

exit 0
