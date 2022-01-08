#!/bin/bash

set -e

# 检查 roscore 是否运行，没有运行则运行 roscore
COUNT=$(ps -elf | grep roscore |  grep -v "grep" | wc -l)
if [[ $COUNT -eq 0 ]]; then
	echo -e '[INFO] Opening roscore...'
	gnome-terminal --tab --title="roscore" -- bash -c "roscore;"
	sleep 4
fi

# 播发 GPS 数据
gnome-terminal --tab --title="gps-publish" -- bash -c \
"
source ~/workspace/robotcar_sensor/devel/setup.bash
echo <YOUR-PASSWORD> | sudo -S chmod a+rw /dev/ttyACM0
rosrun ublox_serial ublox_gps_pub
#roslaunch ublox_serial ublox_serial_launch.launch
"

exit 0
