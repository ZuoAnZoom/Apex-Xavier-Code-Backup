#!/bin/bash

set -e

# 修改串口权限
PASSWORD=<YOUR-PASSWORD>
echo $PASSWORD | sudo -S chmod a+rw /dev/ttyACM0

# 检查 roscore 是否运行，没有运行则运行 roscore
COUNT=$(ps -elf | grep roscore |  grep -v "grep" | wc -l)
if [[ $COUNT -eq 0 ]]; then
	echo -e '[INFO] Opening roscore...'
	gnome-terminal --tab --title="roscore" -- bash -c "roscore;"
	sleep 4
fi

# 使用 nmea_navsat_driver 包来播发 GPS NMEA 数据
# 该包安装方式如下：sudo apt-get install ros-<distro>-nmea-navsat-driver libgps-dev
# https://www.jianshu.com/p/6a86fc69d412

gnome-terminal --tab --title="gps-publish-nmea" -- bash -c \
"
source ~/workspace/robotcar_sensor/devel/setup.bash
roslaunch nmea_navsat_driver nmea_serial_driver.launch port:=/dev/ttyACM0 baud:=115200
"

exit 0
