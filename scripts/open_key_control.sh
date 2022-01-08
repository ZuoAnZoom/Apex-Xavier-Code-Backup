#!/bin/bash

#set -ex

function start_key_control() {
	source ~/workspace/robotcar_ws/devel/setup.bash
	echo -e "[INFO] Using Key to Control Car!"
	rosrun robotcar_teleop teleop_real.py


	chassis_pid=$(pidof bringup)
	echo -e "[INFO] PID of car chassis is $chassis_pid."
	echo -e "[INFO] Do you want to kill car chassis? [y/n]"
	read ans
	if [[ "$ans" == "y" ]]; then
		kill -9 $chassis_pid
		if [[ "$?"=="0" ]]; then
			echo -e "[INFO] car chassis successfully killed."
		else
			echo -e "[EROR] car chassis kill FAILED. Please kill it manuallt."
		fi
	elif [[ "$ans"=="n" ]]; then
		echo -e "[INFO] car chassis stay alive. Please kill manually."
	fi
}

gnome-terminal --tab --title="key-control" -- bash -c \
"
source ~/workspace/robotcar_ws/devel/setup.bash; 
echo -e '[INFO] Using Key to Control Car!';	
rosrun robotcar_teleop teleop_real.py;
"

exit 0

