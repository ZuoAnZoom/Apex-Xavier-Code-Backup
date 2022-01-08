#!/bin/bash

set -e

COUNT=$(ps -elf | grep roscore |  grep -v "grep" | wc -l)
if [[ $COUNT -eq 0 ]]; then
	echo -e '[INFO] Opening roscore...'
	gnome-terminal --tab --title="roscore" -- bash -c "roscore;"
	sleep 4
fi

echo -e "[INFO] Opening car chassi..."
PASSWORD=<YOUR-PASSWORD>
gnome-terminal --tab --title="bringup" -- bash -c "source ~/workspace/robotcar_ws/devel/setup.bash; rosrun robotcar_bringup bringup $PASSWORD;"

exit 0
