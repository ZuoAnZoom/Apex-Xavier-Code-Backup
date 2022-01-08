#!/bin/bash

set -e

pid=$(pidof bringup)
kill -9 $pid
if [[ "$?"=="0" ]]; then
	echo -e "[INFO] Kill car chassis successfully!"
else
	echo -e "[EROR] Kill car chassis FAILED! Please kill it from htop!"
fi

exit 0
