#! /bin/bash

sudo modprobe can

sudo modprobe can_raw

sudo modprobe mttcan

sudo ip link set down can0

sudo ip link set can0 type can bitrate 500000 berr-reporting on fd off

sudo ip link set up can0

sudo ip -details -statistics link show can0
