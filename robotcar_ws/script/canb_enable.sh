#! /bin/bash

sudo modprobe can

sudo modprobe can_raw

sudo modprobe mttcan

sudo ip link set down can1

sudo ip link set can1 type can bitrate 500000 berr-reporting on fd off

sudo ip link set up can1

sudo ip -details -statistics link show can1
