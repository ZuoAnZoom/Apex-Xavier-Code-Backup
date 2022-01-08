#!/bin/bash


set -e

gnome-terminal --tab --title="open-camera" -- bash -c \
"
source ~/workspace/robotcar_ws/devel/setup.bash;
echo -e '[INFO] GMSL Cameras opening...';
echo -e '[INFO] Please RUN rqt OR rqt_image_view to see the camera images.';
#roslaunch miivii_gmsl_ros 2_nodes_with_8_cameras.launch fps_AB:=25;
roslaunch miivii_gmsl_ros 1_node_with_4_cameras.launch fps_A:=25;
"

sleep 4

gnome-terminal --tab --title="rqt_image_view" -- bash -c "rqt_image_view"

exit 0
