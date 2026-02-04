#!/bin/bash
cd ~/balemaleEMBEDDED
source install/setup.bash
ros2 run camera_driver camera_node --ros-args -p device:=/dev/video0 -p width:=640 -p height:=480 -p fps:=30.0 -r __node:=cam_calib -r image_raw:=/camera/image_raw -r camera_info:=/camera/camera_info
