#!/bin/bash
cd ~/balemaleEMBEDDED
source install/setup.bash

echo "=== Starting front cam (C920 /dev/video2) ==="
ros2 run camera_driver camera_node --ros-args -p device:=/dev/video2 -p width:=640 -p height:=480 -p camera_name:=cam_front &
PID1=$!
sleep 2

echo "=== Starting side cam (Brio /dev/video0) ==="
ros2 run camera_driver camera_node --ros-args -p device:=/dev/video0 -p width:=640 -p height:=480 -p camera_name:=cam_side &
PID2=$!
sleep 2

echo ""
echo "=== Camera Topics ==="
ros2 topic list | grep cam

echo ""
echo "=== Front cam rate ==="
timeout 3 ros2 topic hz /cam_front/image_raw 2>&1 | tail -2

echo ""
echo "=== Side cam rate ==="
timeout 3 ros2 topic hz /cam_side/image_raw 2>&1 | tail -2

echo ""
echo "=== Cleanup ==="
kill $PID1 $PID2 2>/dev/null
echo "Done"
