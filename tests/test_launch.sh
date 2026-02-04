#!/bin/bash
cd ~/balemaleEMBEDDED
source install/setup.bash

echo "=== Launching system.launch.py ==="
ros2 launch robot_bringup system.launch.py &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

sleep 10

echo ""
echo "=== Camera Topics ==="
ros2 topic list 2>/dev/null | grep -E "cam|image"

echo ""
echo "=== Front cam hz ==="
timeout 3 ros2 topic hz /cam_front/image_raw 2>&1 | tail -2

echo ""
echo "=== Side cam hz ==="
timeout 3 ros2 topic hz /cam_side/image_raw 2>&1 | tail -2

echo ""
echo "=== Debug image topics ==="
ros2 topic list 2>/dev/null | grep debug

echo ""
echo "Press Ctrl+C to stop..."
wait $LAUNCH_PID
