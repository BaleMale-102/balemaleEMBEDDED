#!/bin/bash
# stop.sh - 긴급 정지

source ~/balemaleEMBEDDED/install/setup.bash

echo "=== EMERGENCY STOP ==="
ros2 topic pub --once /control/cmd_vel geometry_msgs/Twist "{}"
echo "Stopped."
