#!/bin/bash
# launch.sh - 시스템 실행

cd ~/balemaleEMBEDDED
source install/setup.bash

echo "=== Robot System Launch ==="
echo "Options:"
echo "  loader_simulate:=true (default)"
echo "  simulation:=true (for no hardware)"
echo ""

ros2 launch robot_bringup system.launch.py "$@"
