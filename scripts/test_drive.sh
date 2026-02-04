#!/bin/bash
# test_drive.sh - 주행 테스트 명령
# Usage: ./test_drive.sh [waypoints]
# Example: ./test_drive.sh 0,1,2
#          ./test_drive.sh 0,1,5,6,7

source ~/balemaleEMBEDDED/install/setup.bash

if [ -z "$1" ]; then
    echo "=== 주행 테스트 ==="
    echo "Usage: $0 <waypoints>"
    echo ""
    echo "Examples:"
    echo "  $0 0,1,2        # 마커 0→1→2"
    echo "  $0 0,1,5,6      # 마커 0→1→5→6"
    echo "  $0 1,5,17       # 마커 1→5→17"
    echo ""
    echo "=== 맵 레이아웃 ==="
    echo "      0  ← 시작"
    echo "      |"
    echo "      1  ← 첫 갈림길"
    echo "     /|\\"
    echo "    2 | 3"
    echo "    |   |"
    echo "    4 5-8 9  ← 중간"
    echo "    |     |"
    echo "   10 11-14 15  ← 하단"
    echo ""
    echo "주차: 16-19(A), 20-23(B), 24-27(C)"
    exit 1
fi

echo "Starting drive: $1"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START $1'"
