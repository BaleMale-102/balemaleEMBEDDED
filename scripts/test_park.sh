#!/bin/bash
# test_park.sh - 주차 테스트 명령
# Usage: ./test_park.sh [slot_id]
# Example: ./test_park.sh 17

source ~/balemaleEMBEDDED/install/setup.bash

if [ -z "$1" ]; then
    echo "=== 주차 테스트 ==="
    echo "Usage: $0 <slot_id>"
    echo ""
    echo "Examples:"
    echo "  $0 16    # A1 슬롯"
    echo "  $0 17    # A2 슬롯"
    echo "  $0 20    # B1 슬롯"
    echo "  $0 24    # C1 슬롯"
    echo ""
    echo "=== 주차 슬롯 ==="
    echo "  A: 16, 17, 18, 19"
    echo "  B: 20, 21, 22, 23"
    echo "  C: 24, 25, 26, 27"
    exit 1
fi

echo "Starting park to slot: $1"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START_PARK $1'"
