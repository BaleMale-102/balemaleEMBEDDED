#!/bin/bash
# test_full.sh - 풀 미션 테스트 (주행 + 주차/출차)
# Usage: ./test_full.sh <waypoints> <slot_id> [exit]
# Example: ./test_full.sh 1,3,9 17
# Example: ./test_full.sh 1,5 17 exit

source ~/balemaleEMBEDDED/install/setup.bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "=== 풀 미션 테스트 (주행 + 주차/출차) ==="
    echo "Usage: $0 <waypoints> <slot_id> [exit]"
    echo ""
    echo "입고 (기본):"
    echo "  $0 1,3,9 17       # 1→3→9 주행 후 슬롯17 주차"
    echo "  $0 1,5,6,7 18     # 1→5→6→7 주행 후 슬롯18 주차"
    echo "  $0 0,1,5 20       # 0→1→5 주행 후 슬롯20 주차"
    echo ""
    echo "출차 (exit 옵션):"
    echo "  $0 1,5 17 exit    # 1→5→슬롯17 출차 (차량 회수)"
    echo "  $0 1,5,6,7 20 exit  # 슬롯20 출차"
    echo ""
    echo "Note: waypoints는 경유 마커, slot은 최종 주차 위치"
    echo ""
    echo "=== 주차 슬롯 ==="
    echo "  A: 16, 17, 18, 19"
    echo "  B: 20, 21, 22, 23"
    echo "  C: 24, 25, 26, 27"
    exit 1
fi

WAYPOINTS=$1
SLOT=$2
MODE=${3:-"park"}

if [ "$MODE" = "exit" ]; then
    echo "=== Exit Mission Test (출차) ==="
    echo "Waypoints: $WAYPOINTS"
    echo "Target Slot: $SLOT"
    echo "Flow: DRIVE→PARK→LOAD→RETURN_HOME→UNLOAD"
    echo ""
    ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'EXIT $WAYPOINTS $SLOT'"
else
    echo "=== Full Mission Test (입고) ==="
    echo "Waypoints: $WAYPOINTS"
    echo "Target Slot: $SLOT"
    echo ""
    ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START_FULL $WAYPOINTS $SLOT'"
fi
