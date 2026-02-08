#!/bin/bash
# test_full.sh - 풀 미션 테스트 (주행 + 주차/출차)
# Usage: ./test_full.sh <full_path> <slot_id> [exit]
#   full_path: 서버 전체 왕복 경로 (origin → parking area → home)
# Example: ./test_full.sh 0,1,5,1,0 17
# Example: ./test_full.sh 0,1,5,1,0 17 exit

source ~/balemaleEMBEDDED/install/setup.bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "=== 풀 미션 테스트 (주행 + 주차/출차) ==="
    echo "Usage: $0 <full_path> <slot_id> [exit]"
    echo ""
    echo "  full_path: 전체 왕복 경로 (origin → parking area → home)"
    echo "  slot_id:   목표 주차 슬롯 ID"
    echo ""
    echo "입고 (기본):"
    echo "  $0 0,1,5,1,0 17       # 0→1→5 주행 → 슬롯17 주차 → 5→1→0 복귀"
    echo "  $0 0,1,5,1,0 18       # 슬롯18 주차"
    echo "  $0 0,1,2,4,5,1,0 20   # 왼쪽 경로로 B1 주차"
    echo ""
    echo "출차 (exit 옵션):"
    echo "  $0 0,1,5,1,0 17 exit    # 슬롯17 출차 (차량 회수)"
    echo "  $0 0,1,5,1,0 20 exit    # 슬롯20 출차"
    echo ""
    echo "Note: full_path는 전체 왕복 경로, parking_road_marker 자동 계산"
    echo ""
    echo "=== 주차 슬롯 (도로마커 매핑) ==="
    echo "  A: 16(5,11), 17(6,12), 18(7,13), 19(8,14)"
    echo "  B: 20(5,11), 21(6,12), 22(7,13), 23(8,14)"
    echo "  C: 24(5,11), 25(6,12), 26(7,13), 27(8,14)"
    exit 1
fi

FULL_PATH=$1
SLOT=$2
MODE=${3:-"park"}

if [ "$MODE" = "exit" ]; then
    echo "=== Exit Mission Test (출차) ==="
    echo "Full Path: $FULL_PATH"
    echo "Target Slot: $SLOT"
    echo "Flow: DRIVE(pre)→PARK→LOAD→RETREAT→DRIVE(post)→UNLOAD"
    echo ""
    ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'EXIT $FULL_PATH $SLOT'"
else
    echo "=== Full Mission Test (입고) ==="
    echo "Full Path: $FULL_PATH"
    echo "Target Slot: $SLOT"
    echo "Flow: DRIVE(pre)→PARK→UNLOAD→RETREAT→DRIVE(post)→WAIT"
    echo ""
    ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START_FULL $FULL_PATH $SLOT'"
fi
