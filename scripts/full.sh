#!/bin/bash
# full.sh - 입고 풀 미션 (주차 + 하역 + 홈 복귀)
# Flow: WAIT → RECOGNIZE → LOAD → DRIVE(pre) → PARK → UNLOAD → RETREAT → DRIVE(post) → WAIT
#
# Usage: ./full.sh <full_path> <slot_id> [plate_number]
#   full_path: 서버 전체 왕복 경로 (origin → parking area → home)
# Example: ./full.sh 0,1,5,1,0 17
#          ./full.sh 0,1,5,1,0 17 12가3456

source ~/balemaleEMBEDDED/install/setup.bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "=== 풀 미션 (주차 + 홈 복귀) ==="
    echo "Usage: $0 <full_path> <slot_id> [plate_number]"
    echo ""
    echo "  full_path: 전체 왕복 경로 (origin → parking area → home)"
    echo "  slot_id:   목표 주차 슬롯 ID"
    echo ""
    echo "Examples:"
    echo "  $0 0,1,5,1,0 17            # 0→1→5 주행 → 슬롯17 주차 → 5→1→0 복귀"
    echo "  $0 0,1,5,1,0 17 12가3456    # 번호판 지정"
    echo "  $0 0,1,2,4,5,1,0 20        # 왼쪽 경로로 B1 주차"
    echo "  $0 0,1,3,9,8,1,0 19        # 오른쪽 경로로 A4 주차"
    echo ""
    echo "Flow: WAIT → RECOGNIZE → LOAD → DRIVE(pre) → PARK → UNLOAD → RETREAT → DRIVE(post) → WAIT"
    echo ""
    echo "=== 주차 슬롯 (도로마커 매핑) ==="
    echo "  A: 16(5,11), 17(6,12), 18(7,13), 19(8,14)"
    echo "  B: 20(5,11), 21(6,12), 22(7,13), 23(8,14)"
    echo "  C: 24(5,11), 25(6,12), 26(7,13), 27(8,14)"
    exit 1
fi

FULL_PATH=$1
SLOT=$2
PLATE=${3:-"12가3456"}

echo "=== Full Mission (입고 + 복귀) ==="
echo "Plate:     $PLATE"
echo "Full Path: $FULL_PATH"
echo "Slot:      $SLOT"
echo "Flow: WAIT → RECOGNIZE → LOAD → DRIVE(pre) → PARK → UNLOAD → RETREAT → DRIVE(post) → WAIT"
echo ""

# Step 1: WAIT_VEHICLE 모드 진입 (is_full_mission=True 설정)
echo "[1/3] WAIT_VEHICLE 시작..."
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'WAIT'"
sleep 1

# Step 2: 번호판 인식 시뮬레이션 → RECOGNIZE 진입
echo "[2/3] 번호판 인식: $PLATE → RECOGNIZE..."
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'PLATE $PLATE'"
sleep 1

# Step 3: 서버 검증 시뮬레이션 → LOAD → 자동 진행
# VERIFY는 전체 왕복 경로를 받아 parking_road_marker를 자동 계산
echo "[3/3] 서버 검증: slot=$SLOT, full_path=$FULL_PATH → LOAD..."
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY $SLOT $FULL_PATH'"

echo ""
echo "미션 시작 완료! 이후 자동 진행:"
echo "  LOAD → DRIVE(pre) → PARK_* → UNLOAD → RETREAT → DRIVE(post) → WAIT_VEHICLE"
echo ""
echo "모니터링: ros2 topic echo /mission/state"
