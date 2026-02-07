#!/bin/bash
# full.sh - 입고 풀 미션 (주차 + 하역 + 홈 복귀)
# Flow: WAIT → RECOGNIZE → LOAD → DRIVE → PARK → UNLOAD → RETURN_HOME → DRIVE(역순) → WAIT
#
# Usage: ./full.sh <waypoints> <slot_id> [plate_number]
# Example: ./full.sh 0,1,5 17
#          ./full.sh 0,1,5 17 12가3456

source ~/balemaleEMBEDDED/install/setup.bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "=== 풀 미션 (주차 + 홈 복귀) ==="
    echo "Usage: $0 <waypoints> <slot_id> [plate_number]"
    echo ""
    echo "Examples:"
    echo "  $0 0,1,5 17            # 0→1→5 주행 → 슬롯17 주차 → 하역 → 복귀"
    echo "  $0 0,1,5 17 12가3456    # 번호판 지정"
    echo ""
    echo "Flow: WAIT → RECOGNIZE → LOAD → DRIVE → PARK → UNLOAD → RETURN_HOME → WAIT"
    echo ""
    echo "=== 주차 슬롯 ==="
    echo "  A: 16, 17, 18, 19"
    echo "  B: 20, 21, 22, 23"
    echo "  C: 24, 25, 26, 27"
    exit 1
fi

WAYPOINTS=$1
SLOT=$2
PLATE=${3:-"12가3456"}

echo "=== Full Mission (입고 + 복귀) ==="
echo "Plate:     $PLATE"
echo "Waypoints: $WAYPOINTS"
echo "Slot:      $SLOT"
echo "Flow: WAIT → RECOGNIZE → LOAD → DRIVE → PARK → UNLOAD → RETURN → WAIT"
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
echo "[3/3] 서버 검증: slot=$SLOT, waypoints=$WAYPOINTS → LOAD..."
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY $SLOT $WAYPOINTS'"

echo ""
echo "미션 시작 완료! 이후 자동 진행:"
echo "  LOAD → DRIVE → PARK_* → UNLOAD → RETURN_HOME → DRIVE(역순) → WAIT_VEHICLE"
echo ""
echo "모니터링: ros2 topic echo /mission/state"
