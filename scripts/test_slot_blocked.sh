#!/bin/bash
# test_slot_blocked.sh - 주차 슬롯 장애물 감지 → 재할당 테스트
#
# Flow:
#   1. WAIT → PLATE → VERIFY 17 (waypoints 1,3,9,6) → LOAD → DRIVE → PARK_DETECT
#   2. PARK_DETECT 상태에서 side_anomaly 발행 → SLOT_BLOCKED (ESTOP)
#   3. 서버 재할당 시뮬: STOP → START_FULL 9,15,12 25 → DRIVE → PARK
#
# Usage: ./test_slot_blocked.sh

source ~/balemaleEMBEDDED/install/setup.bash

WAYPOINTS_1="1,3,9,6"
SLOT_1=17
WAYPOINTS_2="9,15,12"
SLOT_2=25
PLATE="12가3456"

echo "============================================"
echo " 슬롯 장애물 감지 → 재할당 테스트"
echo "============================================"
echo "Phase 1: $WAYPOINTS_1 → slot $SLOT_1 (입고)"
echo "Phase 2: 장애물 감지 → SLOT_BLOCKED"
echo "Phase 3: $WAYPOINTS_2 → slot $SLOT_2 (재할당)"
echo "============================================"
echo ""

# ==========================================
# Phase 1: 입고 미션 시작 (slot 17)
# ==========================================
echo "=== Phase 1: 입고 미션 시작 ==="

echo "[1/3] WAIT_VEHICLE 시작..."
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'WAIT'"
sleep 1

echo "[2/3] 번호판 인식: $PLATE"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'PLATE $PLATE'"
sleep 1

echo "[3/3] 서버 검증: slot=$SLOT_1, waypoints=$WAYPOINTS_1"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY $SLOT_1 $WAYPOINTS_1'"

echo ""
echo ">>> 미션 진행 중... PARK_DETECT 상태 대기"
echo ">>> 상태 모니터링: ros2 topic echo /mission/state"
echo ""

# ==========================================
# Phase 2: PARK_DETECT 대기 → 장애물 발행
# ==========================================
echo "=== Phase 2: PARK_DETECT 대기 중... ==="
echo "(PARK_DETECT 상태 진입 시 자동으로 장애물 발행)"
echo ""

# PARK_DETECT 상태가 될 때까지 대기
while true; do
    STATE=$(ros2 topic echo --once /mission/state std_msgs/String 2>/dev/null | grep "data:" | head -1)
    if echo "$STATE" | grep -q "PARK_DETECT"; then
        echo ">>> PARK_DETECT 상태 감지!"
        sleep 2  # side_cam settle 대기
        break
    fi
    # 미션 실패/종료 감지
    if echo "$STATE" | grep -q -E "ERROR|IDLE|WAIT_VEHICLE"; then
        echo ">>> 미션 종료/에러 감지: $STATE"
        echo ">>> 스크립트 중단"
        exit 1
    fi
    sleep 0.5
done

# side_anomaly 발행 (cone, class_id=4, 거리 20cm)
echo ">>> 장애물 발행: cone at 20cm (side camera)"
ros2 topic pub --once --qos-reliability best_effort --qos-durability volatile /perception/side_anomaly/detections robot_interfaces/DetectionArray "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera_side'},
  detections: [{
    class_id: 4,
    class_name: 'cone',
    confidence: 0.95,
    x1: 100, y1: 100, x2: 200, y2: 200,
    distance: 20.0,
    text: '',
    has_sticker: false
  }],
  num_plates: 0,
  num_obstacles: 1
}"

echo ">>> SLOT_BLOCKED 발생! (ESTOP)"
echo ""

# anomaly_report 확인
echo "=== 서버 anomaly_report 수신 대기 (3초) ==="
timeout 3 ros2 topic echo /server/anomaly_report std_msgs/String --once 2>/dev/null || echo "(timeout)"
echo ""

# ==========================================
# Phase 3: 서버 재할당 시뮬레이션 (slot 25)
# ==========================================
echo "=== Phase 3: 서버 재할당 시뮬레이션 ==="
echo ">>> 2초 후 새 미션 할당: $WAYPOINTS_2 → slot $SLOT_2"
sleep 2

# 현재 미션 중지
echo "[1/2] 현재 미션 중지 (STOP)..."
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'STOP'"
sleep 1

# 새 미션 시작 (slot 25로 재할당)
echo "[2/2] 새 미션 시작: waypoints=$WAYPOINTS_2, slot=$SLOT_2"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START_FULL $WAYPOINTS_2 $SLOT_2'"

echo ""
echo "============================================"
echo " 재할당 완료!"
echo " $WAYPOINTS_2 → slot $SLOT_2 주행 시작"
echo "============================================"
echo ""
echo "모니터링: ros2 topic echo /mission/state"
