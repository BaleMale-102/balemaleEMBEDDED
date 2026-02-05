#!/bin/bash
# Perception 시스템 테스트 스크립트
#
# Usage:
#   ./test_perception.sh status   # 서버/노드 상태 확인
#   ./test_perception.sh topics   # ROS2 토픽 목록
#   ./test_perception.sh echo     # 토픽 데이터 출력
#   ./test_perception.sh all      # 전체 테스트

EMBEDDED_PATH="/home/a102/balemaleEMBEDDED"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source $EMBEDDED_PATH/install/setup.bash 2>/dev/null

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_port() {
    local port=$1
    local name=$2
    if nc -z localhost $port 2>/dev/null; then
        echo -e "  ${GREEN}[OK]${NC} $name (port $port)"
        return 0
    else
        echo -e "  ${RED}[FAIL]${NC} $name (port $port) - Not running"
        return 1
    fi
}

check_topic() {
    local topic=$1
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
        local hz=$(timeout 2 ros2 topic hz $topic 2>/dev/null | head -1 | grep -oP '\d+\.\d+' || echo "0")
        if [ "$hz" != "0" ]; then
            echo -e "  ${GREEN}[OK]${NC} $topic (${hz} Hz)"
        else
            echo -e "  ${YELLOW}[WARN]${NC} $topic (no data)"
        fi
        return 0
    else
        echo -e "  ${RED}[FAIL]${NC} $topic - Not found"
        return 1
    fi
}

test_status() {
    echo "=========================================="
    echo " Perception System Status"
    echo "=========================================="
    echo ""
    echo "[AI Servers]"
    check_port 9001 "Anomaly Server"
    check_port 9002 "OCR Server"
    echo ""
    echo "[ROS2 Nodes]"
    if ros2 node list 2>/dev/null | grep -q "anomaly_detector"; then
        echo -e "  ${GREEN}[OK]${NC} anomaly_detector"
    else
        echo -e "  ${RED}[FAIL]${NC} anomaly_detector - Not running"
    fi
    if ros2 node list 2>/dev/null | grep -q "ocr_detector"; then
        echo -e "  ${GREEN}[OK]${NC} ocr_detector"
    else
        echo -e "  ${RED}[FAIL]${NC} ocr_detector - Not running"
    fi
    echo ""
}

test_topics() {
    echo "=========================================="
    echo " ROS2 Topics (Perception)"
    echo "=========================================="
    echo ""
    ros2 topic list 2>/dev/null | grep -E "perception|cam_front|cam_side" || echo "No perception topics found"
    echo ""
}

test_echo() {
    echo "=========================================="
    echo " Topic Data (5 seconds each)"
    echo "=========================================="
    echo ""

    echo "[Anomaly Detections]"
    timeout 5 ros2 topic echo /perception/anomaly/detections --once 2>/dev/null || echo "  No data"
    echo ""

    echo "[OCR Plate]"
    timeout 5 ros2 topic echo /perception/ocr/plate --once 2>/dev/null || echo "  No data"
    echo ""

    echo "[OCR Sticker]"
    timeout 5 ros2 topic echo /perception/ocr/sticker --once 2>/dev/null || echo "  No data"
    echo ""
}

test_all() {
    test_status
    test_topics
}

case "${1:-status}" in
    status)
        test_status
        ;;
    topics)
        test_topics
        ;;
    echo)
        test_echo
        ;;
    all)
        test_all
        ;;
    *)
        echo "Usage: $0 [status|topics|echo|all]"
        echo ""
        echo "  status - Check server/node status (default)"
        echo "  topics - List perception ROS2 topics"
        echo "  echo   - Echo topic data"
        echo "  all    - Run all tests"
        exit 1
        ;;
esac
