#!/bin/bash
# test_server.sh - 서버 통신 테스트
# Usage: ./test_server.sh [command]

source ~/balemaleEMBEDDED/install/setup.bash

show_help() {
    echo "=== 서버 통신 테스트 ==="
    echo ""
    echo "Usage: $0 <command> [args]"
    echo ""
    echo "Commands:"
    echo "  wait                    WAIT_VEHICLE 상태로 진입 (번호판 대기)"
    echo "  plate <번호판>          번호판 인식 시뮬레이션"
    echo "  verify <슬롯> [경로]    서버 응답 시뮬레이션"
    echo "  full <번호판> <슬롯> [경로]  전체 플로우 시뮬레이션"
    echo "  status                  현재 상태 확인"
    echo ""
    echo "Examples:"
    echo "  $0 wait                 # 대기 상태 진입"
    echo "  $0 plate 12가3456       # 번호판 인식"
    echo "  $0 verify 17 0,1,5      # 서버 응답 (슬롯17, 경로 0→1→5)"
    echo "  $0 full 12가3456 17 0,1,5  # 전체 테스트"
    echo "  $0 status               # 상태 확인"
    echo ""
    echo "=== 실제 서버 연동 시 ==="
    echo "1. wait 로 대기 상태 진입"
    echo "2. ANPR이 번호판 인식하면 자동으로 서버에 요청"
    echo "3. 서버 응답 받으면 자동으로 주행 시작"
}

case "$1" in
    wait)
        echo "Entering WAIT_VEHICLE state..."
        ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'WAIT'"
        ;;
    plate)
        if [ -z "$2" ]; then
            echo "Usage: $0 plate <번호판>"
            echo "Example: $0 plate 12가3456"
            exit 1
        fi
        echo "Simulating plate detection: $2"
        ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'PLATE $2'"
        ;;
    verify)
        if [ -z "$2" ]; then
            echo "Usage: $0 verify <슬롯> [경로]"
            echo "Example: $0 verify 17 0,1,5"
            exit 1
        fi
        SLOT=$2
        PATH_STR=${3:-""}
        if [ -n "$PATH_STR" ]; then
            echo "Simulating server response: slot=$SLOT, path=$PATH_STR"
            ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY $SLOT $PATH_STR'"
        else
            echo "Simulating server response: slot=$SLOT"
            ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY $SLOT'"
        fi
        ;;
    full)
        if [ -z "$2" ] || [ -z "$3" ]; then
            echo "Usage: $0 full <번호판> <슬롯> [경로]"
            echo "Example: $0 full 12가3456 17 0,1,5"
            exit 1
        fi
        PLATE=$2
        SLOT=$3
        PATH_STR=${4:-""}

        echo "=== Full Server Flow Test ==="
        echo "Plate: $PLATE"
        echo "Slot: $SLOT"
        echo "Path: $PATH_STR"
        echo ""

        echo "[1/3] Entering WAIT_VEHICLE state..."
        ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'WAIT'"
        sleep 1

        echo "[2/3] Simulating plate detection..."
        ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'PLATE $PLATE'"
        sleep 1

        echo "[3/3] Simulating server response..."
        if [ -n "$PATH_STR" ]; then
            ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY $SLOT $PATH_STR'"
        else
            ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY $SLOT'"
        fi

        echo ""
        echo "Mission started! Monitor with: ./monitor.sh 1"
        ;;
    status)
        echo "=== Current State ==="
        timeout 2 ros2 topic echo /mission/state --once 2>/dev/null || echo "(no message)"
        ;;
    *)
        show_help
        ;;
esac
