#!/bin/bash
# monitor.sh - 상태 모니터링
# Usage: ./monitor.sh [topic]

source ~/balemaleEMBEDDED/install/setup.bash

show_menu() {
    echo "=== 모니터링 ==="
    echo "1) mission   - 미션 상태"
    echo "2) driving   - 주행 상태"
    echo "3) marker    - 마커 추적"
    echo "4) side      - 측면 마커 (주차용)"
    echo "5) parking   - 주차 상태"
    echo "6) loader    - 로더 상태"
    echo "7) all       - 토픽 목록"
    echo ""
    echo "Usage: $0 [1-7 or topic_name]"
}

case "$1" in
    1|mission)
        ros2 topic echo /mission/state
        ;;
    2|driving)
        ros2 topic echo /driving/state
        ;;
    3|marker)
        ros2 topic echo /perception/tracked_marker
        ;;
    4|side)
        ros2 topic echo /perception/side_markers
        ;;
    5|parking)
        ros2 topic echo /parking/status
        ;;
    6|loader)
        ros2 topic echo /loader/status
        ;;
    7|all)
        ros2 topic list
        ;;
    "")
        show_menu
        ;;
    *)
        # Custom topic
        ros2 topic echo "$1"
        ;;
esac
