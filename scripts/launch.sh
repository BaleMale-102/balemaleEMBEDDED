#!/bin/bash
# launch.sh - 전체 시스템 실행 (AI 서버 + ROS2)
#
# Usage:
#   ./launch.sh              # 전체 시스템 시작
#   ./launch.sh stop         # 전체 시스템 종료
#   ./launch.sh status       # 상태 확인
#   ./launch.sh ros-only     # ROS만 실행 (AI 서버 이미 실행 중일 때)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMBEDDED_PATH="/home/a102/balemaleEMBEDDED"
BALEMALE_AI_PATH="/home/a102/balemaleAI"
CONDA_ENV="anpr_310"
SESSION_NAME="balemale"

start_all() {
    echo "=========================================="
    echo " Balemale System Launch"
    echo "=========================================="
    echo ""

    # 기존 세션 종료
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    sleep 1

    # tmux 세션 생성
    echo "[1/3] Starting AI Anomaly Server..."
    tmux new-session -d -s $SESSION_NAME -n ai_anomaly
    tmux send-keys -t $SESSION_NAME:ai_anomaly "source /home/a102/miniforge3/etc/profile.d/conda.sh && conda activate $CONDA_ENV && cd $BALEMALE_AI_PATH && python servers/anomaly_server.py" C-m

    echo "[2/3] Starting AI OCR Server..."
    tmux new-window -t $SESSION_NAME -n ai_ocr
    tmux send-keys -t $SESSION_NAME:ai_ocr "source /home/a102/miniforge3/etc/profile.d/conda.sh && conda activate $CONDA_ENV && cd $BALEMALE_AI_PATH && python servers/ocr_server.py" C-m

    # AI 서버 초기화 대기
    echo "      Waiting for AI servers to initialize..."
    sleep 5

    echo "[3/3] Starting ROS2 System (includes perception nodes)..."
    tmux new-window -t $SESSION_NAME -n ros2
    tmux send-keys -t $SESSION_NAME:ros2 "cd $EMBEDDED_PATH && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_bringup system.launch.py $*" C-m

    echo ""
    echo "=========================================="
    echo " System Started!"
    echo "=========================================="
    echo ""
    echo " View logs:  tmux attach -t $SESSION_NAME"
    echo " Stop:       $0 stop"
    echo ""
    echo " Windows:"
    echo "   0: ai_anomaly  - Anomaly AI Server (TensorRT)"
    echo "   1: ai_ocr      - OCR AI Server (TensorRT)"
    echo "   2: ros2        - ROS2 System (sensors + perception + control)"
    echo ""
    echo " tmux shortcuts:"
    echo "   Ctrl+B, N     - Next window"
    echo "   Ctrl+B, P     - Previous window"
    echo "   Ctrl+B, 0-2   - Jump to window"
    echo "   Ctrl+B, D     - Detach (keep running)"
    echo ""
}

start_ros_only() {
    echo "[ROS] Starting ROS2 system only..."
    cd $EMBEDDED_PATH
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch robot_bringup system.launch.py "$@"
}

stop_all() {
    echo "[STOP] Stopping all systems..."
    tmux kill-session -t $SESSION_NAME 2>/dev/null && echo "  Session '$SESSION_NAME' terminated" || echo "  Session not running"

    # 개별 프로세스도 정리
    pkill -f "anomaly_server.py" 2>/dev/null
    pkill -f "ocr_server.py" 2>/dev/null
    echo "  Cleanup complete"
}

check_status() {
    echo "=========================================="
    echo " System Status"
    echo "=========================================="
    echo ""

    echo "[tmux Session]"
    if tmux has-session -t $SESSION_NAME 2>/dev/null; then
        echo "  Session '$SESSION_NAME': Running"
        tmux list-windows -t $SESSION_NAME 2>/dev/null | sed 's/^/    /'
    else
        echo "  Session '$SESSION_NAME': Not running"
    fi
    echo ""

    echo "[AI Servers]"
    if pgrep -f "anomaly_server.py" > /dev/null; then
        echo "  Anomaly Server: Running (port 9001)"
    else
        echo "  Anomaly Server: Not running"
    fi
    if pgrep -f "ocr_server.py" > /dev/null; then
        echo "  OCR Server: Running (port 9002)"
    else
        echo "  OCR Server: Not running"
    fi
    echo ""

    echo "[ROS2 Nodes]"
    source /opt/ros/humble/setup.bash 2>/dev/null
    ros2 node list 2>/dev/null | grep -E "anomaly|ocr|robot" | sed 's/^/  /' || echo "  No nodes running"
    echo ""
}

case "${1:-start}" in
    start)
        shift
        start_all "$@"
        ;;
    ros-only)
        shift
        start_ros_only "$@"
        ;;
    stop)
        stop_all
        ;;
    status)
        check_status
        ;;
    *)
        echo "Usage: $0 [start|stop|status|ros-only]"
        echo ""
        echo "  start    - Start full system (default)"
        echo "  stop     - Stop all processes"
        echo "  status   - Check system status"
        echo "  ros-only - Start ROS2 only (AI servers already running)"
        exit 1
        ;;
esac
