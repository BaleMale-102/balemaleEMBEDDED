# ROS2 Autonomous RC Car - Redesign

자율주행 RC카 프로젝트의 새로운 아키텍처 설계 및 구현

## 개요

NVIDIA Jetson + Arduino Mega 기반의 4륜 메카넘 휠 RC카를 위한 ROS2 Humble 패키지입니다.

### 주요 기능
- ArUco 마커 기반 내비게이션
- 차선 인식 및 추종
- Kalman 필터 기반 마커 추적 및 예측
- MQTT 서버 통신
- FSM 기반 미션 관리

## 하드웨어 사양

| 구성요소 | 상세 |
|---------|------|
| 컴퓨터 | NVIDIA Jetson (Ubuntu 22.04 + ROS2 Humble) |
| 모터 컨트롤러 | Arduino Mega 2560 |
| 구동 | DC 모터 4개 + 메카넘 휠 |
| 카메라 | USB 카메라 2개 (전방/하단) |
| IMU | MPU6050 (Arduino 연결) |
| 마커 | ArUco 4x4_50, 10cm |

## 패키지 구조

```
src/
├── interfaces/
│   └── robot_interfaces/      # 커스텀 메시지 정의
├── perception/
│   ├── marker_detector/       # ArUco 마커 검출
│   ├── marker_tracker/        # 마커 추적 (Kalman)
│   └── lane_detector/         # 차선 검출
├── control/
│   ├── motion_controller/     # 모션 제어
│   └── wheel_controller/      # 휠 제어 + Arduino
├── planning/
│   ├── mission_manager/       # 미션 FSM
│   └── server_bridge/         # MQTT 브릿지
└── bringup/
    └── robot_bringup/         # Launch + Config
```

## 빌드

```bash
cd ~/balemaleEMBEDDED
source /opt/ros/humble/setup.bash

# 전체 빌드
colcon build --packages-select \
    robot_interfaces \
    marker_detector marker_tracker lane_detector \
    motion_controller wheel_controller \
    mission_manager server_bridge \
    robot_bringup

source install/setup.bash
```

## 실행

### 전체 시스템
```bash
ros2 launch robot_bringup robot.launch.py
```

### 시뮬레이션 모드
```bash
ros2 launch robot_bringup simulation.launch.py
```

### 개별 노드
```bash
ros2 run marker_detector detector_node
ros2 run lane_detector detector_node
ros2 run motion_controller controller_node
```

## 토픽 구조

### 인식 (Perception)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/markers` | MarkerArray | 검출된 마커들 |
| `/perception/tracked_marker` | TrackedMarker | 추적 중인 마커 |
| `/perception/lane_status` | LaneStatus | 차선 상태 |

### 제어 (Control)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/control/cmd_vel` | Twist | 속도 명령 |
| `/control/enable_drive` | Bool | 주행 활성화 |
| `/motor/command` | MotorCommand | 모터 PWM |

### 미션 (Mission)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mission/state` | String | FSM 상태 |
| `/mission/target_marker` | Int32 | 목표 마커 ID |
| `/server/task_cmd` | MissionCommand | 서버 명령 |

## 설정 파일

### robot_params.yaml
- `marker_detector.marker_size`: 마커 크기 (m)
- `lane_detector.line_color`: 차선 색상 (black/white)
- `motion_controller.max_vx`: 최대 전진 속도
- `wheel_controller.serial_port`: Arduino 포트

## FSM 상태 흐름

```
IDLE → DRIVE → STOP_AT_MARKER → ALIGN → TURN → DRIVE → ... → FINISH
```

## 라이선스

MIT License
