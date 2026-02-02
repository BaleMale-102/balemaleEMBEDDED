# ROS2 Autonomous RC Car - Redesign

자율주행 RC카 프로젝트의 새로운 아키텍처 설계 및 구현

## 개요

NVIDIA Jetson + Arduino UNO 기반의 4륜 메카넘 휠 RC카를 위한 ROS2 Humble 패키지입니다.

### 주요 기능
- ArUco 마커 기반 내비게이션
- 차선 인식 및 추종 (PID 기반 시각 서보)
- Kalman 필터 기반 마커 추적 및 예측
- MQTT 서버 통신
- FSM 기반 미션 관리
- Open-loop 모터 제어 + IMU heading hold

## 하드웨어 사양

| 구성요소 | 상세 |
|---------|------|
| 컴퓨터 | NVIDIA Jetson (Ubuntu 22.04 + ROS2 Humble) |
| 모터 컨트롤러 | Arduino UNO + MoebiusTech Motor Hat (PCA9685) |
| 구동 | DC 모터 4개 + 메카넘 휠 (엔코더 미사용) |
| 카메라 | USB 카메라 3개 (전방/하단/측면) |
| IMU | MPU6050 (Jetson I2C 연결) |
| 마커 | ArUco 4x4_50, 4cm |

### 제어 방식
- **Open-loop 모터 제어**: 엔코더 피드백 없이 속도 명령을 직접 PWM으로 변환
- **IMU Heading Hold**: MPU6050 자이로로 직진 보정
- **시각 서보 제어**: 차선/마커 인식 기반 횡방향 보정 (PID)

## 패키지 구조

```
src/
├── interfaces/
│   └── robot_interfaces/      # 커스텀 메시지 정의
├── drivers/
│   ├── arduino_driver/        # Arduino 시리얼 브릿지 (속도 명령)
│   └── camera_driver/         # USB 카메라 드라이버
├── perception/
│   ├── marker_detector/       # ArUco 마커 검출 (전방 카메라)
│   ├── marker_tracker/        # 마커 추적 (Kalman)
│   └── lane_detector/         # 차선 검출 (하단 카메라)
├── control/
│   ├── motion_controller/     # 모션 제어 + 시각 서보 PID
│   └── wheel_controller/      # 메카넘 역기구학 (사용 안함, arduino_driver로 대체)
├── planning/
│   ├── mission_manager/       # 미션 FSM
│   └── server_bridge/         # MQTT 브릿지
└── bringup/
    └── robot_bringup/         # Launch + Config
```

### 참고: 측면 카메라 (side_cam)
- 주차선 인식용 측면 카메라 사용
- `/cam_side/image_raw` 토픽 발행
- `parking_line_node`, `slot_marker_node`에서 구독

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

### 카메라 (Sensors)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cam_front/image_raw` | Image | 전방 카메라 (마커 검출) |
| `/cam_bottom/image_raw` | Image | 하단 카메라 (차선 검출) |
| `/cam_side/image_raw` | Image | 측면 카메라 (주차선 검출) |
| `/imu/data` | Imu | MPU6050 IMU 데이터 |

### 인식 (Perception)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/markers` | MarkerArray | 검출된 마커들 |
| `/perception/tracked_marker` | TrackedMarker | 추적 중인 마커 |
| `/perception/lane_status` | LaneStatus | 차선 상태 |
| `/perception/parking_line` | ParkingLineStatus | 주차선 상태 |

### 제어 (Control)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cmd_vel` | Twist | 속도 명령 (vx, vy, wz) |
| `/control/enable_drive` | Bool | 주행 활성화 |
| `/arduino/status` | String | Arduino 상태 |

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
- `arduino_driver.port`: Arduino 시리얼 포트

## 제어 아키텍처

### Open-loop 모터 제어
엔코더 미사용으로 인해 속도 피드백 제어(PID)를 사용하지 않습니다:

```
motion_controller → /cmd_vel → arduino_driver → Arduino (역기구학) → 모터
                                    ↑
                                IMU heading hold (직진 보정)
```

### 시각 서보 PID (유지)
차선/마커 인식 기반 횡방향 보정에 PID 사용:
- 차선 오프셋 → vy 보정 (lane_vy_pid)
- 차선 각도 → wz 보정 (lane_wz_pid)

### Localization (미구현)
- 엔코더 없이 위치 추정이 제한됨
- 마커 기반 위치 업데이트로 보완 예정
- IMU 적분 + 마커 보정 방식 계획 중

## FSM 상태 흐름

```
IDLE → DRIVE → STOP_AT_MARKER → ALIGN → TURN → DRIVE → ... → FINISH
```

## 라이선스

MIT License
