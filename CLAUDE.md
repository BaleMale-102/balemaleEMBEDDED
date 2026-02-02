# CLAUDE.md - RC Car Autonomous Parking System

## ⚠️ 코드 작성 전 필수 확인
- **토픽/메시지 사용 시 이 문서 참조**
- **msg 필드 접근 시 아래 정의 확인**
- **좌표계**: ROS 표준 (X-전방, Y-좌측, Z-상방, 반시계 양수)
- **단위**: 거리 m, 각도 rad, 속도 m/s

---

## 패키지 목록

| 패키지명 | 역할 | 위치 |
|---------|------|------|
| **rc_interfaces** | 커스텀 ROS2 메시지 정의 | src/Utils/ |
| **rc_perception** | 인식 (차선, ArUco 마커, 주차선) | src/Perception/ |
| **rc_imu_mpu6050** | MPU6050 IMU 드라이버 | src/Perception/ |
| **rc_localization** | EKF 기반 위치추정 | src/Planning/ |
| **rc_mission** | 미션 FSM 및 경로 계획 | src/Planning/ |
| **rc_control_stack** | 제어 스택 (차선/마커 추종, 주차) | src/Control/ |
| **arduino_bridge** | Arduino UART 브릿지 + Odometry | src/Control/ |
| **rc_mqtt_bridge** | MQTT 서버 통신 브릿지 | src/Utils/ |
| **rc_bringup** | Launch 파일 모음 | src/Utils/ |

---

## 커스텀 메시지 정의

### MarkerStatus.msg
ArUco 마커 인식 결과
```
std_msgs/Header header
bool valid                # 마커 검출 여부
int32 id                  # 마커 ID (0-15: 도로, 16-27: 주차칸)
float32 rel_x             # 카메라 기준 X (오른쪽 양수, m)
float32 rel_y             # 카메라 기준 Y (아래 양수, m)
float32 rel_z             # 카메라 기준 Z (전방 양수, m)
float32 rel_yaw           # 마커 yaw (rad)
float32 quality           # 검출 품질 (0 ~ 1)
```

### DriveCmd.msg
주행 명령
```
std_msgs/Header header
bool enable               # 주행 활성화
float32 vx                # 전진 속도 (m/s, 전방 양수)
float32 vy                # 횡방향 속도 (m/s, 왼쪽 양수)
float32 wz                # 회전 속도 (rad/s, 반시계 양수)
string source             # 명령 출처 (디버그용)
```

### DrivingState.msg
주행 상태 (실시간 제어 상태 + 에러 정보)
```
std_msgs/Header header
string state              # 현재 제어 모드 (IDLE, DRIVE, TURNING, PARK 등)
int32 target_marker_id    # 목표 마커 ID (-1: 없음)
int32 observed_marker_id  # 현재 관측 마커 ID (-1: 없음)
float32 ex                # X 방향 에러 (m)
float32 ey                # Y 방향 에러 (m)
float32 eyaw              # Yaw 에러 (rad)
float32 confidence        # 상태 신뢰도 (0 ~ 1)
string detail             # 상세 정보 (디버그용)
```

### TaskCmd.msg
서버로부터 받는 태스크 명령
```
std_msgs/Header header
int32[] route_ids         # 경유 마커 ID 리스트
int32 goal_id             # 최종 목표 마커 ID
string goal               # 최종 목표 (마커 ID 또는 슬롯명) - 문자열
string task_type          # 태스크 유형 (PICKUP, DROPOFF, RETURN 등)
string task_id            # 태스크 고유 ID
```

### TaskStatus.msg
태스크 수행 상태
```
std_msgs/Header header
string task_id            # 태스크 고유 ID
string status             # 상태 (RUNNING, COMPLETED, FAILED, CANCELLED)
string current_state      # 현재 FSM 상태
int32 current_marker_id   # 현재 위치 마커 ID
float32 progress          # 진행률 (0 ~ 1)
string message            # 상태 메시지
```

### LaneStatus.msg
차선 인식 결과
```
std_msgs/Header header
bool in_lane              # 차선 검출 여부
float32 offset_px         # 중심으로부터 오프셋 (픽셀)
float32 offset_norm       # 정규화된 오프셋 (-1 ~ +1, 왼쪽이 음수)
float32 angle             # 차선 각도 (rad, 반시계가 양수)
float32 quality           # 검출 품질 (0 ~ 1)
```

### ParkingLineStatus.msg
주차선 인식 결과
```
std_msgs/Header header
bool valid                # 주차선 검출 여부
float32 offset_norm       # 정규화된 오프셋 (-1 ~ +1)
float32 angle             # 주차선 각도 (rad)
float32 quality           # 검출 품질 (0 ~ 1)
```

---

## 토픽 목록

### 서버 통신 (MQTT 브릿지)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/server/task_cmd` | TaskCmd | mqtt_bridge | mission_manager | 서버 태스크 명령 |
| `/server/task_status` | TaskStatus | mission_manager | mqtt_bridge | 태스크 상태 보고 |

### 인식 (Perception)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/perception/lane` | LaneStatus | lane_node_v2 | control_stack | 차선 인식 결과 |
| `/perception/marker_status` | MarkerStatus | marker_pose | control_stack, mission_manager, ekf_localization | 전방 마커 인식 |
| `/perception/marker_pose` | PoseStamped | marker_pose | - | 마커 포즈 (ROS 좌표계) |
| `/perception/parking_line` | ParkingLineStatus | parking_line | control_stack | 주차선 인식 |
| `/perception/slot_marker_pose` | PoseStamped | slot_marker | control_stack | 주차칸 마커 포즈 |

### 미션 (Mission)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/mission/state` | String | mission_manager | control_stack | 현재 미션 상태 (IDLE, DRIVE, TURNING, PARK 등) |
| `/mission/turn_target_rad` | Float32 | mission_manager | control_stack | 턴 목표 각도 |
| `/mission/target_marker` | Int32 | mission_manager | control_stack | 목표 마커 ID |
| `/mission/marker_reached` | Int32 | control_stack | mission_manager | 마커 도달 알림 |
| `/mission/turn_done` | Bool | control_stack | mission_manager | 턴 완료 알림 |
| `/mission/align_done` | Bool | control_stack | mission_manager | 정렬 완료 알림 |

### 제어 (Control)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/control/enable_drive` | Bool | mission_manager | control_stack | 주행 활성화 |
| `/control/drive_cmd` | DriveCmd | control_stack | safety_manager | 제어 명령 |
| `/control/drive_cmd_safe` | DriveCmd | safety_manager | arduino_bridge, ekf_localization | 안전 처리된 명령 |
| `/control/drive_cmd_emergency` | DriveCmd | mqtt_bridge | safety_manager | 긴급 정지 명령 |
| `/driving/state` | DrivingState | control_stack | mqtt_bridge | 주행 상태 |

### 위치추정 (Localization)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/localization/pose` | PoseStamped | ekf_localization | mqtt_bridge | 추정 위치 |
| `/localization/odom` | Odometry | ekf_localization | - | Odometry 출력 |
| `/localization/fix_valid` | Bool | ekf_localization | - | 마커 기반 위치 유효 여부 |

### 센서 (Sensors)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/imu/data` | Imu | imu_mpu6050 | safety_manager, mission_manager, ekf_localization | 필터링된 IMU |
| `/imu/data_raw` | Imu | imu_mpu6050 | - | Raw IMU |
| `/odom/wheel` | Odometry | arduino_bridge | ekf_localization | 휠 오도메트리 |
| `/arduino/status` | String | arduino_bridge | - | Arduino 상태 |
| `/cam_front/image_raw` | Image | v4l2_camera | marker_pose | 전방 카메라 |
| `/cam_bottom/image_raw` | Image | v4l2_camera | lane_node_v2 | 하단 카메라 |
| `/cam_side/image_raw` | Image | v4l2_camera | parking_line, slot_marker | 측면 카메라 |

### 주차 (Parking)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/mission/target_slot` | Int32 | mission_manager | control_stack | 목표 주차칸 ID |
| `/parking/ready_to_drop` | Bool | control_stack | (로봇팔) | 주차 완료, 적재/하차 준비 |
| `/parking/drop_complete` | Bool | (로봇팔) | control_stack | 적재/하차 완료 |

---

## 주요 파라미터

### control_stack.yaml
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `drive_vx` | 0.003 | 주행 전진 속도 (m/s) |
| `drive_vy_gain` | -0.03 | 횡방향 보정 게인 |
| `reach_distance` | 0.30 | 마커 도달 판정 거리 (m) |
| `extra_forward_sec` | 1.0 | 마커 사라짐 후 추가 전진 시간 |
| `turn_wz` | 0.15 | 회전 속도 (rad/s) |
| `max_vx` | 0.05 | 최대 전진 속도 |
| `max_vy` | 0.05 | 최대 횡방향 속도 |
| `max_wz` | 0.5 | 최대 회전 속도 |

### safety_manager (in control_stack.yaml)
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `watchdog_timeout_sec` | 0.3 | 명령 없으면 정지 |
| `use_imu_heading_hold` | true | IMU 기반 직진 보정 |
| `heading_hold_kp` | 0.4 | Heading hold P 게인 |
| `slew_rate_vx` | 0.5 | 가속 제한 |

### lane_v2.yaml
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `image_topic` | /cam_bottom/image_raw | 입력 이미지 |
| `line_color` | black | 차선 색상 (black/white) |
| `use_adaptive` | true | Adaptive threshold 사용 |
| `adaptive_block_size` | 25 | 블록 크기 |
| `roi_top_ratio` | 0.3 | 상단 제외 비율 |
| `kalman_q` | 0.005 | Kalman process noise |
| `kalman_r` | 0.05 | Kalman measurement noise |

### ekf_localization.yaml
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `marker_map_yaml` | "" | 마커 맵 파일 경로 (launch에서 설정) |
| `map_unit_scale` | 0.01 | 맵 단위 스케일 (cm → m) |
| `cam_offset_x` | 0.06 | 카메라 오프셋 X (m) |
| `min_marker_quality` | 0.25 | 최소 마커 품질 |

### arduino_bridge.yaml
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `port` | /dev/ttyUSB0 | 시리얼 포트 |
| `baud` | 115200 | 통신 속도 |
| `max_pwm` | 3000 | Arduino PWM 최대값 |

### mqtt_bridge.yaml
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `car_id` | car_01 | 차량 ID |
| `mqtt_host` | localhost | MQTT 브로커 주소 |
| `mqtt_port` | 1883 | MQTT 포트 |

---

## 노드 실행파일 매핑

| 패키지 | 실행파일 | 노드명 |
|--------|----------|--------|
| rc_mission | mission_manager_node | mission_manager_node |
| rc_mqtt_bridge | mqtt_bridge_node | mqtt_bridge_node |
| rc_control_stack | control_stack_node | control_stack_node |
| rc_control_stack | safety_manager_node | safety_manager_node |
| arduino_bridge | arduino_bridge_node | arduino_bridge_node |
| rc_perception | lane_node_v2 | lane_node |
| rc_perception | lane_node_v3 | lane_node |
| rc_perception | marker_pose_node | marker_pose_node |
| rc_perception | parking_line_node | parking_line_node |
| rc_perception | slot_marker_node | slot_marker_node |
| rc_imu_mpu6050 | imu_mpu6050_node | imu_node |
| rc_localization | ekf_localization_node | ekf_localization_node |

---

## Launch 파일

### 전체 시스템 실행
```bash
ros2 launch rc_bringup system.launch.py
ros2 launch rc_bringup system.launch.py marker_map_yaml:=/path/to/marker_map.yaml
ros2 launch rc_bringup system.launch.py show_debug:=true
```

### 개별 모듈 실행
```bash
ros2 launch rc_bringup sensors.launch.py           # 카메라, IMU, TF
ros2 launch rc_perception perception.launch.py     # 인식 노드들
ros2 launch rc_localization localization.launch.py # EKF 위치추정
ros2 launch rc_control_stack control.launch.py     # 제어 노드들
ros2 launch rc_mqtt_bridge mqtt_bridge.launch.py   # MQTT 브릿지
```

---

## 마커 맵 (marker_map.yaml)

### 도로 마커 (ID 0-15)
| ID | 위치 (cm) | 설명 |
|----|-----------|------|
| 0 | (60, 18) | 시작 위치 |
| 1 | (60, 54) | 첫번째 갈림길 |
| 2 | (18, 54) | 좌측 상단 |
| 3 | (102, 54) | 우측 상단 |
| 4 | (18, 111) | 좌측 중간 |
| 5 | (42.5, 111) | 주차구역 1 |
| 6 | (54.5, 111) | 주차구역 2 |
| 7 | (66, 111) | 주차구역 3 |
| 8 | (78, 111) | 주차구역 4 |
| 9 | (102, 111) | 우측 중간 |
| 10 | (18, 187) | 좌측 하단 |
| 11 | (42.5, 187) | 주차구역 5 |
| 12 | (54.5, 187) | 주차구역 6 |
| 13 | (66, 187) | 주차구역 7 |
| 14 | (78, 187) | 주차구역 8 |
| 15 | (102, 187) | 우측 하단 |

### 주차칸 마커 (ID 16-27)
| ID | 위치 (cm) | 슬롯 ID | 설명 |
|----|-----------|---------|------|
| **구역 1** (y=82.5, ID 1과 5 사이) ||||
| 16 | (42.5, 82.5) | A1 | |
| 17 | (54.5, 82.5) | A2 | |
| 18 | (66, 82.5) | A3 | |
| 19 | (78, 82.5) | A4 | |
| **구역 2-1** (y=136.33, 3등분 첫번째) ||||
| 20 | (42.5, 136.33) | B1 | |
| 21 | (54.5, 136.33) | B2 | |
| 22 | (66, 136.33) | B3 | |
| 23 | (78, 136.33) | B4 | |
| **구역 2-2** (y=161.66, 3등분 두번째) ||||
| 24 | (42.5, 161.66) | C1 | |
| 25 | (54.5, 161.66) | C2 | |
| 26 | (66, 161.66) | C3 | |
| 27 | (78, 161.66) | C4 | |

### 맵 레이아웃 (상단에서 바라본 뷰)
```
      x=18    x=42.5  x=54.5  x=66   x=78   x=102
       |        |       |       |       |       |
y=18   |        |       |   0   |       |       |      <- 시작
       |        |       |       |       |       |
y=54   2--------|-------|---1---|-------|-------3      <- 첫번째 갈림길
       |        |       |       |       |       |
y=82.5 |       16      17      18      19       |      <- 주차칸 구역 1
       |        |       |       |       |       |
y=111  4-------5-------6-------7-------8-------9       <- 중간 라인
       |        |       |       |       |       |
y=136  |       20      21      22      23       |      <- 주차칸 구역 2-1
       |        |       |       |       |       |
y=162  |       24      25      26      27       |      <- 주차칸 구역 2-2
       |        |       |       |       |       |
y=187  10------11------12------13------14------15      <- 하단 라인
```

---

## 미션 FSM 상태

```
IDLE → DRIVE → STOP_AT_MARKER → ADVANCE_TO_CENTER → ALIGN_TO_MARKER
     → STOP_BUMP → TURNING → DRIVE → ... → PARK → FINISH
```

### 상태 설명
| 상태 | 설명 |
|------|------|
| IDLE | 대기 중 |
| DRIVE | 차선/마커 추종 주행 |
| STOP_AT_MARKER | 마커 앞 정지 |
| ADVANCE_TO_CENTER | 마커 중심으로 전진 |
| ALIGN_TO_MARKER | 마커에 정밀 정렬 |
| STOP_BUMP | 관성 안정화 (짧은 정지) |
| TURNING | 제자리 회전 |
| PARK | 주차 수행 중 |
| FINISH | 미션 완료 |
| ERROR | 오류 발생 |

---

## 코드 작성 규칙

### 일반 규칙
- ROS2 Humble 호환
- Python 노드: `rclpy` 사용
- 메시지 import: `from rc_interfaces.msg import ...`
- QoS: 센서 데이터는 `qos_profile_sensor_data` 사용

### 좌표계 변환
```python
# OpenCV (카메라) → ROS (로봇)
# OpenCV: X-right, Y-down, Z-forward
# ROS: X-forward, Y-left, Z-up
pose.x = tvec[2]   # forward
pose.y = -tvec[0]  # left
pose.z = -tvec[1]  # up
```

### 속도 제한
```python
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

# 사용
vx = clamp(vx, -self.max_vx, self.max_vx)
```

### 각도 정규화
```python
def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
```

### 토픽 파라미터화
```python
self.declare_parameter('topic_name', '/default/topic')
topic = self.get_parameter('topic_name').value
self.sub = self.create_subscription(MsgType, topic, self.callback, 10)
```

---

## 빌드 및 테스트

```bash
# 빌드
cd ~/balemaleEMBEDDED
colcon build --symlink-install

# 환경 설정
source install/setup.bash

# 테스트 실행
ros2 launch rc_bringup system.launch.py

# 토픽 확인
ros2 topic list
ros2 topic echo /perception/marker_status
ros2 topic echo /driving/state
```

---

## 디렉토리 구조

```
balemaleEMBEDDED/
├── config/
│   ├── cam_front_calib.yaml      # 카메라 캘리브레이션
│   └── marker_map.yaml           # 마커 맵 (전역)
├── src/
│   ├── Control/
│   │   ├── arduino_bridge/       # Arduino UART 브릿지
│   │   └── rc_control_stack/     # 제어 스택
│   ├── Perception/
│   │   ├── rc_imu_mpu6050/       # IMU 드라이버
│   │   └── rc_perception/        # 인식 노드들
│   ├── Planning/
│   │   ├── rc_localization/      # EKF 위치추정
│   │   └── rc_mission/           # 미션 FSM
│   └── Utils/
│       ├── rc_bringup/           # Launch 파일
│       ├── rc_interfaces/        # 커스텀 메시지
│       └── rc_mqtt_bridge/       # MQTT 브릿지
└── Arduino/                      # Arduino 펌웨어
```
