# CLAUDE.md - RC Car Autonomous System

## 코드 작성 전 필수 확인
- **토픽/메시지 사용 시 이 문서 참조**
- **msg 필드 접근 시 아래 정의 확인**
- **좌표계**: ROS 표준 (X-전방, Y-좌측, Z-상방, 반시계 양수)
- **단위**: 거리 m, 각도 rad, 속도 m/s
- **현재 상태**: 라인 인식 비활성화, 마커 전용 주행. DRIVE → STOP_AT_MARKER → ADVANCE_TO_CENTER → TURN → ALIGN_TO_MARKER → DRIVE 사이클
- **완료된 과제**: IMU 기반 Turn, ALIGN 모드, ANPR+장애물 검출 통합
- **ANPR 노드**: 별도 conda 환경(anpr_310)에서 실행 - balemaleAI 의존
---

## 패키지 구조

```
balemaleEMBEDDED/
├── Arduino/Motor/
│   └── Arduino_MoebiusTech_v2.ino    # 메카넘 휠 펌웨어
├── config/
│   └── cam_front_calib.yaml          # 카메라 캘리브레이션 (원본)
├── teleop.py                         # 수동 제어 스크립트
└── src/
    ├── bringup/robot_bringup/        # Launch 및 설정
    │   ├── config/
    │   │   ├── robot_params.yaml     # 전체 파라미터
    │   │   ├── marker_map.yaml       # 마커 맵
    │   │   └── cam_front_calib.yaml  # 카메라 캘리브레이션
    │   └── launch/
    │       ├── system.launch.py      # 진입점 (sensors + robot)
    │       ├── sensors.launch.py     # 센서 노드
    │       └── robot.launch.py       # 인식/제어/계획 노드
    ├── drivers/
    │   ├── camera_driver/            # USB 카메라 (V4L2)
    │   ├── arduino_driver/           # Arduino UART 브릿지
    │   └── imu_driver/               # MPU6050 I2C
    ├── perception/
    │   ├── marker_detector/          # ArUco 마커 검출
    │   ├── marker_tracker/           # Kalman 필터 추적
    │   ├── anpr_detector/            # 번호판/장애물 검출 (balemaleAI)
    │   └── lane_detector/            # 차선 검출 (비활성화)
    ├── control/
    │   └── motion_controller/        # 모션 제어
    ├── planning/
    │   ├── mission_manager/          # 미션 FSM
    │   └── server_bridge/            # MQTT 브릿지
    └── interfaces/
        └── robot_interfaces/         # 커스텀 메시지
```

---

## 노드 실행 매핑

| 패키지 | 실행파일 | 노드명 | Launch |
|--------|----------|--------|--------|
| camera_driver | camera_node | cam_front | sensors |
| camera_driver | camera_node | cam_side | sensors |
| arduino_driver | arduino_node | arduino_driver | sensors |
| imu_driver | imu_node | imu_node | sensors |
| marker_detector | detector_node | marker_detector | robot |
| marker_tracker | tracker_node | marker_tracker | robot |
| anpr_detector | detector_node | anpr_detector | **별도 (conda anpr_310)** |
| motion_controller | controller_node | motion_controller | robot |
| mission_manager | manager_node | mission_manager | robot |
| server_bridge | bridge_node | server_bridge | robot |

---

## 커스텀 메시지 (robot_interfaces)

### Marker.msg
```
int32 id                  # 마커 ID
geometry_msgs/Point position   # 위치 (카메라 프레임)
geometry_msgs/Quaternion orientation  # 방향
float32 distance          # 거리 (m)
float32 angle             # 각도 (rad)
float32 confidence        # 검출 신뢰도 (0~1)
```

### MarkerArray.msg
```
std_msgs/Header header
Marker[] markers          # 검출된 마커 배열
```

### TrackedMarker.msg
```
int32 id                  # 마커 ID
geometry_msgs/Pose pose   # 현재 추정 포즈
geometry_msgs/Pose predicted_pose  # 예측 포즈 (0.5s ahead)
geometry_msgs/Vector3 velocity     # 속도
float32 distance          # 거리 (m)
float32 angle             # 각도 (rad)
bool is_detected          # 현재 검출 여부
float32 prediction_confidence      # 예측 신뢰도
float32 tracking_duration          # 추적 지속 시간
float32 time_since_detection       # 마지막 검출 후 경과 시간
```

### LaneStatus.msg (비활성화)
```
bool valid                # 차선 검출 여부
float32 offset            # 중심 오프셋 (픽셀)
float32 offset_normalized # 정규화 오프셋 (-1~+1)
float32 angle             # 차선 각도 (rad)
float32 curvature         # 곡률
float32 confidence        # 신뢰도
```

### DrivingState.msg
```
string state              # 제어 모드 (IDLE, DRIVE, TURN 등)
int32 target_marker_id    # 목표 마커 ID (-1: 없음)
int32 observed_marker_id  # 관측 마커 ID (-1: 없음)
float32 error_x           # X 에러 (m)
float32 error_y           # Y 에러 (m)
float32 error_yaw         # Yaw 에러 (rad)
float32 cmd_vx            # 명령 vx
float32 cmd_vy            # 명령 vy
float32 cmd_wz            # 명령 wz
string detail             # 상세 정보
```

### MissionCommand.msg
```
string command            # 명령 (START, STOP)
int32[] waypoint_ids      # 경유 마커 ID 리스트
int32 final_goal_id       # 최종 목표 마커 ID
string task_type          # 태스크 유형 (PARK, DROPOFF 등)
string task_id            # 태스크 고유 ID
```

### MissionStatus.msg
```
string task_id            # 태스크 ID
string status             # 상태 (RUNNING, COMPLETED, FAILED)
string current_state      # 현재 FSM 상태
int32 current_waypoint_idx  # 현재 웨이포인트 인덱스
int32 current_marker_id   # 현재 마커 ID
float32 progress          # 진행률 (0~1)
string message            # 상태 메시지
```

### MotorCommand.msg (미사용 - Twist 사용)
```
int16 front_left          # 전좌 모터 PWM (-3000 ~ 3000)
int16 front_right         # 전우 모터 PWM
int16 rear_left           # 후좌 모터 PWM
int16 rear_right          # 후우 모터 PWM
```

### Detection.msg
```
# 클래스 상수
uint8 CLASS_PLATE=0
uint8 CLASS_STICKER=1
uint8 CLASS_PERSON=2
uint8 CLASS_BOX=3
uint8 CLASS_CONE=4

uint8 class_id            # 검출된 객체 클래스
string class_name         # 클래스 이름 ("plate", "sticker", "person", "box", "cone")
float32 confidence        # 검출 신뢰도 (0~1)
int32 x1, y1, x2, y2      # Bounding box (픽셀 좌표)
string text               # OCR 결과 (번호판인 경우)
bool has_sticker          # 스티커 유무 (번호판인 경우)
```

### DetectionArray.msg
```
std_msgs/Header header
Detection[] detections    # 검출된 객체 배열
uint8 num_plates          # 검출된 번호판 수
uint8 num_obstacles       # 검출된 장애물 수 (person + box + cone)
```

---

## 토픽 목록

### 센서 (Sensors)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/cam_front/image_raw` | Image | camera_driver | marker_detector |
| `/cam_front/camera_info` | CameraInfo | camera_driver | marker_detector |
| `/cam_side/image_raw` | Image | camera_driver | (주차용) |
| `/cam_side/camera_info` | CameraInfo | camera_driver | - |
| `/imu/data_raw` | Imu | imu_driver | - |
| `/imu/data` | Imu | imu_driver | motion_controller |
| `/arduino/status` | String | arduino_driver | - |

### 인식 (Perception)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/perception/markers` | MarkerArray | marker_detector | marker_tracker |
| `/perception/tracked_marker` | TrackedMarker | marker_tracker | motion_controller |
| `/perception/debug/marker_image` | Image | marker_detector | (디버그) |
| `/perception/anpr/detections` | DetectionArray | anpr_detector | mission_manager |
| `/perception/anpr/plate` | String | anpr_detector | (호환용) |
| `/perception/anpr/sticker` | String | anpr_detector | (호환용) |
| `/perception/anpr/debug_image` | Image | anpr_detector | (디버그) |

### 미션 (Mission)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/mission/state` | String | mission_manager | motion_controller |
| `/mission/target_marker` | Int32 | mission_manager | marker_tracker, motion_controller |
| `/mission/turn_target_rad` | Int32 | mission_manager | motion_controller |
| `/mission/current_heading` | Float32 | mission_manager | (디버그) |
| `/mission/marker_reached` | Int32 | motion_controller | mission_manager |
| `/mission/turn_done` | Bool | motion_controller | mission_manager |
| `/mission/align_done` | Bool | - | mission_manager |
| `/mission/test_cmd` | String | (수동) | mission_manager |

### 제어 (Control)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/control/enable_drive` | Bool | mission_manager | motion_controller |
| `/control/cmd_vel` | Twist | motion_controller | arduino_driver |
| `/driving/state` | DrivingState | motion_controller | server_bridge |

### 서버 통신 (MQTT)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/server/task_cmd` | MissionCommand | server_bridge | mission_manager |
| `/server/task_status` | MissionStatus | mission_manager | server_bridge |
| `/control/drive_cmd_emergency` | Twist | server_bridge | - |

---

## 파라미터 설정 (robot_params.yaml)

### 센서 드라이버
```yaml
/cam_front:
  device: "/dev/video0"
  width: 640
  height: 480
  fps: 30.0
  frame_id: "camera_front"
  calibration_file: "cam_front_calib.yaml"

/cam_side:
  device: "/dev/video4"
  width: 640
  height: 480
  fps: 30.0
  frame_id: "camera_side"

/arduino_driver:
  port: "/dev/ttyUSB0"
  baudrate: 115200
  watchdog_timeout: 0.15      # ROS측 (Arduino: 200ms)
  max_vx: 0.01                # 속도 제한 (m/s)
  max_vy: 0.015               # 좌우 이동 제한 (상향됨)
  max_wz: 0.15                # 회전 제한 (rad/s, 상향됨)
  wz_offset: 0.0              # 직진 보정

/imu_node:
  i2c_bus: 7                  # Jetson Orin Nano
  i2c_address: 0x68
  publish_rate_hz: 100.0
  calib_samples: 500          # 캘리브레이션 샘플 수 (강화됨)
  comp_alpha: 0.98            # Complementary filter gyro weight
  # DLPF: 21Hz bandwidth (0x04) - 빠른 회전 반응
```

### 인식
```yaml
/marker_detector:
  dictionary: "DICT_4X4_50"
  marker_size: 0.04           # 4cm
  publish_debug_image: true

/marker_tracker:
  process_noise: 0.01
  measurement_noise: 0.05
  prediction_timeout: 2.0
  camera_offset_x: 0.10       # 카메라→base_link 전방 오프셋

/anpr_detector:
  config_path: "/home/a102/balemaleAI/config.yaml"
  image_topic: "/cam_front/image_raw"
  publish_debug_image: true
  show_debug_window: false
```

### 제어
```yaml
/motion_controller:
  control_rate: 20.0
  max_vx: 0.008
  max_vy: 0.015               # 좌우 이동 속도 제한
  max_wz: 0.075               # 회전 속도 제한 (상향됨)
  # Marker approach
  marker_vx_kp: 0.015
  marker_vy_kp: 0.01          # 좌우 이동 게인 (상향됨)
  marker_wz_kp: 0.025
  marker_reach_distance: 0.25
  marker_min_distance: 0.08
  marker_lost_reach_distance: 0.35
  marker_align_threshold: 0.1   # 좌우 정렬 임계값 (rad, ~6도) - 이 이상이면 정렬 먼저
  advance_vx: 0.02
  advance_time: 0.7           # ADVANCE_TO_CENTER 고정 전진 시간 (초, 상향됨)
  # Turn (PD 제어)
  turn_wz: 0.06               # 최대 회전 속도 (상향됨)
  turn_wz_min: 0.0225         # 최소 회전 속도 (상향됨)
  turn_tolerance: 0.07        # ~4도
  turn_kp: 1.5
  turn_kd: 0.3
  turn_decel_zone: 0.35       # ~20도 (하향됨)
  turn_scale: 1.0             # 턴 각도 스케일 (부족하면 1.1~1.15)
```

### 계획
```yaml
/mission_manager:
  update_rate: 10.0
  default_turn_angle: 1.57    # 90도
  # State timeouts (초과 시 ERROR)
  timeout_stop_at_marker: 2.0
  timeout_advance_to_center: 2.0
  timeout_align_to_marker: 5.0
  timeout_stop_bump: 0.5
  timeout_turning: 30.0
  timeout_park: 30.0
  # State transition delays
  delay_stop_at_marker: 0.5
  delay_stop_bump: 0.3

/server_bridge:
  mqtt_host: "localhost"
  mqtt_port: 1883
  car_id: "car_01"
  status_rate: 2.0
```

---

## Arduino 프로토콜 (MoebiusTech v2)

### 시리얼 설정
- Baudrate: 115200
- 줄바꿈: `\n`
- Watchdog: 200ms (명령 없으면 정지)

### 명령어
| 명령 | 형식 | 설명 |
|------|------|------|
| V | `V vx vy wz` | 속도 명령 (m/s, rad/s) - ROS용 |
| D | `D vx vy wz` | 정규화 명령 (-1~1) |
| S / Z | `S` 또는 `Z` | 정지 |
| P | `P value` | MAX_PWM 설정 (300~4000) |
| C | `C idx scale` | 모터 보정 (idx: 0-3, scale: 0.5~1.5) |
| O | `O` | 오도메트리 (미지원: 0 반환) |
| ? | `?` | 상태 확인 |

### 속도-정규화 변환
```
MAX_VEL_LINEAR = 0.05 m/s   → normalized 1.0
MAX_VEL_ANGULAR = 0.5 rad/s → normalized 1.0
```

### 메카넘 휠 믹싱
```
A (FL) = vx + vy + wz
B (FR) = vx - vy - wz
C (RL) = vx - vy + wz
D (RR) = vx + vy - wz
```

### 모터 극성 및 보정
```cpp
SIGN_A = -1, SIGN_B = +1, SIGN_C = -1, SIGN_D = +1
motorScale[4] = {1.0, 1.1, 1.0, 1.0}  // B 모터 느려서 1.1 보정
```

---

## 미션 FSM 상태

```
IDLE → DRIVE → STOP_AT_MARKER → ADVANCE_TO_CENTER → STOP_BUMP → TURNING → ALIGN_TO_MARKER → DRIVE → ...
                                                                              ↓
                                                                       (마지막 waypoint)
                                                                              ↓
                                                                        PARK → FINISH
```

| 상태 | 설명 | 타임아웃 | 타임아웃 후 전환 |
|------|------|----------|------------------|
| IDLE | 대기 중 | - | - |
| DRIVE | 마커 추종 주행 | - | - |
| STOP_AT_MARKER | 마커 앞 정지 | 2.0s | → ADVANCE |
| ADVANCE_TO_CENTER | 마커 중심으로 전진 | 2.0s | → ERROR |
| STOP_BUMP | 관성 안정화 | 0.5s | → TURNING |
| TURNING | 제자리 회전 | 30.0s | → ALIGN_TO_MARKER (ERROR 아님) |
| ALIGN_TO_MARKER | Turn 후 마커 정렬 | 5.0s | → DRIVE (ERROR 아님) |
| PARK | 주차 수행 | 30.0s | → ERROR |
| FINISH | 완료 | - | - |
| ERROR | 오류 | - | - |

### 턴 각도 계산 (동적)
```python
# 이전→현재 마커에서 heading 추정
heading = atan2(current_y - prev_y, current_x - prev_x)
# 현재→다음 마커로 target direction 계산
target_dir = atan2(next_y - current_y, next_x - current_x)
# 턴 각도
turn_angle = wrap_angle(target_dir - heading)
```

---

## 마커 맵 (marker_map.yaml)

### 도로 마커 (ID 0-15)
| ID | 위치 (cm) | 설명 |
|----|-----------|------|
| 0 | (60, 18) | 시작 |
| 1 | (60, 54) | 첫 갈림길 |
| 2 | (18, 54) | 좌측 상단 |
| 3 | (102, 54) | 우측 상단 |
| 4 | (18, 111) | 좌측 중간 |
| 5-8 | (42.5-78, 111) | 주차구역 상단 |
| 9 | (102, 111) | 우측 중간 |
| 10 | (18, 187) | 좌측 하단 |
| 11-14 | (42.5-78, 187) | 주차구역 하단 |
| 15 | (102, 187) | 우측 하단 |

### 주차 마커 (ID 16-27)
| ID | 슬롯 | 위치 (cm) |
|----|------|-----------|
| 16-19 | A1-A4 | y=82.5 |
| 20-23 | B1-B4 | y=136.33 |
| 24-27 | C1-C4 | y=161.66 |

### 맵 레이아웃
```
      x=18    x=42.5  x=54.5  x=66   x=78   x=102
       |        |       |       |       |       |
y=18   |        |       |   0   |       |       |      ← 시작
       |        |       |       |       |       |
y=54   2--------|-------|---1---|-------|-------3      ← 첫 갈림길
       |        |       |       |       |       |
y=82.5 |       16      17      18      19       |      ← 주차 A
       |        |       |       |       |       |
y=111  4-------5-------6-------7-------8-------9       ← 중간
       |        |       |       |       |       |
y=136  |       20      21      22      23       |      ← 주차 B
       |        |       |       |       |       |
y=162  |       24      25      26      27       |      ← 주차 C
       |        |       |       |       |       |
y=187  10------11------12------13------14------15      ← 하단
```

---

## TF 프레임

```
odom (static)
└── base_link
    ├── camera_front (0.10m 전방, 0.01m 상)
    ├── camera_side (0.05m 우측, 0.03m 상, yaw -90°)
    └── imu_link (동일 위치)
```

---

## IMU 축 변환

**장착: 칩 Y→전방, 칩 Z→하방**

```python
# Raw → ROS 변환 (imu_node.py)
ax = ay_raw       # Robot X = Chip Y
ay = -ax_raw      # Robot Y = -Chip X
az = -az_raw      # Robot Z = -Chip Z
gx = gy_raw
gy = -gx_raw
gz = gz_raw       # yaw 방향 보정 (반전 제거)
```

---

## 모터 방향 보정 (motion_controller)

**중요: 실제 로봇 테스트 결과, vy와 wz 부호가 ROS 표준과 반대**

### 마커 접근 (vy)
```python
# 좌우 이동 부호 반전
vy = -self.marker_vy_kp * angle  # (원래: +)
```
- `angle > 0` (마커가 왼쪽) → `vy < 0` → 실제 왼쪽 이동

### 마커 접근 로직
```
|angle| > marker_align_threshold → [ALIGN] 좌우 정렬만 (vy, 게인 2배)
|angle| ≤ marker_align_threshold → [DRIVE] 직진 + 미세 보정 (vx + vy)

ALIGN_TO_MARKER 모드 → 게인 3배 (Turn 후 빠른 정렬)
```

### ALIGN 최소 속도
vy가 너무 작으면 모터가 반응하지 않음. 최소 속도 보장:
```python
# DRIVE 모드 ALIGN, ALIGN_TO_MARKER 모드 모두
if abs(vy) < 0.01 and abs(angle) > 0.05:
    vy = 0.01 if angle < 0 else -0.01
```

### Turn 제어 (wz)
```python
# turned 계산 (부호 반전 - IMU 방향 보정)
turned = start_yaw - current_yaw  # (원래: current - start)

# wz 출력 (부호 반전 - 모터 방향 보정)
cmd.angular.z = -wz

# turn_target 스케일 적용
turn_target_rad = raw_rad * turn_scale
```

**Turn이 부족하게 도는 경우:**
- `robot_params.yaml`에서 `turn_scale` 값을 1.1~1.15로 증가

---

## 빌드 및 실행

```bash
# 빌드 (메인 시스템)
cd ~/balemaleEMBEDDED_redesign
colcon build
source install/setup.bash

# 전체 실행
ros2 launch robot_bringup system.launch.py

# 옵션
ros2 launch robot_bringup system.launch.py simulation:=true
ros2 launch robot_bringup system.launch.py show_debug:=false
ros2 launch robot_bringup system.launch.py cam_front_dev:=/dev/video1
```

### ANPR 노드 별도 실행 (conda anpr_310 환경)
```bash
# 터미널 2에서 실행
conda activate anpr_310
cd ~/balemaleEMBEDDED_redesign

# 빌드 (처음 한번, setuptools 58.2.0 필요)
pip install setuptools==58.2.0
colcon build --packages-select anpr_detector

# 실행
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run anpr_detector detector_node --ros-args -p show_debug_window:=true
```

### Launch 인자
| 인자 | 기본값 | 설명 |
|------|--------|------|
| simulation | false | 시뮬레이션 모드 |
| show_debug | true | 디버그 창 표시 |
| cam_front_dev | /dev/video0 | 전방 카메라 |
| cam_side_dev | /dev/video4 | 측면 카메라 |
| arduino_port | /dev/ttyUSB0 | Arduino 포트 |

---

## 디버깅 명령

```bash
# 토픽 확인
ros2 topic list
ros2 topic echo /perception/tracked_marker
ros2 topic echo /perception/anpr/detections
ros2 topic echo /perception/anpr/plate
ros2 topic echo /driving/state
ros2 topic echo /mission/state

# ANPR 디버그 이미지 확인
ros2 run rqt_image_view rqt_image_view /perception/anpr/debug_image

# 테스트 미션 (마커 1 → 2)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START 1,2'"

# 긴급 정지
ros2 topic pub /control/cmd_vel geometry_msgs/Twist "{}"

# Arduino 직접 테스트
ros2 topic pub /control/cmd_vel geometry_msgs/Twist "{linear: {x: 0.005}}"
```

### 수동 제어 (teleop.py)
```bash
# Direct 모드 (기본) - ROS 없이 Arduino 직접 제어
python3 teleop.py

# ROS 모드 - ROS 노드 통해 제어
python3 teleop.py --ros

# 포트 지정
python3 teleop.py --port /dev/ttyUSB0
```

| 키 | 동작 |
|---|---|
| W/↑ | 전진 |
| S/↓ | 후진 |
| A/← | 좌측 이동 |
| D/→ | 우측 이동 |
| Q | 좌회전 |
| E | 우회전 |
| Space | 정지 |
| 1 | Enable drive (ROS only) |
| 2 | Disable drive (ROS only) |
| +/- | 속도 조절 |
| Ctrl+C | 종료 |

---

## 코드 작성 규칙

### 메시지 import
```python
from robot_interfaces.msg import Marker, MarkerArray, TrackedMarker
from robot_interfaces.msg import DrivingState, MissionCommand, MissionStatus
from robot_interfaces.msg import Detection, DetectionArray
```

### 좌표 변환 (OpenCV → ROS)
```python
# Camera frame (OpenCV): X-right, Y-down, Z-forward
# Robot frame (ROS): X-forward, Y-left, Z-up
ros_x = opencv_z + camera_offset_x  # forward
ros_y = -opencv_x                    # left
```

### 속도 제한
```python
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
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

### QoS 프로파일
```python
from rclpy.qos import qos_profile_sensor_data
# 센서 데이터 (카메라, IMU)는 sensor_data QoS 사용
```
