# RC Car Autonomous System

## 코드 작성 전 필수 확인
- **토픽/메시지 사용 시 이 문서 참조**
- **msg 필드 접근 시 아래 정의 확인**
- **좌표계**: ROS 표준 (X-전방, Y-좌측, Z-상방, 반시계 양수)
- **단위**: 거리 m, 각도 rad, 속도 m/s
- **시스템 유형**: 주차 로봇 (TARGET 차량을 적재/하역하는 로봇)
- **현재 상태**: 라인 인식 비활성화, 마커 전용 주행
- **입고 플로우(PARK)**: WAIT_VEHICLE → RECOGNIZE → LOAD → DRIVE → PARK → UNLOAD → RETURN_HOME → WAIT_VEHICLE
- **출차 플로우(EXIT)**: DRIVE → PARK_* → LOAD → RETURN_HOME → UNLOAD → WAIT_VEHICLE
- **주차 시스템**: PARK_DETECT → PARK_ALIGN_MARKER → PARK_FINAL (side_cam 마커 전용, slot_line 비활성화)
- **완료된 과제**: IMU 기반 Turn, ALIGN 모드, ANPR+장애물 검출 통합, Side Camera 주차 시스템, Loader 드라이버, Server Bridge (MQTT)
- **Loader 통신**: serial_bridge 모드 (ROS 토픽 → 별도 Python 스크립트 → Arduino)
- **ANPR 노드**: 별도 conda 환경(anpr_310)에서 실행 - balemaleAI 의존
---

## 패키지 구조

```
balemaleEMBEDDED/
├── Arduino/
│   ├── Motor/
│   │   └── Arduino_MoebiusTech_v2.ino    # 메카넘 휠 펌웨어
│   └── Lift/
│       └── carrying.ino                   # 적재 메커니즘 펌웨어
├── config/
│   ├── cam_front_calib.yaml          # 전방 카메라 캘리브레이션 (원본)
│   └── cam_side_calib.yaml           # 측면 카메라 캘리브레이션 (원본)
├── teleop.py                         # 수동 제어 스크립트
└── src/
    ├── bringup/robot_bringup/        # Launch 및 설정
    │   ├── config/
    │   │   ├── robot_params.yaml     # 전체 파라미터
    │   │   ├── marker_map.yaml       # 마커 맵
    │   │   ├── cam_front_calib.yaml  # 전방 카메라 캘리브레이션
    │   │   └── cam_side_calib.yaml   # 측면 카메라 캘리브레이션
    │   └── launch/
    │       ├── system.launch.py      # 진입점 (sensors + robot)
    │       ├── sensors.launch.py     # 센서 노드
    │       ├── robot.launch.py       # 인식/제어/계획 노드
    │       └── server_test.launch.py # 서버 통신 테스트 (센서 없이)
    ├── drivers/
    │   ├── camera_driver/            # USB 카메라 (V4L2)
    │   ├── arduino_driver/           # Arduino UART 브릿지 (모터)
    │   ├── loader_driver/            # Loader 드라이버 (serial_bridge 모드)
    │   └── imu_driver/               # MPU6050 I2C
    ├── perception/
    │   ├── marker_detector/          # ArUco 마커 검출 (front + side)
    │   ├── marker_tracker/           # Kalman 필터 추적
    │   ├── slot_line_detector/       # 노란 직사각형 검출 (비활성화 - 카메라 거리 문제)
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
| marker_detector | detector_node | side_marker_detector | robot |
| marker_tracker | tracker_node | marker_tracker | robot |
| slot_line_detector | detector_node | slot_line_detector | **비활성화** |
| anpr_detector | detector_node | anpr_detector | **별도 (conda anpr_310)** |
| motion_controller | controller_node | motion_controller | robot |
| loader_driver | loader_node | loader_driver | robot |
| mission_manager | manager_node | mission_manager | robot |
| server_bridge | bridge_node | server_bridge | robot |

---

## 커스텀 메시지 (robot_interfaces)

### Marker.msg
```
std_msgs/Header header    # 타임스탬프 및 프레임
int32 id                  # 마커 ID
geometry_msgs/Pose pose   # 3D 포즈 (카메라 프레임)
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
std_msgs/Header header             # 타임스탬프 및 프레임
int32 id                           # 마커 ID
geometry_msgs/Pose pose            # 현재 추정 포즈
geometry_msgs/Pose predicted_pose  # 예측 포즈 (0.5s ahead)
geometry_msgs/Vector3 velocity     # 속도
float32 distance                   # 거리 (m)
float32 angle                      # 각도 (rad)
bool is_detected                   # 현재 검출 여부
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

### SlotLineStatus.msg (주차용)
```
std_msgs/Header header
bool valid                    # 검출 유효 여부
float32 center_offset_x       # 직사각형 중심 X 오프셋 (m)
float32 center_offset_y       # 직사각형 중심 Y 오프셋 (m)
float32 angle                 # 직사각형 회전 각도 (rad)
float32 width                 # 검출된 폭 (픽셀)
float32 height                # 검출된 높이 (픽셀)
float32 confidence            # 검출 신뢰도 (0-1)
```

### ParkingStatus.msg (디버그용)
```
std_msgs/Header header
string sub_state              # 주차 서브상태 (DETECT, ALIGN_MARKER, etc.)
int32 detected_slot_id        # 검출된 슬롯 마커 ID (-1: 없음)
int32 target_slot_id          # 목표 슬롯 마커 ID
bool slot_verified            # 슬롯 그룹 검증 완료
float32 marker_distance       # 마커까지 거리 (m)
float32 marker_angle          # 마커 각도 (rad)
string message                # 상태 메시지
```

### LoaderCommand.msg (적재 메커니즘)
```
std_msgs/Header header
string command                # 명령: LOAD, UNLOAD, STOP
```

### LoaderStatus.msg (적재 상태)
```
std_msgs/Header header
string status                 # IDLE, LOADING, UNLOADING, DONE, ERROR, DISCONNECTED
bool is_loaded                # 차량 적재 여부
string message                # 상태 메시지
```

### PlateQuery.msg (서버 조회)
```
std_msgs/Header header
string plate_number           # 조회할 번호판
string car_id                 # 로봇 ID (선택)
```

### PlateResponse.msg (서버 응답)
```
std_msgs/Header header
string plate_number           # 번호판
bool verified                 # 입차 허용 여부
int32 assigned_slot_id        # 배정된 주차 마커 ID (-1: 미배정)
int32[] waypoint_ids          # 경유 마커 리스트
string message                # 응답 메시지
```

---

## 커스텀 서비스 (robot_interfaces)

### SetTargetMarker.srv
```
# Request
int32 marker_id               # 추적/내비게이션 대상 마커 ID
---
# Response
bool success                  # 성공 여부
string message                # 결과 메시지
```

### GetRobotState.srv
```
# Request (empty)
---
# Response
string state                  # 현재 FSM 상태
int32 current_marker_id       # 현재 마커 ID
float32 x                     # 추정 X 위치
float32 y                     # 추정 Y 위치
float32 yaw                   # 추정 Yaw 각도
float32 battery_voltage       # 배터리 전압
```

### SetRoute.srv
```
# Request
int32[] marker_ids            # 마커 ID 시퀀스 (경로)
---
# Response
bool success                  # 성공 여부
string message                # 결과 메시지
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
| `/perception/side_markers` | MarkerArray | side_marker_detector | mission_manager, motion_controller |
| `/perception/slot_rect` | SlotLineStatus | slot_line_detector | motion_controller |
| `/slot_line_detector/debug_image` | Image | slot_line_detector | (디버그) |

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
| `/parking/target_slot` | Int32 | mission_manager | motion_controller |
| `/parking/status` | ParkingStatus | mission_manager | (디버그) |
| `/parking/align_marker_done` | Bool | motion_controller | mission_manager |
| `/parking/align_rect_done` | Bool | motion_controller | mission_manager |
| `/parking/final_done` | Bool | motion_controller | mission_manager |

### 제어 (Control)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/control/enable_drive` | Bool | mission_manager | motion_controller |
| `/control/cmd_vel` | Twist | motion_controller | arduino_driver |
| `/driving/state` | DrivingState | motion_controller | server_bridge |

### 서버 통신 (MQTT - balemale backend)
| ROS 토픽 | 타입 | 발행자 | 구독자 |
|----------|------|--------|--------|
| `/server/task_cmd` | MissionCommand | server_bridge | mission_manager |
| `/server/task_status` | MissionStatus | mission_manager | server_bridge |
| `/plate/query` | PlateQuery | mission_manager | server_bridge |
| `/plate/response` | PlateResponse | server_bridge | mission_manager |

#### MQTT 토픽 (balemale/robot/{robotId}/...)
| MQTT 토픽 | 방향 | 설명 |
|-----------|------|------|
| `balemale/robot/{id}/cmd` | Subscribe | 서버 명령 (type 1:배차, 2:재경로, 3:출차) |
| `balemale/robot/{id}/map` | Subscribe | 노드 맵 정보 (type 4) |
| `balemale/robot/{id}/cmd/ack` | Subscribe | 서버의 요청 수신 확인 |
| `balemale/robot/{id}/request/dispatch` | Publish | 배차 요청 (번호판 인식 시) |
| `balemale/robot/{id}/request/reroute` | Publish | 재경로 요청 (장애물 감지 시) |
| `balemale/robot/{id}/event` | Publish | 로봇 이벤트 (상태 변경) |
| `balemale/robot/{id}/anomaly` | Publish | 이상 탐지 보고 (사람, 장애물) |
| `balemale/robot/{id}/ack` | Publish | 서버 명령 수신 확인 |
| `balemale/robot/{id}/heartbeat` | Publish | 하트비트 |

#### eventType 종류
| 값 | 설명 |
|----|------|
| `WAITING` | 대기 중 (IDLE, WAIT_VEHICLE, COMPLETED) |
| `LOADING` | 적재/하역 중 (LOAD, UNLOAD) |
| `DRIVING_DESTINATION` | 목적지로 이동 중 (DRIVE, TURNING, ALIGN 등) |
| `DRIVING_HOME` | 홈으로 복귀 중 (RETURN_HOME) |
| `PARKING` | 주차 중 (PARK_*) |
| `ESTOP` | 긴급 정지/오류 (ERROR) |
| `FAILED` | 미션 실패 |

#### anomalyType 종류
| 값 | 설명 |
|----|------|
| `HUMAN` | 사람 감지 |
| `OBSTACLE` | 장애물 감지 |
| `SYSTEM` | 시스템 오류 |

#### MQTT 페이로드 형식
**배차 요청 (dispatch request):**
```json
{
  "reqId": "req-001",
  "plate": "12가3456",
  "nowNodeId": 0,
  "targetLocation": "DESTINATION",
  "ts": "2026-01-27T10:40:00"
}
```

**서버 명령 (cmd type 1 - 배차 결과):**
```json
{
  "type": "1",
  "reqId": "req-001",
  "cmdId": "400c09d1-d181-4438-b1e8-2f244466000a",
  "payload": {
    "vehicleId": 1,
    "targetNodeId": 17,
    "path": [0, 1, 5, 17]
  }
}
```

**로봇 이벤트:**
```json
{
  "reqId": "req-001",
  "vehicleId": 1,
  "eventType": "LOADING",
  "ts": "2026-01-27T10:40:00"
}
```

**이상 탐지 보고 (anomaly):**
```json
{
  "reqId": "req-001",
  "vehicleId": 1,
  "eventType": "ESTOP",
  "anomalyType": "HUMAN",
  "edgeId": 1,
  "nowNodeId": 4,
  "nextNodeId": 5,
  "ts": "2026-01-27T10:40:00"
}
```

**ACK 송신 (로봇 → 서버):**
```json
{
  "cmdId": "400c09d1-d181-4438-b1e8-2f244466000a",
  "isAck": true
}
```

### 적재 (Loader)
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/loader/command` | LoaderCommand | mission_manager | loader_driver |
| `/loader/status` | LoaderStatus | loader_driver | mission_manager |

### Loader Bridge (serial_bridge.py - 별도 실행)
| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/motor_cmd` | String | loader_driver | serial_bridge | Arduino 명령 ("1", "2") |
| `/loading_status` | String | serial_bridge | loader_driver | Arduino 응답 ("completed") |

### 서버 맵 업데이트
| 토픽 | 타입 | 발행자 | 구독자 |
|------|------|--------|--------|
| `/server/map_updated` | Bool | server_bridge | mission_manager |

---

## 파라미터 설정 (robot_params.yaml)

### 센서 드라이버
```yaml
/cam_front:
  device: "/dev/video0"       # 환경에 따라 다름 (launch 인자로 변경 가능)
  width: 640
  height: 480
  fps: 30.0
  frame_id: "camera_front"
  camera_name: "cam_front"
  calibration_file: "cam_front_calib.yaml"

/cam_side:
  device: "/dev/video2"       # 환경에 따라 다름 (launch 인자로 변경 가능)
  width: 640
  height: 480
  fps: 30.0
  frame_id: "camera_side"
  camera_name: "cam_side"
  calibration_file: "cam_side_calib.yaml"

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

/loader_driver:
  port: "/dev/ttyACM0"        # Loader Arduino (직접 모드용)
  baudrate: 9600              # carrying.ino 기본값
  timeout: 0.1
  status_rate_hz: 10.0
  simulate: false
  use_bridge: true            # serial_bridge 모드 (기본값, 별도 실행 필요)
  operation_duration: 15.0    # 타임아웃 (Arduino 응답 없을 때 대체)
```

### 인식
```yaml
/marker_detector:
  dictionary: "DICT_4X4_50"
  marker_size: 0.04           # 4cm
  publish_debug_image: true
  image_topic: "/cam_front/image_raw"
  camera_info_topic: "/cam_front/camera_info"
  frame_id: "camera_front_link"

/side_marker_detector:
  dictionary: "DICT_4X4_50"
  marker_size: 0.04           # 4cm
  publish_debug_image: true
  image_topic: "/cam_side/image_raw"
  camera_info_topic: "/cam_side/camera_info"
  output_topic: "/perception/side_markers"
  frame_id: "camera_side_link"

/marker_tracker:
  process_noise: 0.01
  measurement_noise: 0.05
  prediction_timeout: 2.0
  min_prediction_confidence: 0.3
  camera_offset_x: 0.10       # 카메라→base_link 전방 오프셋

/anpr_detector:
  config_path: "/home/a102/balemaleAI/config.yaml"
  image_topic: "/cam_front/image_raw"
  publish_debug_image: true
  show_debug_window: false

/slot_line_detector:
  image_topic: "/cam_side/image_raw"
  output_topic: "/perception/slot_rect"
  publish_debug_image: true
  show_debug_window: false
  # HSV thresholds (노란색) - 개별 값으로 설정
  hsv_low_h: 20
  hsv_low_s: 100
  hsv_low_v: 100
  hsv_high_h: 35
  hsv_high_s: 255
  hsv_high_v: 255
  min_area: 500
  max_area: 50000
  min_aspect_ratio: 0.3
  max_aspect_ratio: 3.0
  pixel_to_meter: 0.001           # 픽셀→미터 변환
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
  advance_time: 0.8           # ADVANCE_TO_CENTER 고정 전진 시간 (초)
  # Turn (PD 제어)
  turn_wz: 0.06               # 최대 회전 속도 (상향됨)
  turn_wz_min: 0.0225         # 최소 회전 속도 (상향됨)
  turn_tolerance: 0.07        # ~4도
  turn_kp: 1.5
  turn_kd: 0.3
  turn_decel_zone: 0.35       # ~20도 (하향됨)
  turn_scale: 1.0             # 턴 각도 스케일 (부족하면 1.1~1.15)
  # Parking (주차 제어)
  park_creep_vx: 0.003        # PARK_DETECT 탐색 속도
  park_align_kp: 0.02         # 마커 정렬 P 게인
  park_rect_kp: 0.015         # 직사각형 정렬 P 게인
  park_final_kp: 0.02         # 최종 거리 조정 P 게인
  park_max_vx: 0.008          # 주차 시 최대 전후 속도
  park_max_vy: 0.01           # 주차 시 최대 좌우 속도
  park_min_speed: 0.003       # 주차 시 최소 속도
  park_marker_threshold: 0.05 # 마커 정렬 완료 임계값 (rad)
  park_rect_threshold: 0.015  # 직사각형 정렬 완료 임계값 (m)
  park_distance_threshold: 0.02 # 최종 거리 완료 임계값 (m)
  park_target_distance: 0.15  # 목표 주차 거리 (m)
  # Stall detection
  stall_check_interval: 0.5   # 스톨 체크 주기 (초)
  stall_boost_increment: 0.002 # 스톨 시 파워 증가량
  stall_max_boost: 0.015      # 최대 부스트 값
  stall_threshold_wz: 0.005   # 회전 스톨 임계값 (rad)
  stall_threshold_vy: 0.01    # 이동 스톨 임계값 (rad)
  stall_pulse_duration: 0.15  # 펄스 모드 지속 시간 (초)
  post_turn_search_timeout: 3.0 # Turn 후 마커 탐색 타임아웃 (초)
```

### 계획
```yaml
/mission_manager:
  update_rate: 10.0
  default_turn_angle: 1.57    # 90도
  # Full mission config
  home_marker_id: 0           # 홈 위치 마커 ID
  auto_start_waiting: true    # 자동으로 WAIT_VEHICLE 시작
  car_id: "car_01"            # 로봇 식별자
  # State timeouts (초과 시 ERROR)
  timeout_stop_at_marker: 2.0
  timeout_advance_to_center: 2.0
  timeout_align_to_marker: 10.0    # VY+WZ+탐색 포함 (상향됨)
  timeout_stop_bump: 0.5
  timeout_turning: 30.0
  timeout_park: 30.0
  # Full mission timeouts
  timeout_recognize: 60.0     # ANPR + 서버 응답 대기
  timeout_load: 30.0          # 적재 완료 대기
  timeout_unload: 30.0        # 하역 완료 대기
  timeout_return_home: 120.0  # 복귀 최대 시간
  # Parking sub-state timeouts
  timeout_park_detect: 10.0   # PARK_DETECT 탐색 시간
  timeout_park_recovery: 15.0 # PARK_RECOVERY 복구 시간
  timeout_park_align_marker: 15.0 # PARK_ALIGN_MARKER 정렬 시간
  timeout_park_align_rect: 10.0   # PARK_ALIGN_RECT 정렬 시간
  timeout_park_final: 10.0    # PARK_FINAL 최종 조정 시간
  # State transition delays
  delay_stop_at_marker: 0.5
  delay_stop_bump: 0.3

/server_bridge:
  mqtt_host: "43.202.0.116"         # EC2 서버 호스트 (http:// 없이)
  mqtt_port: 8000             # balemale backend 포트
  mqtt_username: ""
  mqtt_password: ""
  robot_id: 1                 # MQTT 토픽용 로봇 ID
  heartbeat_rate: 1.0         # 하트비트 주기 (Hz)
  status_rate: 2.0            # 상태 전송 주기 (Hz)
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

## Loader Arduino 프로토콜 (carrying.ino)

적재 메커니즘용 별도 Arduino (모터 Arduino와 다른 보드).

### 하드웨어
- DC 모터: ENA(6), IN1(5), IN2(4) (H-bridge) - 리프트 전후 이동
- 서보: 핀 11, 푸시 메커니즘 (0°=언로드, 90°=중립, 180°=로드)
- 리밋 스위치: FWD(9), REV(8) (INPUT_PULLUP, LOW=눌림)

### 시리얼 설정
- Port: `/dev/ttyACM0` (모터는 `/dev/ttyUSB0`)
- Baudrate: 9600

### 명령어
| 명령 | 설명 |
|------|------|
| `1` | Load (적재) - 전진→리밋→서보180°→후진→리밋 |
| `2` | Unload (하역) - 전진→리밋→서보0°→후진→리밋 |

### 응답 (carrying.ino 수정 필요 시)
현재 carrying.ino는 **응답 없음**. serial_bridge.py가 `LOAD_COMPLETE` 문자열을 기다리지만,
Arduino 코드에 응답 출력이 없으므로 **operation_duration 타임아웃**으로 완료 처리됨.

### 통신 모드
**Mode 1: serial_bridge 모드 (기본, use_bridge=true)**
```
mission_manager → /loader/command (LOAD)
        ↓
loader_driver → /motor_cmd ("1")
        ↓
serial_bridge.py → Arduino (시리얼 "1")
        ↓
Arduino: 동작 완료 후 "LOAD_COMPLETE" 출력 (구현 시)
        ↓
serial_bridge.py → /loading_status ("completed")
        ↓
loader_driver → /loader/status (DONE)
        ↓
mission_manager
```

**Mode 2: 직접 시리얼 모드 (use_bridge=false)**
```
loader_driver → 직접 시리얼 통신 → Arduino
```

### serial_bridge.py 실행 (~/testArduino/)
```bash
cd ~/testArduino
python3 serial_bridge.py
```

### 파라미터
```yaml
/loader_driver:
  port: "/dev/ttyACM0"        # 직접 모드용
  baudrate: 9600              # carrying.ino 기본값
  use_bridge: true            # serial_bridge 모드 (기본값)
  operation_duration: 15.0    # 타임아웃 (Arduino 응답 없을 때 대체)
```

---

## 시스템 통합 플로우

### 전체 통합 흐름 (WAIT → LOAD)
```
[WAIT_VEHICLE]
    │ ← /perception/anpr/detections (번호판 검출, ANPR 노드)
    ▼
[RECOGNIZE]
    │ → /plate/query (mission_manager)
    │ → server_bridge → MQTT: balemale/robot/{id}/request/dispatch
    │
    │ ← MQTT: balemale/robot/{id}/cmd (type=1, 배차 결과)
    │ ← server_bridge → /plate/response + /server/task_cmd
    ▼
[LOAD]
    │ → /loader/command (LOAD) (mission_manager)
    │ → /motor_cmd ("1") (loader_driver)
    │ → serial_bridge.py → Arduino (시리얼)
    │
    │ ← Arduino: "LOAD_COMPLETE" (또는 타임아웃)
    │ ← serial_bridge.py → /loading_status ("completed")
    │ ← loader_driver → /loader/status (DONE, is_loaded=true)
    ▼
[DRIVE] → ... → [PARK] → [UNLOAD] → [RETURN_HOME] → [WAIT_VEHICLE]
```

### 필수 프로세스
| 프로세스 | 위치 | 설명 |
|----------|------|------|
| ROS 노드 | launch | `ros2 launch robot_bringup system.launch.py` |
| serial_bridge | ~/testArduino/ | `python3 serial_bridge.py` (Loader Arduino) |
| ANPR | conda anpr_310 | `ros2 run anpr_detector detector_node` |

---

## 미션 FSM 상태

### 입고 플로우 (PARK - 서버 type 1)
```
       ┌──────────────────────────────────────────────────────────────────┐
       │                                                                  │
       ↓                                                                  │
IDLE → WAIT_VEHICLE → RECOGNIZE → LOAD → DRIVE → ... → PARK → UNLOAD → RETURN_HOME
           ↑              │                              │
           │              │                              ↓
           └──────────────┴─────────(ERROR)──────────> ERROR
```

### 출차 플로우 (EXIT - 서버 type 3)
```
서버 출차명령 (슬롯ID + 경로)
    ↓
DRIVE → ... → PARK_* → LOAD → RETURN_HOME → UNLOAD → WAIT_VEHICLE
 (슬롯으로)     (정렬)   (적재)  (홈 복귀)      (하역)
```

### 공통 서브사이클
```
주행 사이클 (DRIVE 내부):
DRIVE → STOP_AT_MARKER → ADVANCE_TO_CENTER → STOP_BUMP → TURNING → ALIGN_TO_MARKER → DRIVE

주차 사이클 (PARK 내부):
PARK_DETECT → PARK_ALIGN_MARKER → PARK_ALIGN_RECT → PARK_FINAL
     ↓ (잘못된 zone)
 PARK_RECOVERY ──────────────────────────────┘
```

### 풀 미션 상태 (Full Mission States)
| 상태 | 입고(PARK) | 출차(EXIT) |
|------|-----------|-----------|
| IDLE | 대기 중 | 대기 중 |
| WAIT_VEHICLE | 홈에서 차량 대기 (ANPR) | 출차 하역 후 대기 |
| RECOGNIZE | 서버에 번호판 조회 (60s) | - |
| LOAD | 홈에서 적재 → DRIVE | 슬롯에서 적재 → RETURN_HOME |
| DRIVE | 홈→슬롯 주행 | 홈→슬롯 / 슬롯→홈 주행 |
| PARK_* | 슬롯 정렬 → UNLOAD | 슬롯 정렬 → LOAD |
| UNLOAD | 슬롯에서 하역 → RETURN_HOME | 홈에서 하역 → WAIT_VEHICLE |
| RETURN_HOME | 슬롯→홈 빈차 복귀 | 슬롯→홈 차량 탑재 복귀 |

### 주행 상태 (Driving States)
| 상태 | 설명 | 타임아웃 | 다음 상태 |
|------|------|----------|-----------|
| DRIVE | 마커 추종 주행 | - | → STOP_AT_MARKER (마커 도달) |
| STOP_AT_MARKER | 마커 앞 정지 | 2.0s | → ADVANCE_TO_CENTER |
| ADVANCE_TO_CENTER | 마커 중심으로 전진 | 2.0s | → STOP_BUMP / ERROR |
| STOP_BUMP | 관성 안정화 | 0.5s | → TURNING / PARK |
| TURNING | 제자리 회전 | 30.0s | → ALIGN_TO_MARKER |
| ALIGN_TO_MARKER | Turn 후 마커 정렬 | 10.0s | → DRIVE |
| FINISH | 완료 | - | - |
| ERROR | 오류 | - | - |

### 주차 서브상태
| 상태 | 설명 | 타임아웃 | 타임아웃 후 전환 |
|------|------|----------|------------------|
| PARK_DETECT | side_cam으로 슬롯 마커 탐색 | 10s | → ERROR |
| PARK_RECOVERY | 잘못된 zone에서 올바른 zone으로 이동 | 15s | → PARK_DETECT |
| PARK_ALIGN_MARKER | 마커 angle로 전후 정렬 (vx) | 15s | → PARK_ALIGN_RECT |
| PARK_ALIGN_RECT | 노란 직사각형 중심으로 좌우 정렬 (vy) | 10s | → PARK_FINAL |
| PARK_FINAL | 마커 거리로 최종 위치 조정 (vy) | 10s | → FINISH |

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
└── base_link (IMU 위치)
    ├── camera_front (0.10m 전방, 0.0005m 우측, 0.04m 상, yaw 0°)
    ├── camera_side (0.10m 좌측, 0.075m 하, yaw +90°)
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

### ALIGN_TO_MARKER 2단계 정렬
Turn 후 정렬은 2단계로 진행:
```
Phase 1 (VY): 좌우 이동으로 마커 중심 정렬
Phase 2 (WZ): 회전으로 각도 미세 조정
→ 완료 후 주행 시작
```

### Post-Turn 마커 탐색
Turn 완료 후 마커가 안 보이면 자동 탐색:
```python
if marker_seen_during_turn and not marker_visible_at_turn_end:
    # 마커가 보였다가 사라짐 → Turn 반대 방향으로 탐색
    search_direction = -turn_direction
else:
    # 마커를 못 찾음 → Turn 방향으로 계속 탐색
    search_direction = turn_direction
```

### 스톨 감지 및 파워 부스트
모터가 공회전(명령은 가지만 움직이지 않음)하면 자동으로 파워 증가:
```python
# 스톨 감지
if yaw_change < stall_threshold_wz:  # WZ 스톨
    stall_wz_boost += stall_boost_increment
if angle_change < stall_threshold_vy:  # VY 스톨
    stall_vy_boost += stall_boost_increment

# 연속 3회 스톨 → 펄스 모드 (짧은 시간 강한 부스트)
if consecutive_stalls >= 3:
    pulse_mode = True
```

파라미터:
```yaml
stall_check_interval: 0.5     # 체크 주기 (초)
stall_boost_increment: 0.002  # 증가량
stall_max_boost: 0.015        # 최대 부스트
stall_pulse_duration: 0.15    # 펄스 지속 시간
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

## 주차 시스템 (Side Camera)

### 슬롯 그룹
```python
SLOT_GROUPS = {
    'A': range(16, 20),  # ID 16-19, y=82.5cm
    'B': range(20, 24),  # ID 20-23, y=136.33cm
    'C': range(24, 28),  # ID 24-27, y=161.66cm
}
```

### Side Camera 좌표 변환
Side camera는 base_link에서 10cm 좌측, yaw=+90° 장착
```
Side Camera Frame:        Robot Frame:
  Z (forward) ─────────→ +Y (좌측)
  X (right)   ─────────→ -X (후방)
  Y (down)    ─────────→ -Z (아래)
```

### 마커 검출 해석 (side camera, 좌측 장착)
- `marker.distance` = 차량과 마커 간 **측면 거리** (좌측)
- `marker.angle` = **전후방 오프셋** (angle > 0: 차가 마커보다 **뒤에** 있음 → 전진 필요)

### 제어 매핑 (좌측 카메라)
| 상태 | 센서 | 오차 | 제어 |
|------|------|------|------|
| PARK_ALIGN_MARKER | side_marker.angle | 전후 오차 | vx = +kp * angle (전진/후진) |
| PARK_FINAL | side_marker.distance | 거리 오차 | vy = -kp * error (좌측으로 이동) |

### 슬롯 검증 로직
```python
def is_same_zone(detected_id, target_id):
    """같은 그룹(A/B/C)인지 확인"""
    for zone, ids in SLOT_GROUPS.items():
        if detected_id in ids and target_id in ids:
            return True
    return False
```

잘못된 zone 검출 시 → PARK_RECOVERY로 전후 이동 후 재탐색

---

## 빌드 및 실행

```bash
# 빌드 (메인 시스템)
cd ~/balemaleEMBEDDED
colcon build
source install/setup.bash

# 전체 실행 (3개 터미널 필요)
# 터미널 1: 메인 시스템
ros2 launch robot_bringup system.launch.py

# 터미널 2: Loader serial_bridge (Arduino 통신)
cd ~/testArduino && python3 serial_bridge.py

# 터미널 3: ANPR (conda 환경, 아래 참조)

# 옵션
ros2 launch robot_bringup system.launch.py simulation:=true
ros2 launch robot_bringup system.launch.py show_debug:=false
ros2 launch robot_bringup system.launch.py cam_front_dev:=/dev/video1

# 서버 통신만 테스트 (센서 없이)
ros2 launch robot_bringup server_test.launch.py
ros2 launch robot_bringup server_test.launch.py simulation:=true  # MQTT 없이
```

### ANPR 노드 별도 실행 (conda anpr_310 환경)
```bash
# 터미널 2에서 실행
conda activate anpr_310
cd ~/balemaleEMBEDDED

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
| cam_front_dev | /dev/video0 | 전방 카메라 (C920) |
| cam_side_dev | /dev/video2 | 측면 카메라 (Brio 100) |
| arduino_port | /dev/ttyUSB0 | 모터 Arduino 포트 |

### Launch 파일
| 파일 | 설명 |
|------|------|
| system.launch.py | 전체 시스템 (sensors + robot) |
| sensors.launch.py | 센서 노드만 (카메라, IMU, 모터 Arduino) |
| robot.launch.py | 인식/제어/계획 노드 |
| server_test.launch.py | 서버 통신 테스트 (mission_manager + server_bridge) |

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

# 주차 테스트 (슬롯 17)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'START_PARK 17'"

# 주차 디버그
ros2 topic echo /parking/status
ros2 topic echo /perception/side_markers
ros2 run rqt_image_view rqt_image_view /slot_line_detector/debug_image

# 로더 테스트 (serial_bridge 먼저 실행)
cd ~/testArduino && python3 serial_bridge.py  # 터미널 1
ros2 topic echo /loader/status                 # 터미널 2
ros2 topic echo /motor_cmd                     # 터미널 3 (브릿지로 전송되는 명령)
ros2 topic echo /loading_status                # 터미널 4 (Arduino 응답)
ros2 topic pub --once /loader/command robot_interfaces/LoaderCommand "{command: 'LOAD'}"
ros2 topic pub --once /loader/command robot_interfaces/LoaderCommand "{command: 'UNLOAD'}"
# Arduino 응답 시뮬레이션 (Arduino 없을 때)
ros2 topic pub --once /loading_status std_msgs/String "data: 'completed'"

# 풀 미션 디버그
ros2 topic echo /plate/query
ros2 topic echo /plate/response

# 풀 미션 수동 시뮬레이션 - 입고 (센서/서버 없이)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'WAIT'"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'PLATE 12가3456'"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY 17 0,1,5'"
# → LOAD 상태로 전환, /loader/command 발행됨

# 출차 테스트 (슬롯에서 차량 회수)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'EXIT 1,5 17'"
# → DRIVE→PARK→LOAD→RETURN_HOME→UNLOAD 플로우

# 로더 완료 시뮬레이션 (serial_bridge 없을 때)
ros2 topic pub --once /loading_status std_msgs/String "data: 'completed'"

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
from robot_interfaces.msg import SlotLineStatus, ParkingStatus  # 주차용
from robot_interfaces.msg import LoaderCommand, LoaderStatus   # 적재용
from robot_interfaces.msg import PlateQuery, PlateResponse     # 서버 통신

# 서비스 import
from robot_interfaces.srv import SetTargetMarker, GetRobotState, SetRoute
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
