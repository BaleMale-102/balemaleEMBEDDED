# 시스템 아키텍처 설계 문서

## 설계 일시
- 날짜: 2026-02-02
- 브랜치: redesign/full-rewrite-20260202

---

## 1. 설계 원칙

### 1.1 핵심 원칙
1. **단일 책임 원칙**: 각 노드는 하나의 명확한 기능만 수행
2. **모듈화**: 독립적으로 테스트/교체 가능한 컴포넌트
3. **시뮬레이션 우선**: 하드웨어 없이도 테스트 가능
4. **파라미터화**: 모든 설정은 YAML 파일로 관리
5. **안전 우선**: Watchdog, 속도 제한, 비상 정지

### 1.2 좌표계 규약
- **ROS 표준** (REP 103)
- X: 전방 (양수)
- Y: 좌측 (양수)
- Z: 상방 (양수)
- 회전: 반시계 방향 (양수)
- 단위: 거리 [m], 각도 [rad], 속도 [m/s], [rad/s]

### 1.3 명명 규칙
- 패키지: `snake_case` (예: `marker_detector`)
- 노드: `snake_case_node` (예: `marker_detector_node`)
- 토픽: `/category/name` (예: `/perception/markers`)
- 파라미터: `snake_case` (예: `wheel_radius`)

---

## 2. 패키지 구조

```
~/balemaleEMBEDDED/src/
│
├── interfaces/
│   └── robot_interfaces/              # 커스텀 메시지/서비스/액션
│       ├── msg/
│       │   ├── Marker.msg
│       │   ├── MarkerArray.msg
│       │   ├── TrackedMarker.msg
│       │   ├── LaneStatus.msg
│       │   ├── MotorCommand.msg
│       │   ├── DrivingState.msg
│       │   ├── MissionCommand.msg
│       │   └── MissionStatus.msg
│       ├── srv/
│       │   ├── SetRoute.srv
│       │   └── GetRobotState.srv
│       ├── action/
│       │   └── NavigateToMarker.action
│       ├── CMakeLists.txt
│       └── package.xml
│
├── drivers/
│   ├── camera_driver/                 # USB 카메라 드라이버
│   │   ├── camera_driver/
│   │   │   └── camera_node.py
│   │   ├── config/
│   │   │   └── camera_params.yaml
│   │   ├── launch/
│   │   │   └── camera.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── arduino_driver/                # Arduino 시리얼 브릿지
│       ├── arduino_driver/
│       │   ├── arduino_node.py
│       │   └── serial_protocol.py
│       ├── config/
│       │   └── arduino_params.yaml
│       ├── launch/
│       │   └── arduino.launch.py
│       ├── setup.py
│       └── package.xml
│
├── perception/
│   ├── marker_detector/               # ArUco 마커 검출
│   │   ├── marker_detector/
│   │   │   ├── detector_node.py
│   │   │   └── aruco_detector.py
│   │   ├── config/
│   │   │   ├── detector_params.yaml
│   │   │   └── camera_calibration.yaml
│   │   ├── launch/
│   │   │   └── detector.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── marker_tracker/                # 마커 추적 + Kalman 예측
│   │   ├── marker_tracker/
│   │   │   ├── tracker_node.py
│   │   │   ├── kalman_filter.py
│   │   │   └── marker_predictor.py
│   │   ├── config/
│   │   │   └── tracker_params.yaml
│   │   ├── launch/
│   │   │   └── tracker.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── lane_detector/                 # 바닥 라인 검출
│       ├── lane_detector/
│       │   ├── detector_node.py
│       │   ├── lane_detector.py
│       │   └── image_processor.py
│       ├── config/
│       │   └── lane_params.yaml
│       ├── launch/
│       │   └── lane.launch.py
│       ├── setup.py
│       └── package.xml
│
├── control/
│   ├── motion_controller/             # 상위 모션 제어 (FSM)
│   │   ├── motion_controller/
│   │   │   ├── controller_node.py
│   │   │   ├── state_machine.py
│   │   │   └── control_modes.py
│   │   ├── config/
│   │   │   └── controller_params.yaml
│   │   ├── launch/
│   │   │   └── controller.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── wheel_controller/              # 메카넘 역기구학 + PWM
│       ├── wheel_controller/
│       │   ├── wheel_node.py
│       │   ├── mecanum_kinematics.py
│       │   └── pwm_converter.py
│       ├── config/
│       │   └── wheel_params.yaml
│       ├── launch/
│       │   └── wheel.launch.py
│       ├── setup.py
│       └── package.xml
│
├── planning/
│   ├── route_planner/                 # 마커 경로 계획
│   │   ├── route_planner/
│   │   │   ├── planner_node.py
│   │   │   └── route_manager.py
│   │   ├── config/
│   │   │   └── marker_map.yaml
│   │   ├── launch/
│   │   │   └── planner.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── mission_manager/               # 미션 FSM
│       ├── mission_manager/
│       │   ├── mission_node.py
│       │   └── mission_fsm.py
│       ├── config/
│       │   └── mission_params.yaml
│       ├── launch/
│       │   └── mission.launch.py
│       ├── setup.py
│       └── package.xml
│
├── communication/
│   └── server_bridge/                 # 외부 서버 통신
│       ├── server_bridge/
│       │   ├── bridge_node.py
│       │   └── protocol_handler.py
│       ├── config/
│       │   └── server_params.yaml
│       ├── setup.py
│       └── package.xml
│
└── bringup/
    └── robot_bringup/                 # Launch 및 설정 통합
        ├── launch/
        │   ├── robot.launch.py        # 전체 시스템
        │   ├── drivers.launch.py      # 드라이버만
        │   ├── perception.launch.py   # 인식만
        │   ├── control.launch.py      # 제어만
        │   └── simulation.launch.py   # 시뮬레이션
        ├── config/
        │   ├── robot_params.yaml      # 통합 파라미터
        │   ├── marker_map.yaml        # 마커 위치
        │   └── camera_calibration.yaml
        ├── rviz/
        │   └── robot.rviz
        ├── setup.py
        └── package.xml
```

---

## 3. 노드 구성도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              External Server                                 │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │ TCP/WebSocket
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           server_bridge_node                                 │
│  - 외부 서버 연결                                                            │
│  - 미션 명령 수신/상태 송신                                                   │
└─────────────────────────────────────────────────────────────────────────────┘
          │ /mission/command                    ▲ /mission/status
          ▼                                     │
┌─────────────────────────────────────────────────────────────────────────────┐
│                          mission_manager_node                                │
│  - 미션 FSM (READY → RUNNING → COMPLETED)                                   │
│  - route_planner 연동                                                        │
└─────────────────────────────────────────────────────────────────────────────┘
          │ /route/set_route (srv)              ▲ /route/progress
          ▼                                     │
┌─────────────────────────────────────────────────────────────────────────────┐
│                          route_planner_node                                  │
│  - 마커 맵 관리                                                              │
│  - 현재/다음 타겟 마커 퍼블리시                                               │
│  - 회전 방향 계산                                                            │
└─────────────────────────────────────────────────────────────────────────────┘
          │ /navigation/target_marker           ▲ /navigation/marker_reached
          │ /navigation/turn_direction          │
          ▼                                     │
┌─────────────────────────────────────────────────────────────────────────────┐
│                        motion_controller_node                                │
│  - FSM: IDLE, DRIVING, TURNING, LINE_FOLLOWING, PARKING                     │
│  - 마커/라인 정보 → 속도 명령 변환                                            │
│  - 안전 제한 적용                                                            │
└─────────────────────────────────────────────────────────────────────────────┘
          │                                     ▲
          │ /cmd_vel                            │ /perception/tracked_marker
          │                                     │ /perception/lane_status
          ▼                                     │
┌─────────────────────────────────────────────────────────────────────────────┐
│                         wheel_controller_node                                │
│  - 메카넘 역기구학                                                           │
│  - cmd_vel → 4개 바퀴 속도                                                   │
│  - 각속도 → PWM 변환                                                         │
└─────────────────────────────────────────────────────────────────────────────┘
          │ /motor/command
          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          arduino_driver_node                                 │
│  - Serial 통신 (115200 baud)                                                 │
│  - 속도 명령 전송 (V vx vy wz)                                               │
│  - Watchdog (명령 없으면 자동 정지)                                          │
└─────────────────────────────────────────────────────────────────────────────┘
          │
          │ USB Serial (/dev/ttyUSB0)
          ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Arduino UNO                                     │
│  - MoebiusTech Motor Hat (PCA9685)                                          │
│  - 메카넘 역기구학 (내장)                                                    │
│  - Open-loop 모터 제어 (엔코더 없음)                                         │
└─────────────────────────────────────────────────────────────────────────────┘

※ IMU (MPU6050)는 Jetson에 I2C로 직접 연결됨 (별도 imu_mpu6050 드라이버)


                    ═══════════════════════════════════════
                                 인식 파이프라인
                    ═══════════════════════════════════════

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  camera_node     │  │  camera_node     │  │  camera_node     │
│  (front_cam)     │  │  (bottom_cam)    │  │  (side_cam)      │
│  /dev/video0     │  │  /dev/video2     │  │  /dev/video4     │
└──────────────────┘  └──────────────────┘  └──────────────────┘
          │                    │                    │
          │ /cam_front/        │ /cam_bottom/       │ /cam_side/
          │  image_raw         │  image_raw         │  image_raw
          ▼                    ▼                    ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│marker_detector   │  │lane_detector     │  │parking_line_node │
│ - ArUco 검출     │  │ - 차선 검출      │  │ - 주차선 검출    │
│ - 포즈 추정      │  │ - PID 서보       │  │ - 슬롯 마커      │
└──────────────────┘  └──────────────────┘  └──────────────────┘
          │                    │                    │
          │ /perception/       │ /perception/       │ /perception/
          │  markers           │  lane_status       │  parking_line
          ▼                    │                    │
┌──────────────────┐           │                    │
│marker_tracker    │           │                    │
│ - Kalman Filter  │           │                    │
│ - 마커 예측      │           │                    │
└──────────────────┘           │                    │
          │                    │                    │
          │ /perception/       │                    │
          │  tracked_marker    │                    │
          └────────────────────┴────────────────────┴──▶ motion_controller_node
```

---

## 4. 토픽 설계

### 4.1 센서 토픽 (입력)

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/cam_front/image_raw` | sensor_msgs/Image | v4l2_camera | marker_detector | 전방 카메라 (마커) |
| `/cam_bottom/image_raw` | sensor_msgs/Image | v4l2_camera | lane_detector | 하단 카메라 (차선) |
| `/cam_side/image_raw` | sensor_msgs/Image | v4l2_camera | parking_line_node | 측면 카메라 (주차선) |
| `/imu/data` | sensor_msgs/Imu | imu_mpu6050_node | motion_controller, safety_manager | IMU (Jetson I2C) |

### 4.2 인식 토픽 (처리)

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/perception/markers` | robot_interfaces/MarkerArray | marker_detector | marker_tracker | 검출된 마커들 |
| `/perception/tracked_marker` | robot_interfaces/TrackedMarker | marker_tracker | motion_controller | 추적 마커 (예측 포함) |
| `/perception/lane_status` | robot_interfaces/LaneStatus | lane_detector | motion_controller | 라인 상태 |
| `/perception/debug/marker_image` | sensor_msgs/Image | marker_detector | (디버그) | 마커 시각화 |
| `/perception/debug/lane_image` | sensor_msgs/Image | lane_detector | (디버그) | 라인 시각화 |

### 4.3 네비게이션 토픽

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/navigation/target_marker` | std_msgs/Int32 | route_planner | marker_tracker, motion_controller | 목표 마커 ID |
| `/navigation/turn_direction` | std_msgs/Float32 | route_planner | motion_controller | 회전 방향 (rad) |
| `/navigation/marker_reached` | std_msgs/Int32 | motion_controller | route_planner | 마커 도달 알림 |
| `/navigation/route_progress` | std_msgs/Float32 | route_planner | mission_manager | 경로 진행률 |

### 4.4 제어 토픽 (출력)

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/cmd_vel` | geometry_msgs/Twist | motion_controller | arduino_driver | 속도 명령 (vx, vy, wz) |
| `/arduino/status` | std_msgs/String | arduino_driver | (모니터링) | Arduino 응답 상태 |

**참고**: 엔코더 미사용으로 `/motor/command`, `/odom/wheel` 토픽은 사용하지 않음.
Arduino에서 직접 메카넘 역기구학 계산 후 PWM 출력.

### 4.5 상태/미션 토픽

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|------|------|--------|--------|------|
| `/mission/command` | robot_interfaces/MissionCommand | server_bridge | mission_manager | 미션 명령 |
| `/mission/status` | robot_interfaces/MissionStatus | mission_manager | server_bridge | 미션 상태 |
| `/state/driving` | robot_interfaces/DrivingState | motion_controller | (모니터링) | 주행 상태 |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | (각 노드) | (모니터링) | 진단 정보 |

---

## 5. 서비스 설계

| 서비스 | 타입 | 서버 | 클라이언트 | 설명 |
|--------|------|------|-----------|------|
| `/route/set_route` | robot_interfaces/SetRoute | route_planner | mission_manager | 경로 설정 |
| `/robot/get_state` | robot_interfaces/GetRobotState | motion_controller | (외부) | 로봇 상태 조회 |
| `/motor/enable` | std_srvs/SetBool | arduino_driver | motion_controller | 모터 활성화 |

---

## 6. 상태 머신 (FSM)

### 6.1 Mission FSM (mission_manager)

```
              START
                │
                ▼
┌───────────────────────────────┐
│           READY               │
│  - 대기 상태                   │
│  - 경로 미설정                 │
└───────────────────────────────┘
                │ /mission/command (START)
                ▼
┌───────────────────────────────┐
│          RUNNING              │◄──────┐
│  - route_planner 활성화        │       │
│  - motion_controller 시작      │       │
└───────────────────────────────┘       │
       │              │                 │
       │ PAUSE        │ route done      │ RESUME
       ▼              ▼                 │
┌────────────┐  ┌───────────────────────┴───┐
│  PAUSED    │  │        COMPLETED          │
└────────────┘  │  - 미션 완료               │
                │  - 서버에 결과 보고         │
                └───────────────────────────┘
```

### 6.2 Motion FSM (motion_controller)

```
                    ┌─────────────────────┐
                    │        IDLE         │
                    │  - 정지 상태         │
                    │  - cmd_vel = 0       │
                    └─────────────────────┘
                              │
              enable_drive    │
                              ▼
    ┌─────────────────────────────────────────────────────┐
    │                      DRIVING                         │
    │  - 마커 방향으로 전진                                 │
    │  - tracked_marker 기반 횡이동 보정                    │
    │  - 마커 도달 시 → 다음 상태                           │
    └─────────────────────────────────────────────────────┘
         │                  │                    │
         │ turn_needed      │ line_zone          │ reached_final
         ▼                  ▼                    ▼
┌──────────────┐  ┌──────────────────┐  ┌──────────────────┐
│   TURNING    │  │  LINE_FOLLOWING  │  │     PARKING      │
│ - 제자리 회전 │  │  - 라인 추종     │  │  - 정밀 정렬     │
│ - IMU 기반   │  │  - lane_status   │  │  - 최종 위치     │
└──────────────┘  └──────────────────┘  └──────────────────┘
         │                  │                    │
         │ turn_done        │ exit_zone          │ aligned
         └──────────────────┴────────────────────┘
                              │
                              ▼
                         DRIVING (loop)
```

---

## 7. 데이터 흐름 시퀀스

### 7.1 마커 네비게이션 시퀀스

```
┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
│ Camera  │  │Detector │  │ Tracker │  │ Motion  │  │  Wheel  │  │ Arduino │
└────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘
     │            │            │            │            │            │
     │  image     │            │            │            │            │
     │───────────>│            │            │            │            │
     │            │            │            │            │            │
     │            │  markers   │            │            │            │
     │            │───────────>│            │            │            │
     │            │            │            │            │            │
     │            │            │ tracked    │            │            │
     │            │            │───────────>│            │            │
     │            │            │            │            │            │
     │            │            │            │  cmd_vel   │            │
     │            │            │            │───────────>│            │
     │            │            │            │            │            │
     │            │            │            │            │  motor_cmd │
     │            │            │            │            │───────────>│
     │            │            │            │            │            │
     │            │            │            │            │    PWM     │
     │            │            │            │            │     ───────│───> Motors
     │            │            │            │            │            │
```

### 7.2 마커 손실 시 예측 시퀀스

```
┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐
│Detector │  │ Tracker │  │ Motion  │  │  Time   │
└────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘
     │            │            │            │
     │ markers=[] │            │            │    t=0: 마커 검출 안됨
     │───────────>│            │            │
     │            │            │            │
     │            │ predict()  │            │    Kalman 예측 모드
     │            │───────────>│            │
     │            │            │            │
     │            │ tracked    │            │
     │            │ (predicted)│            │
     │            │───────────>│            │
     │            │            │            │
     │            │            │ continue   │    예측 기반 주행 유지
     │            │            │ driving    │
     │            │            │            │
     │ markers=[1]│            │            │    t=0.5s: 마커 재검출
     │───────────>│            │            │
     │            │            │            │
     │            │ update()   │            │    Kalman 업데이트
     │            │            │            │
```

---

## 8. 파라미터 구조

### 8.1 통합 파라미터 파일 (robot_params.yaml)

```yaml
# ============================================
# 로봇 공통 파라미터
# ============================================
robot:
  frame_id: "base_link"

# ============================================
# 카메라 드라이버 (v4l2_camera 사용)
# ============================================
camera_front:
  device: "/dev/video0"
  width: 640
  height: 480
  fps: 30
  frame_id: "camera_front_link"

camera_bottom:
  device: "/dev/video2"
  width: 320
  height: 240
  fps: 30
  frame_id: "camera_bottom_link"

camera_side:
  device: "/dev/video4"
  width: 320
  height: 240
  fps: 30
  frame_id: "camera_side_link"

# ============================================
# Arduino 드라이버 (Open-loop 제어)
# ============================================
arduino_driver:
  port: "/dev/ttyUSB0"
  baudrate: 115200
  timeout: 0.1
  simulate: false           # 시뮬레이션 모드
  cmd_rate_hz: 20.0         # 명령 전송 주기
  watchdog_timeout: 0.5     # 명령 없으면 정지
  max_vx: 0.10              # 최대 전진 속도 (m/s)
  max_vy: 0.10              # 최대 횡방향 속도 (m/s)
  max_wz: 1.0               # 최대 회전 속도 (rad/s)

# ============================================
# 마커 검출
# ============================================
marker_detector:
  dictionary: "DICT_4X4_50"
  marker_size: 0.10  # meters
  min_marker_perimeter: 50  # pixels
  publish_debug_image: true

# ============================================
# 마커 추적
# ============================================
marker_tracker:
  process_noise: 0.01
  measurement_noise: 0.05
  prediction_timeout: 2.0  # seconds
  min_prediction_confidence: 0.3

# ============================================
# 라인 검출
# ============================================
lane_detector:
  roi_top_ratio: 0.5
  roi_bottom_ratio: 1.0
  sobel_threshold: 50
  min_line_pixels: 100
  kalman_q: 0.01
  kalman_r: 0.05
  publish_debug_image: true

# ============================================
# 모션 컨트롤러
# ============================================
motion_controller:
  # 속도 설정
  drive_speed: 0.05  # m/s
  turn_speed: 0.3    # rad/s
  line_follow_speed: 0.03  # m/s

  # 게인
  lateral_gain: 0.1    # 횡이동 보정
  heading_gain: 0.3    # 방향 보정
  lane_gain: 0.05      # 라인 추종

  # 임계값
  reach_distance: 0.30  # meters
  turn_tolerance: 0.1   # radians (~5.7 degrees)

  # 안전 제한
  max_linear_speed: 0.1   # m/s
  max_angular_speed: 0.5  # rad/s

  # 타이밍
  control_rate: 20.0  # Hz
  watchdog_timeout: 0.5  # seconds

# ============================================
# 휠 컨트롤러 (DEPRECATED - arduino_driver 사용)
# ============================================
# 메카넘 역기구학은 Arduino 펌웨어에서 처리
# Arduino 파라미터 (펌웨어 내장):
#   WHEEL_RADIUS: 0.040m
#   WHEEL_BASE_X: 0.075m
#   WHEEL_BASE_Y: 0.090m
#   MAX_VEL_LINEAR: 0.10 m/s
#   MAX_VEL_ANGULAR: 1.0 rad/s
#   MAX_PWM: 2000
#   MIN_PWM: 300
#   WATCHDOG_MS: 300

# ============================================
# 경로 계획
# ============================================
route_planner:
  marker_map_file: "marker_map.yaml"

# ============================================
# 미션 관리
# ============================================
mission_manager:
  auto_start: false
```

---

## 9. 빌드 의존성

### 9.1 시스템 의존성
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- OpenCV 4.x (with contrib)

### 9.2 ROS2 패키지 의존성
```xml
<!-- 공통 -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>

<!-- 카메라 -->
<depend>cv_bridge</depend>
<depend>image_transport</depend>

<!-- 마커 검출 -->
<exec_depend>opencv-contrib-python</exec_depend>
<depend>tf2_ros</depend>

<!-- 제어 -->
<depend>diagnostic_msgs</depend>
```

### 9.3 Python 의존성
```
numpy>=1.21.0
opencv-contrib-python>=4.5.0
pyserial>=3.5
transforms3d>=0.4.1
```

---

## 10. 테스트 전략

### 10.1 단위 테스트
- 각 모듈별 독립 테스트
- Mock 객체로 의존성 분리

### 10.2 통합 테스트
- 시뮬레이션 모드로 전체 파이프라인 테스트
- 녹화된 데이터로 재현 테스트

### 10.3 하드웨어 테스트
- 개별 센서 동작 확인
- 모터 응답 확인
- 전체 시스템 동작 확인
