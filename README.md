# 🚗 Smart Parking RC Car - ROS2 시스템 개선

## 📋 개선 요약

### Phase 1: Arduino 엔코더 PID 제어
- **파일**: `Arduino_motor_pid.ino`
- **특징**:
  - 4휠 독립 PID 속도 제어
  - 엔코더 기반 피드백
  - Odometry 출력 지원
  - 저속 정밀 제어 (deadband 보정)

### Phase 2: Lane Node v2
- **파일**: `rc_perception/lane_node_v2.py`
- **개선점**:
  - Adaptive threshold (조명 변화 대응)
  - Morphology 강화 (끊어진 라인 연결)
  - Sliding window 추적
  - Kalman filter 스무딩

### Phase 3: DrivingState 발행
- **파일**: `rc_control_stack/control_stack_node.py`
- **토픽**: `/driving/state` (DrivingState)
- **정보**: 모드, 마커 ID, 에러(ex, ey, eyaw), confidence

### Phase 4: EKF Localization
- **파일**: `rc_localization/ekf_localization_node.py`
- **센서 융합**:
  - ArUco Marker (업데이트)
  - IMU gyro (예측)
  - Wheel Odometry (예측 + 업데이트)

---

## 📁 파일 구조

```
ros2_ws/src/
├── Control/
│   ├── arduino_bridge/
│   │   ├── arduino_bridge/
│   │   │   └── arduino_bridge_node.py    # UART + Odometry
│   │   ├── config/arduino_bridge.yaml
│   │   ├── launch/arduino_bridge.launch.py
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── rc_control_stack/
│       ├── rc_control_stack/
│       │   ├── control_stack_node.py     # AUTO 컨트롤러 + DrivingState
│       │   └── safety_manager_node.py    # MUX + IMU Hold
│       ├── config/control_stack.yaml
│       ├── launch/control.launch.py
│       ├── package.xml
│       └── setup.py
│
├── Perception/
│   └── rc_perception/
│       ├── rc_perception/
│       │   ├── lane_node_v2.py           # 개선된 차선 인식
│       │   └── (기존 노드들...)
│       ├── config/lane_v2.yaml
│       ├── launch/perception.launch.py
│       ├── package.xml
│       └── setup.py
│
├── Planning/
│   └── rc_localization/
│       ├── rc_localization/
│       │   └── ekf_localization_node.py  # EKF 위치 추정
│       ├── config/ekf_localization.yaml
│       ├── launch/localization.launch.py
│       ├── package.xml
│       └── setup.py
│
└── Utils/
    └── rc_bringup/
        └── launch/system.launch.py       # 전체 시스템

Arduino/
└── Motor/
    └── Arduino_motor_pid.ino             # 엔코더 PID 펌웨어
```

---

## 🔧 설치

### 1. 패키지 복사
```bash
# 기존 workspace에 복사
cp -r ros2_ws/src/* ~/your_ws/src/

# 또는 새 workspace 생성
mkdir -p ~/smart_parking_ws/src
cp -r ros2_ws/src/* ~/smart_parking_ws/src/
```

### 2. 빌드
```bash
cd ~/smart_parking_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Arduino 펌웨어 업로드
```bash
# Arduino IDE에서 Arduino_motor_pid.ino 열어서 업로드
# 또는 arduino-cli 사용
arduino-cli compile --fqbn arduino:avr:uno Arduino/Motor/Arduino_motor_pid
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno Arduino/Motor/Arduino_motor_pid
```

---

## 🚀 실행

### 전체 시스템
```bash
ros2 launch rc_bringup system.launch.py \
    marker_map_yaml:=/path/to/marker_map.yaml \
    serial_port:=/dev/ttyUSB0
```

### 개별 실행
```bash
# 1. Perception만
ros2 launch rc_perception perception.launch.py show_debug:=true

# 2. Localization만
ros2 launch rc_localization localization.launch.py \
    marker_map_yaml:=/path/to/marker_map.yaml

# 3. Control만
ros2 launch rc_control_stack control.launch.py \
    serial_port:=/dev/ttyUSB0

# 4. Arduino Bridge만
ros2 launch arduino_bridge arduino_bridge.launch.py port:=/dev/ttyUSB0
```

---

## 📡 토픽 목록

### Perception
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/perception/lane` | LaneStatus | 차선 인식 |
| `/perception/marker_status` | MarkerStatus | 마커 상태 |
| `/perception/parking_line` | ParkingLineStatus | 주차선 |
| `/perception/slot_marker_pose` | PoseStamped | 주차칸 마커 |

### Localization
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/localization/pose` | PoseStamped | 추정 위치 |
| `/localization/odom` | Odometry | 추정 오도메트리 |
| `/localization/fix_valid` | Bool | 마커 fix 유효성 |

### Control
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/control/drive_cmd` | DriveCmd | AUTO 명령 |
| `/control/drive_cmd_emergency` | DriveCmd | 긴급 명령 |
| `/control/drive_cmd_safe` | DriveCmd | 최종 명령 |
| `/driving/state` | DrivingState | 주행 상태 |

### Odometry
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/odom/wheel` | Odometry | 휠 오도메트리 |
| `/arduino/status` | String | Arduino 상태 |

---

## ⚙️ 주요 파라미터 튜닝

### Lane Follow
```yaml
# config/control_stack.yaml
lane_vx: 0.15        # 전진 속도 (↓ 느리게, ↑ 빠르게)
lane_k_psi: 0.8      # heading 게인 (↑ 빠른 응답)
lane_k_y: 1.2        # crosstrack 게인 (↑ 강한 복귀)
```

### EKF Localization
```yaml
# config/ekf_localization.yaml
min_marker_quality: 0.25   # 마커 품질 필터 (↓ 허용적)
marker_timeout_sec: 1.0    # 마커 타임아웃 (↑ dead-reckoning 오래)
```

### Arduino PID
```cpp
// Arduino_motor_pid.ino
#define DEFAULT_KP  80.0f   // 비례 게인
#define DEFAULT_KI  40.0f   // 적분 게인
#define DEFAULT_KD  0.5f    // 미분 게인
```

---

## 🛠️ 하드웨어 파라미터 수정

### Arduino 핀 설정
`Arduino_motor_pid.ino` 상단의 핀 정의 수정:
```cpp
// 모터 방향 핀
#define DIR_A1  4
#define DIR_A2  5
// ...

// 엔코더 핀
#define ENC_A_A  2
#define ENC_A_B  A0
// ...
```

### 로봇 물리 파라미터
```cpp
#define WHEEL_RADIUS_M    0.030f   // 휠 반경
#define WHEEL_BASE_X_M    0.060f   // 휠 중심 X 거리
#define WHEEL_BASE_Y_M    0.075f   // 휠 중심 Y 거리
#define ENCODER_PPR       11       // 엔코더 PPR
#define GEAR_RATIO        30       // 기어비
```

---

## 🔄 데이터 흐름

```
                     ┌─────────────────────────────────────────┐
                     │              Mission Manager            │
                     │         /mission/state (DRIVE/PARK)     │
                     └─────────────────┬───────────────────────┘
                                       │
┌──────────────────────────────────────┼──────────────────────────────────────┐
│                                      ▼                                       │
│  ┌─────────────┐  ┌──────────────────────────────┐  ┌─────────────────────┐│
│  │   Cameras   │  │     control_stack_node       │  │   arduino_bridge    ││
│  │ (3x)        │  │  • Lane Follow (Stanley)     │  │   • UART TX/RX      ││
│  └──────┬──────┘  │  • Turning                   │  │   • Odometry RX     ││
│         │         │  • Align to Marker           │  └──────────┬──────────┘│
│         ▼         │  • Park FSM                  │             │           │
│  ┌──────────────┐ │  ──────────────────────────  │             ▼           │
│  │ Perception   │ │  Publishes:                  │  ┌─────────────────────┐│
│  │ • lane_v2    ├─▶  • /control/drive_cmd       │  │      Arduino        ││
│  │ • marker     │ │  • /driving/state           │  │   (PID + Encoder)   ││
│  │ • parking    │ └──────────────┬───────────────┘  │   • 4-wheel PID     ││
│  └──────┬───────┘                │                  │   • Odometry calc   ││
│         │                        ▼                  └──────────┬──────────┘│
│         │         ┌──────────────────────────────┐             │           │
│         │         │     safety_manager_node      │             │           │
│         │         │  • Emergency MUX             │             ▼           │
│         │         │  • Watchdog                  │  ┌─────────────────────┐│
│         │         │  • IMU Heading Hold          │  │   Mecanum Motors    ││
│         │         │  • Slew Rate Limiting        │  └─────────────────────┘│
│         │         └──────────────────────────────┘                         │
│         │                                                                   │
│         ▼                                                                   │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                      EKF Localization                                 │  │
│  │  • Marker Update (absolute position)                                  │  │
│  │  • IMU Predict (angular velocity)                                     │  │
│  │  • Odom Update (wheel velocity)                                       │  │
│  │  ──────────────────────────────────────────────────────────────────   │  │
│  │  Publishes: /localization/pose, /localization/odom, TF               │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## ❗ 주의사항

1. **Arduino 리셋 방지**: DTR/RTS 비활성화 필수
2. **TF 충돌**: EKF와 기존 localization 동시 사용 금지
3. **엔코더 방향**: 휠 회전 방향과 엔코더 카운트 방향 확인
4. **카메라 오프셋**: `cam_offset_x` 정확히 측정 필요

---

## 📝 TODO

- [ ] 엔코더 핀 매핑 실제 하드웨어에 맞게 수정
- [ ] 휠 반경, 기어비 등 물리 파라미터 측정
- [ ] PID 게인 튜닝
- [ ] EKF 노이즈 파라미터 튜닝
- [ ] 주차칸 마커 좌표 입력 (marker_map.yaml ID 16-27)
