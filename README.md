# 🚗 ROS2 Smart Parking RC Car System

자율주행 RC카 기반 스마트 주차 관리 시스템

---

## 📋 목차

1. [시스템 개요](#시스템-개요)
2. [하드웨어 구성](#하드웨어-구성)
3. [소프트웨어 아키텍처](#소프트웨어-아키텍처)
4. [패키지 구조](#패키지-구조)
5. [노드 상세](#노드-상세)
6. [메시지 정의](#메시지-정의)
7. [토픽 목록](#토픽-목록)
8. [설치 방법](#설치-방법)
9. [실행 방법](#실행-방법)
10. [테스트 및 디버깅](#테스트-및-디버깅)
11. [파라미터 튜닝](#파라미터-튜닝)
12. [트러블슈팅](#트러블슈팅)

---

## 시스템 개요

### 프로젝트 설명
- **목적**: ArUco 마커 기반 실내 자율주행 및 자동 주차
- **플랫폼**: 메카넘 휠 RC카 (4WD)
- **환경**: ROS2 Humble, Ubuntu 22.04 (Jetson)

### 주요 기능
- 차선 추종 주행 (Lane Following)
- ArUco 마커 기반 위치 인식 및 네비게이션
- EKF 센서 융합 (Marker + IMU + Wheel Odometry)
- 자동 주차 (측면 주차)
- MQTT 기반 서버 통신
- 수동/자동 모드 전환

### 맵 사양
| 항목 | 값 |
|------|-----|
| 크기 | 120 × 205 cm |
| 원점 | 좌측 하단 (0, 0) |
| 단위 | cm (내부 처리: m) |
| 도로 마커 | ID 0-15, 4cm |
| 주차칸 마커 | ID 16-99, 2cm |

---

## 하드웨어 구성

### 메인 보드
| 구성요소 | 사양 |
|----------|------|
| 컴퓨팅 | Jetson Orin Nano |
| 모터 제어 | Arduino UNO + PCA9685 |
| 통신 | UART (115200 baud) |

### 센서
| 센서 | 용도 | 인터페이스 |
|------|------|------------|
| USB 카메라 × 3 | 전방/하단/측면 | USB (v4l2) |
| MPU6050 | IMU (자이로/가속도) | I2C (bus 7) |
| 엔코더 × 4 | 휠 속도 | Arduino GPIO |

### 액추에이터
| 구성요소 | 사양 |
|----------|------|
| 모터 | DC 기어드 모터 × 4 |
| 드라이버 | PCA9685 (PWM) + L298N |
| 휠 | 메카넘 휠 (Φ60mm) |

### 핀 배치 (Arduino)
```
모터 방향 핀:
  Motor A: D4, D5
  Motor B: D6, D7
  Motor C: D8, D9
  Motor D: D10, D11

엔코더 핀:
  Encoder A: D2 (INT), A0
  Encoder B: D3 (INT), A1
  Encoder C: A2, A3 (Polling)
  Encoder D: A4, A5 (Polling)

PCA9685: I2C (A4=SDA, A5=SCL) - 충돌 시 소프트웨어 I2C 사용
```

---

## 소프트웨어 아키텍처

### 데이터 플로우
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MQTT Server                                     │
│                          (Task Command/Status)                               │
└─────────────────────────────────┬───────────────────────────────────────────┘
                                  │
                    ┌─────────────▼─────────────┐
                    │      rc_mqtt_bridge       │
                    │   (mqtt_bridge_node)      │
                    └─────────────┬─────────────┘
                                  │
        ┌─────────────────────────▼─────────────────────────┐
        │                   rc_mission                       │
        │             (mission_manager_node)                 │
        │         FSM: IDLE→DRIVE→ALIGN→TURN→PARK           │
        └───────────────────────┬───────────────────────────┘
                                │
    ┌───────────────────────────▼───────────────────────────────┐
    │                                                            │
    │  ┌──────────────────┐    ┌──────────────────────────────┐ │
    │  │   Perception     │    │         Control              │ │
    │  │                  │    │                              │ │
    │  │ • lane_node_v2   │    │ • control_stack_node         │ │
    │  │ • marker_pose    │───▶│   (Stanley, Align, Park)     │ │
    │  │ • parking_line   │    │                              │ │
    │  │ • slot_marker    │    │ • safety_manager_node        │ │
    │  │ • imu_mpu6050    │    │   (MUX, Watchdog, IMU Hold)  │ │
    │  └────────┬─────────┘    └──────────────┬───────────────┘ │
    │           │                             │                  │
    │           ▼                             ▼                  │
    │  ┌──────────────────┐    ┌──────────────────────────────┐ │
    │  │  Localization    │    │      arduino_bridge          │ │
    │  │                  │    │                              │ │
    │  │ • ekf_node       │    │ • UART TX/RX                 │ │
    │  │   (Marker+IMU    │    │ • Odometry Publish           │ │
    │  │    +Odom Fusion) │    │ • DriveCmd → PWM             │ │
    │  └──────────────────┘    └──────────────┬───────────────┘ │
    │                                         │                  │
    └─────────────────────────────────────────┼──────────────────┘
                                              │
                              ┌───────────────▼───────────────┐
                              │         Arduino UNO           │
                              │                               │
                              │ • 4-Wheel PID Control         │
                              │ • Encoder Reading             │
                              │ • Odometry Calculation        │
                              │ • Deadband Compensation       │
                              └───────────────────────────────┘
```

### 레이어 구조
```
┌─────────────────────────────────────────────────────────────┐
│ Layer 4: Application    │ MQTT Bridge, Mission Manager      │
├─────────────────────────────────────────────────────────────┤
│ Layer 3: Planning       │ EKF Localization, Path Planning   │
├─────────────────────────────────────────────────────────────┤
│ Layer 2: Control        │ Control Stack, Safety Manager     │
├─────────────────────────────────────────────────────────────┤
│ Layer 1: Perception     │ Lane, Marker, IMU, Parking Line   │
├─────────────────────────────────────────────────────────────┤
│ Layer 0: Driver         │ Arduino Bridge, Camera Driver     │
└─────────────────────────────────────────────────────────────┘
```

---

## 패키지 구조

```
ros2_ws/src/
│
├── Perception/
│   ├── rc_perception/              # 영상 인식 통합 패키지
│   │   ├── rc_perception/
│   │   │   ├── lane_node_v2.py     # 차선 인식 (Adaptive + Kalman)
│   │   │   ├── marker_pose_node.py # ArUco 마커 인식
│   │   │   ├── parking_line_node.py# 주차선 인식
│   │   │   └── slot_marker_node.py # 주차칸 마커 인식
│   │   ├── config/
│   │   │   └── lane_v2.yaml
│   │   └── launch/
│   │       └── perception.launch.py
│   │
│   └── rc_imu_mpu6050/             # IMU 드라이버
│       ├── rc_imu_mpu6050/
│       │   └── imu_mpu6050_node.py # MPU6050 I2C 드라이버
│       └── ...
│
├── Planning/
│   ├── rc_localization/            # 위치 추정
│   │   ├── rc_localization/
│   │   │   └── ekf_localization_node.py  # EKF 센서 융합
│   │   ├── config/
│   │   │   └── ekf_localization.yaml
│   │   └── launch/
│   │       └── localization.launch.py
│   │
│   └── rc_mission/                 # 미션 관리
│       ├── rc_mission/
│       │   └── mission_manager_node.py   # FSM 미션 매니저
│       └── ...
│
├── Control/
│   ├── rc_control_stack/           # 제어 스택
│   │   ├── rc_control_stack/
│   │   │   ├── control_stack_node.py     # 자율주행 컨트롤러
│   │   │   └── safety_manager_node.py    # 안전 관리자
│   │   ├── config/
│   │   │   └── control_stack.yaml
│   │   └── launch/
│   │       └── control.launch.py
│   │
│   └── arduino_bridge/             # Arduino 통신
│       ├── arduino_bridge/
│       │   └── arduino_bridge_node.py    # UART 브릿지
│       ├── config/
│       │   └── arduino_bridge.yaml
│       └── launch/
│           └── arduino_bridge.launch.py
│
├── Utils/
│   ├── rc_interfaces/              # 메시지 정의
│   │   ├── msg/
│   │   │   ├── Line/
│   │   │   │   ├── LaneStatus.msg
│   │   │   │   └── ParkingLineStatus.msg
│   │   │   ├── Marker/
│   │   │   │   └── MarkerStatus.msg
│   │   │   └── State/
│   │   │       ├── DriveCmd.msg
│   │   │       ├── DrivingState.msg
│   │   │       ├── TaskCmd.msg
│   │   │       └── TaskStatus.msg
│   │   └── CMakeLists.txt
│   │
│   ├── rc_bringup/                 # 런치 파일 모음
│   │   ├── config/
│   │   │   └── mission.yaml
│   │   └── launch/
│   │       ├── system.launch.py    # 전체 시스템
│   │       └── sensors.launch.py   # 센서만
│   │
│   └── rc_mqtt_bridge/             # MQTT 통신
│       ├── rc_mqtt_bridge/
│       │   └── mqtt_bridge_node.py
│       ├── config/
│       │   └── mqtt_bridge.yaml
│       └── launch/
│           └── mqtt_bridge.launch.py
│
└── Arduino/
    └── Arduino_motor_pid.ino       # Arduino 펌웨어
```

---

## 노드 상세

### Perception 노드

#### lane_node_v2
차선 인식 및 추적

| 항목 | 내용 |
|------|------|
| 패키지 | rc_perception |
| 실행파일 | lane_node_v2 |
| 입력 | /cam_bottom/image_raw (Image) |
| 출력 | /perception/lane (LaneStatus) |

**알고리즘:**
1. ROI 추출 (상단 30% 제외)
2. Grayscale 변환
3. Adaptive Threshold (조명 변화 대응)
4. Morphology (끊어진 라인 연결)
5. Sliding Window 추적
6. Kalman Filter 스무딩

**주요 파라미터:**
```yaml
line_color: black          # 라인 색상 (black/white)
use_adaptive: true         # Adaptive threshold 사용
adaptive_block_size: 25    # 블록 크기 (홀수)
adaptive_c: 10             # 상수 C
sw_n_windows: 9            # 슬라이딩 윈도우 개수
kalman_q: 0.005            # Process noise
kalman_r: 0.05             # Measurement noise
```

---

#### marker_pose_node
ArUco 마커 인식 및 포즈 추정

| 항목 | 내용 |
|------|------|
| 패키지 | rc_perception |
| 실행파일 | marker_pose_node |
| 입력 | /cam_front/image_raw (Image) |
| 출력 | /perception/marker_status (MarkerStatus) |

**알고리즘:**
1. ArUco 마커 검출 (DICT_4X4_50)
2. solvePnP 포즈 추정
3. EMA 스무딩
4. 품질 점수 계산 (면적 기반)

**마커 크기:**
- ID 0-15: 4cm (도로)
- ID 16-99: 2cm (주차칸)

---

#### imu_mpu6050_node
MPU6050 IMU 드라이버

| 항목 | 내용 |
|------|------|
| 패키지 | rc_imu_mpu6050 |
| 실행파일 | imu_mpu6050_node |
| 출력 | /imu/data (Imu) |

**기능:**
- 시작 시 bias 캘리브레이션 (200 샘플)
- Complementary filter (roll/pitch)
- Gyro 적분 (yaw)

---

### Planning 노드

#### ekf_localization_node
EKF 기반 위치 추정

| 항목 | 내용 |
|------|------|
| 패키지 | rc_localization |
| 실행파일 | ekf_localization_node |
| 입력 | marker_status, /imu/data, /odom/wheel |
| 출력 | /localization/pose (PoseStamped) |

**상태 벡터:** [x, y, θ, vx, vy, ω]

**센서 융합:**
| 센서 | 역할 | 주기 |
|------|------|------|
| Marker | 절대 위치 업데이트 | 검출 시 |
| IMU | 각속도 예측 | 50Hz |
| Wheel Odom | 속도 예측/업데이트 | 50Hz |

---

#### mission_manager_node
FSM 기반 미션 관리

| 항목 | 내용 |
|------|------|
| 패키지 | rc_mission |
| 실행파일 | mission_manager_node |
| 입력 | /server/task_cmd, marker_status |
| 출력 | /mission/state, /control/enable_drive |

**상태 전이:**
```
IDLE
  │ task_cmd 수신
  ▼
DRIVE ←──────────────────┐
  │ 마커 도착             │
  ▼                      │
STOP_AT_MARKER           │
  │                      │
  ▼                      │
ADVANCE_TO_CENTER        │
  │ 거리 < 15cm          │
  ▼                      │
ALIGN_TO_MARKER          │
  │ align_done           │
  ▼                      │
STOP_BUMP                │
  │ 0.12s 대기           │
  ▼                      │
TURNING ─────────────────┘
  │ 턴 완료
  ▼
(다음 마커 없으면)
  │
  ▼
PARK ──────▶ FINISH
```

---

### Control 노드

#### control_stack_node
자율주행 컨트롤러

| 항목 | 내용 |
|------|------|
| 패키지 | rc_control_stack |
| 실행파일 | control_stack_node |
| 입력 | /mission/state, /perception/* |
| 출력 | /control/drive_cmd, /driving/state |

**제어 모드:**

| 모드 | 알고리즘 | 설명 |
|------|----------|------|
| LANE_FOLLOW | Stanley Controller | 차선 추종 |
| TURNING | P Control | 제자리 회전 |
| ADVANCE_TO_CENTER | P Control | 마커 접근 |
| ALIGN_TO_MARKER | PID (x, y, yaw) | 정밀 정렬 |
| PARK | Multi-stage | 주차 시퀀스 |

**Stanley Controller:**
```
δ = ψ + arctan(k_y * e_y / (v + v0))

ψ: heading error
e_y: crosstrack error  
k_y: crosstrack gain
v0: softening constant
```

---

#### safety_manager_node
안전 관리 및 명령 MUX

| 항목 | 내용 |
|------|------|
| 패키지 | rc_control_stack |
| 실행파일 | safety_manager_node |
| 입력 | /control/drive_cmd, /control/drive_cmd_emergency |
| 출력 | /control/drive_cmd_safe |

**기능:**
1. **Emergency MUX**: 긴급 명령 우선
2. **Watchdog**: 200ms timeout → 정지
3. **Slew Rate Limiting**: 급가속 방지
4. **IMU Heading Hold**: 직진 시 yaw 드리프트 보정

---

#### arduino_bridge_node
Arduino UART 통신

| 항목 | 내용 |
|------|------|
| 패키지 | arduino_bridge |
| 실행파일 | arduino_bridge_node |
| 입력 | /control/drive_cmd_safe |
| 출력 | /odom/wheel (Odometry) |

**프로토콜:**
| 명령 | 형식 | 설명 |
|------|------|------|
| D | `D vx vy wz` | 정규화 속도 (-1~+1) |
| V | `V vx vy wz` | 실제 속도 (m/s, rad/s) |
| O | `O` | Odometry 요청 |
| K | `K kp ki kd` | PID 게인 설정 |
| S | `S` | 즉시 정지 |

---

### Utils 노드

#### mqtt_bridge_node
MQTT ↔ ROS2 브릿지

| 항목 | 내용 |
|------|------|
| 패키지 | rc_mqtt_bridge |
| 실행파일 | mqtt_bridge_node |

**MQTT 토픽:**
| 방향 | 토픽 | QoS |
|------|------|-----|
| SUB | rc/{car_id}/task/cmd | 1 |
| SUB | rc/{car_id}/control/emergency | 2 |
| PUB | rc/{car_id}/task/status | 1 |
| PUB | rc/{car_id}/location | 0 |
| PUB | rc/{car_id}/status | 0 |

---

## 메시지 정의

### LaneStatus.msg
```
std_msgs/Header header
bool valid              # 차선 검출 여부
float32 offset_norm     # 정규화 오프셋 (-1 ~ +1, 왼쪽 양수)
float32 angle           # 차선 각도 (rad)
float32 quality         # 검출 품질 (0 ~ 1)
bool in_lane            # 차선 내 여부
```

### MarkerStatus.msg
```
std_msgs/Header header
bool valid              # 마커 검출 여부
int32 id                # 마커 ID
float32 rel_x           # 카메라 기준 X (오른쪽 양수, m)
float32 rel_y           # 카메라 기준 Y (아래 양수, m)
float32 rel_z           # 카메라 기준 Z (전방 양수, m)
float32 rel_yaw         # 마커 yaw (rad)
float32 quality         # 검출 품질 (0 ~ 1)
```

### DriveCmd.msg
```
std_msgs/Header header
bool enable             # 주행 활성화
float32 vx              # 전진 속도 (m/s)
float32 vy              # 횡방향 속도 (m/s)
float32 wz              # 회전 속도 (rad/s)
string source           # 명령 출처
```

### DrivingState.msg
```
std_msgs/Header header
string state            # 현재 모드
int32 target_marker_id  # 목표 마커 ID
int32 observed_marker_id# 관측 마커 ID
float32 ex              # X 에러
float32 ey              # Y 에러
float32 eyaw            # Yaw 에러
float32 confidence      # 신뢰도
string detail           # 상세 정보
```

### TaskCmd.msg
```
std_msgs/Header header
int32[] route_ids       # 경유 마커 ID 리스트
int32 goal_id           # 목표 마커 ID
string goal             # 목표 (문자열)
string task_type        # 태스크 유형
string task_id          # 태스크 ID
```

### TaskStatus.msg
```
std_msgs/Header header
string task_id          # 태스크 ID
string status           # 상태 (RUNNING/COMPLETED/FAILED)
string current_state    # 현재 FSM 상태
int32 current_marker_id # 현재 위치 마커 ID
float32 progress        # 진행률 (0 ~ 1)
string message          # 상태 메시지
```

---

## 토픽 목록

### Perception 토픽
| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| /cam_front/image_raw | Image | v4l2_camera | 전방 카메라 |
| /cam_bottom/image_raw | Image | v4l2_camera | 하단 카메라 |
| /cam_side/image_raw | Image | v4l2_camera | 측면 카메라 |
| /perception/lane | LaneStatus | lane_node | 차선 상태 |
| /perception/marker_status | MarkerStatus | marker_pose_node | 마커 상태 |
| /perception/parking_line | ParkingLineStatus | parking_line_node | 주차선 |
| /perception/slot_marker_pose | PoseStamped | slot_marker_node | 슬롯 마커 |
| /imu/data | Imu | imu_mpu6050_node | IMU 데이터 |

### Localization 토픽
| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| /localization/pose | PoseStamped | ekf_node | 추정 위치 |
| /localization/odom | Odometry | ekf_node | 추정 오도메트리 |
| /odom/wheel | Odometry | arduino_bridge | 휠 오도메트리 |

### Control 토픽
| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| /control/drive_cmd | DriveCmd | control_stack | AUTO 명령 |
| /control/drive_cmd_emergency | DriveCmd | mqtt_bridge/teleop | 긴급/수동 명령 |
| /control/drive_cmd_safe | DriveCmd | safety_manager | 최종 명령 |
| /control/enable_drive | Bool | mission_manager | 주행 활성화 |
| /driving/state | DrivingState | control_stack | 주행 상태 |

### Mission 토픽
| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| /mission/state | String | mission_manager | 미션 상태 |
| /mission/turn_target_rad | Float32 | mission_manager | 턴 목표 각도 |
| /mission/align_done | Bool | control_stack | 정렬 완료 |

### Server 토픽
| 토픽 | 타입 | 발행자 | 설명 |
|------|------|--------|------|
| /server/task_cmd | TaskCmd | mqtt_bridge | 태스크 명령 |
| /server/task_status | TaskStatus | mission_manager | 태스크 상태 |

---

## 설치 방법

### 1. 의존성 설치

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install -y \
    ros-humble-v4l2-camera \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-image-transport \
    python3-pip

# Python 패키지
pip3 install \
    pyserial \
    paho-mqtt \
    smbus2 \
    numpy \
    opencv-python \
    pyyaml
```

### 2. 워크스페이스 설정

```bash
# 워크스페이스 생성
mkdir -p ~/rc_ws/src
cd ~/rc_ws/src

# 패키지 복사 (압축 해제 후)
cp -r smart_parking_final/ros2_ws/src/* .

# 빌드
cd ~/rc_ws
colcon build --symlink-install

# 환경 설정
echo "source ~/rc_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Arduino 펌웨어 업로드

```bash
# Arduino IDE 또는 arduino-cli 사용
arduino-cli compile --fqbn arduino:avr:uno Arduino_motor_pid.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno Arduino_motor_pid.ino
```

### 4. 권한 설정

```bash
# 시리얼 포트 권한
sudo usermod -aG dialout $USER

# I2C 권한 (IMU)
sudo usermod -aG i2c $USER

# 카메라 권한
sudo usermod -aG video $USER

# 재로그인 필요
```

### 5. udev 규칙 (선택)

```bash
# /etc/udev/rules.d/99-rc-car.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino", MODE="0666"
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="xxxx", ATTR{index}=="0", SYMLINK+="cam_front", MODE="0666"

sudo udevadm control --reload-rules
```

---

## 실행 방법

### 전체 시스템 실행

```bash
# 기본 실행
ros2 launch rc_bringup system.launch.py

# 전체 옵션
ros2 launch rc_bringup system.launch.py \
    marker_map_yaml:=/path/to/marker_map.yaml \
    serial_port:=/dev/ttyUSB0 \
    use_mqtt:=true \
    car_id:=car_01 \
    mqtt_host:=192.168.1.100 \
    show_debug:=false
```

### 개별 모듈 실행

```bash
# 1. 센서만 실행
ros2 launch rc_bringup sensors.launch.py \
    cam_front_dev:=/dev/video0 \
    cam_bottom_dev:=/dev/video2 \
    cam_side_dev:=/dev/video4 \
    i2c_bus:=7

# 2. Perception만 실행
ros2 launch rc_perception perception.launch.py show_debug:=true

# 3. Localization만 실행
ros2 launch rc_localization localization.launch.py \
    marker_map_yaml:=/path/to/marker_map.yaml

# 4. Control만 실행
ros2 launch rc_control_stack control.launch.py \
    serial_port:=/dev/ttyUSB0

# 5. Arduino Bridge만 실행
ros2 launch arduino_bridge arduino_bridge.launch.py \
    port:=/dev/ttyUSB0 \
    baud:=115200

# 6. MQTT만 실행
ros2 launch rc_mqtt_bridge mqtt_bridge.launch.py \
    car_id:=car_01 \
    mqtt_host:=localhost
```

### 수동 조작 (Teleop)

```bash
# 키보드 조작 (rc_human_interface 패키지)
ros2 run rc_human_interface keyboard_teleop_node

# 조작 키:
#   W/S: 전진/후진
#   A/D: 좌/우 이동
#   Q/E: 좌/우 회전
#   Space: 정지
#   Esc: 종료
```

### 단일 노드 실행 (디버깅용)

```bash
# 노드 단독 실행
ros2 run rc_perception lane_node_v2 --ros-args \
    -p show_debug:=true \
    -p image_topic:=/cam_bottom/image_raw

ros2 run rc_perception marker_pose_node --ros-args \
    -p show_debug:=true

ros2 run rc_imu_mpu6050 imu_mpu6050_node --ros-args \
    -p i2c_bus:=7

ros2 run rc_localization ekf_localization_node --ros-args \
    -p marker_map_yaml:=/path/to/marker_map.yaml

ros2 run rc_control_stack control_stack_node

ros2 run arduino_bridge arduino_bridge_node --ros-args \
    -p port:=/dev/ttyUSB0
```

---

## 테스트 및 디버깅

### 1단계: 하드웨어 연결 확인

```bash
# Arduino 연결 확인
ls -la /dev/ttyUSB*
# 출력: /dev/ttyUSB0

# 카메라 확인
ls -la /dev/video*
v4l2-ctl --list-devices

# I2C 확인 (IMU)
sudo i2cdetect -y 7
# 0x68에 MPU6050 표시되어야 함
```

### 2단계: Arduino 통신 테스트

```bash
# 시리얼 모니터
screen /dev/ttyUSB0 115200

# Arduino 응답 확인
# "READY" 메시지 출력되어야 함

# 명령 테스트
D 0.5 0 0    # 전진
D 0 0.5 0    # 좌측 이동
D 0 0 0.5    # 좌회전
S            # 정지
O            # Odometry 요청
```

### 3단계: ROS2 토픽 확인

```bash
# 토픽 리스트
ros2 topic list

# 토픽 모니터링
ros2 topic echo /perception/lane
ros2 topic echo /perception/marker_status
ros2 topic echo /imu/data
ros2 topic echo /odom/wheel
ros2 topic echo /localization/pose
ros2 topic echo /control/drive_cmd_safe
ros2 topic echo /driving/state
ros2 topic echo /mission/state

# 토픽 주파수 확인
ros2 topic hz /perception/lane
ros2 topic hz /imu/data
ros2 topic hz /odom/wheel
```

### 4단계: 개별 노드 테스트

#### 4-1. 카메라 테스트
```bash
# 카메라 노드 실행
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0

# 이미지 확인
ros2 run rqt_image_view rqt_image_view
```

#### 4-2. 차선 인식 테스트
```bash
# 디버그 모드로 실행
ros2 run rc_perception lane_node_v2 --ros-args \
    -p show_debug:=true \
    -p publish_debug_image:=true

# 결과 확인
ros2 topic echo /perception/lane
# offset_norm: 0에 가까우면 중앙
# quality: 0.5 이상이면 양호
```

#### 4-3. 마커 인식 테스트
```bash
ros2 run rc_perception marker_pose_node --ros-args \
    -p show_debug:=true

# 결과 확인
ros2 topic echo /perception/marker_status
# valid: true
# rel_z: 마커까지 거리 (m)
```

#### 4-4. IMU 테스트
```bash
ros2 run rc_imu_mpu6050 imu_mpu6050_node

# 결과 확인
ros2 topic echo /imu/data
# orientation: quaternion (z가 yaw)
# angular_velocity: 회전 시 z 값 변화
```

#### 4-5. 모터 제어 테스트
```bash
# Arduino Bridge 실행
ros2 run arduino_bridge arduino_bridge_node

# 명령 발행 테스트
ros2 topic pub /control/drive_cmd_safe rc_interfaces/msg/DriveCmd \
    "{enable: true, vx: 0.1, vy: 0.0, wz: 0.0}"

# 정지
ros2 topic pub /control/drive_cmd_safe rc_interfaces/msg/DriveCmd \
    "{enable: false, vx: 0.0, vy: 0.0, wz: 0.0}"
```

### 5단계: 통합 테스트

#### 5-1. 차선 추종 테스트
```bash
# 시스템 실행 (MQTT 없이)
ros2 launch rc_bringup system.launch.py use_mqtt:=false

# 다른 터미널에서 주행 활성화
ros2 topic pub /control/enable_drive std_msgs/msg/Bool "{data: true}"

# 미션 상태를 DRIVE로 설정
ros2 topic pub /mission/state std_msgs/msg/String "{data: 'DRIVE'}"
```

#### 5-2. 마커 정렬 테스트
```bash
# 미션 상태를 ALIGN_TO_MARKER로 설정
ros2 topic pub /mission/state std_msgs/msg/String "{data: 'ALIGN_TO_MARKER'}"

# 정렬 상태 모니터링
ros2 topic echo /driving/state
```

#### 5-3. 전체 미션 테스트
```bash
# 태스크 명령 발행
ros2 topic pub /server/task_cmd rc_interfaces/msg/TaskCmd \
    "{task_id: 'test_01', route_ids: [0, 1, 2], goal_id: 20}"

# 미션 상태 모니터링
ros2 topic echo /mission/state
ros2 topic echo /driving/state
```

### 6단계: 시각화 도구

```bash
# RViz2
ros2 run rviz2 rviz2

# 추가할 Display:
# - TF
# - Odometry (/localization/odom)
# - PoseStamped (/localization/pose)
# - Image (/cam_front/image_raw)

# rqt 그래프
ros2 run rqt_graph rqt_graph

# rqt 플롯 (실시간 데이터)
ros2 run rqt_plot rqt_plot /perception/lane/offset_norm
```

### 로그 확인

```bash
# 전체 로그
ros2 topic echo /rosout

# 특정 노드 로그 레벨 변경
ros2 service call /lane_node/set_logger_level \
    rcl_interfaces/srv/SetLoggerLevel \
    "{logger_name: 'lane_node', level: 10}"  # DEBUG=10
```

---

## 파라미터 튜닝

### 차선 추종 파라미터

```yaml
# config/control_stack.yaml
lane_vx: 0.15        # 기본 속도 (↑ 빠름, ↓ 느림)
lane_k_psi: 0.8      # heading 게인 (↑ 민감)
lane_k_y: 1.2        # crosstrack 게인 (↑ 강한 복귀)
lane_v0: 0.1         # softening (↑ 부드러움)
```

### 마커 정렬 파라미터

```yaml
align_kp_x: 0.8      # X 방향 게인
align_kp_y: 0.6      # Y 방향 게인
align_kp_yaw: 0.5    # Yaw 게인
align_tolerance_xy: 0.02   # 위치 허용 오차 (m)
align_tolerance_yaw: 0.08  # 각도 허용 오차 (rad)
```

### EKF 노이즈 파라미터

```yaml
# Process noise (Q) - 모델 신뢰도
# 값이 작을수록 모델 신뢰, 클수록 센서 신뢰
q_x: 0.01
q_y: 0.01
q_theta: 0.005

# Measurement noise (R) - 센서 노이즈
r_marker_x: 0.02
r_marker_y: 0.02
r_marker_theta: 0.05
```

### Arduino PID 파라미터

```cpp
// Arduino_motor_pid.ino
#define DEFAULT_KP  80.0f   // 비례 (↑ 빠른 응답, 진동 위험)
#define DEFAULT_KI  40.0f   // 적분 (↑ 정상상태 오차 제거)
#define DEFAULT_KD  0.5f    // 미분 (↑ 오버슈트 감소)
```

**PID 튜닝 방법:**
1. KI, KD를 0으로 설정
2. KP를 진동 직전까지 증가
3. KP를 절반으로 줄임
4. KI를 천천히 증가 (정상상태 오차 제거)
5. 오버슈트가 있으면 KD 추가

---

## 트러블슈팅

### Arduino 연결 문제

**증상:** `Failed to open serial port`

```bash
# 포트 확인
ls -la /dev/ttyUSB*

# 권한 확인
sudo chmod 666 /dev/ttyUSB0

# 또는 사용자를 dialout 그룹에 추가
sudo usermod -aG dialout $USER
# 재로그인 필요
```

**증상:** `No response from Arduino`

```bash
# DTR/RTS 리셋 비활성화 확인
stty -F /dev/ttyUSB0 -hupcl

# Arduino 재업로드
# 또는 USB 케이블 교체
```

### 카메라 문제

**증상:** `Failed to open device`

```bash
# 카메라 목록 확인
v4l2-ctl --list-devices

# 포맷 확인
v4l2-ctl -d /dev/video0 --list-formats-ext

# 다른 프로세스가 사용 중인지 확인
fuser /dev/video0
```

### IMU 문제

**증상:** `I2C read error`

```bash
# I2C 장치 확인
sudo i2cdetect -y 7

# 버스 번호 확인 (Jetson은 보통 7)
ls /dev/i2c-*

# 권한 확인
sudo chmod 666 /dev/i2c-7
```

### 마커 인식 문제

**증상:** 마커가 검출되지 않음

1. 조명 확인 (너무 어둡거나 반사)
2. 마커 크기 파라미터 확인
3. 카메라 캘리브레이션 확인
4. 카메라 초점 확인

```bash
# 디버그 모드로 확인
ros2 run rc_perception marker_pose_node --ros-args -p show_debug:=true
```

### 차선 인식 문제

**증상:** offset이 불안정

1. `kalman_q` 감소 (더 스무딩)
2. `adaptive_block_size` 조정
3. ROI 영역 조정
4. 조명 조건 확인

```bash
# 디버그 이미지 확인
ros2 run rc_perception lane_node_v2 --ros-args \
    -p show_debug:=true \
    -p publish_debug_image:=true
```

### 모터 동작 문제

**증상:** 모터가 움직이지 않음

1. enable 플래그 확인
2. Watchdog timeout 확인
3. Arduino Serial Monitor로 명령 확인
4. PWM 값 확인 (deadband 이상인지)

```bash
# 직접 명령 테스트
ros2 topic pub /control/drive_cmd_safe rc_interfaces/msg/DriveCmd \
    "{enable: true, vx: 0.2, vy: 0.0, wz: 0.0}"
```

### EKF 드리프트 문제

**증상:** 위치가 점점 틀어짐

1. 마커 인식 빈도 확인
2. IMU bias 확인 (시작 시 정지 상태)
3. 휠 오도메트리 캘리브레이션
4. Q/R 노이즈 파라미터 조정

```bash
# 마커 업데이트 확인
ros2 topic echo /localization/fix_valid
```

---

## 참고 자료

### 좌표계
- **맵 좌표계**: 원점 좌측 하단, X 오른쪽, Y 위쪽
- **로봇 좌표계**: X 전방, Y 좌측, Z 위쪽 (REP-103)
- **카메라 좌표계**: X 오른쪽, Y 아래, Z 전방 (OpenCV)

### TF 트리
```
map
 └── odom
      └── base_link
           ├── camera_front
           ├── camera_bottom
           ├── camera_side
           └── imu_link
```

### marker_map.yaml 형식
```yaml
markers:
  - id: 0
    x: 30.0    # cm
    y: 50.0    # cm
    yaw: 1.57  # rad (마커가 바라보는 방향)
  - id: 1
    x: 60.0
    y: 50.0
    yaw: 1.57
  # ... 주차칸 마커 (ID 16-27)
  - id: 16
    x: 100.0
    y: 20.0
    yaw: 0.0
```

---

## 라이선스

Apache-2.0

## 작성자

SSAFY A102 Team