# API Reference

## Messages (robot_interfaces/msg)

### Marker.msg
```
std_msgs/Header header
int32 id                    # 마커 ID
geometry_msgs/Pose pose     # 3D 포즈
float32 distance            # 거리 (m)
float32 angle               # 수평 각도 (rad)
float32 confidence          # 신뢰도 (0-1)
```

### TrackedMarker.msg
```
std_msgs/Header header
int32 id
geometry_msgs/Pose pose            # 현재 포즈
geometry_msgs/Pose predicted_pose  # 예측 포즈 (0.5s)
geometry_msgs/Vector3 velocity     # 속도 (m/s)
float32 distance
float32 angle
bool is_detected                   # 현재 검출 여부
float32 prediction_confidence      # 예측 신뢰도
float32 time_since_detection       # 마지막 검출 후 시간
```

### LaneStatus.msg
```
std_msgs/Header header
bool valid                   # 차선 검출 여부
float32 offset               # 중심 오프셋 (m)
float32 offset_normalized    # 정규화 오프셋 (-1~+1)
float32 angle                # 차선 각도 (rad)
float32 curvature            # 곡률
float32 confidence
```

### ParkingLineStatus.msg (측면 카메라)
```
std_msgs/Header header
bool valid                   # 주차선 검출 여부
float32 offset_norm          # 정규화 오프셋 (-1~+1)
float32 angle                # 주차선 각도 (rad)
float32 quality              # 검출 품질 (0~1)
```

### MotorCommand.msg
```
std_msgs/Header header
int16 front_left     # PWM (-3000 ~ 3000)
int16 front_right
int16 rear_left
int16 rear_right
```

### DrivingState.msg
```
std_msgs/Header header
string state                 # IDLE, LANE_FOLLOW, MARKER_APPROACH, etc.
int32 target_marker_id
int32 observed_marker_id
float32 error_x              # X 에러 (m)
float32 error_y              # Y 에러 (m)
float32 error_yaw            # Yaw 에러 (rad)
float32 cmd_vx, cmd_vy, cmd_wz
string detail
```

### MissionCommand.msg
```
std_msgs/Header header
string command               # START, STOP, PAUSE
int32[] waypoint_ids         # 경유 마커 ID
int32 final_goal_id          # 최종 목표
string task_type             # PICKUP, DROPOFF, RETURN
string task_id
```

### MissionStatus.msg
```
std_msgs/Header header
string task_id
string status                # RUNNING, COMPLETED, FAILED
string current_state         # FSM 상태
int32 current_waypoint_idx
int32 current_marker_id
float32 progress             # 0-1
string message
```

## Services

### SetTargetMarker.srv
```
int32 marker_id
---
bool success
string message
```

### GetRobotState.srv
```
---
string state
int32 current_marker_id
float32 x, y, yaw
float32 battery_voltage
```

## Actions

### NavigateToMarker.action
```
# Goal
int32 target_marker_id
float32 timeout_sec
---
# Result
bool success
string message
float32 total_time
---
# Feedback
string current_state
int32 current_marker_id
float32 distance_remaining
float32 elapsed_time
```

## Node Parameters

### marker_detector
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| dictionary | DICT_4X4_50 | ArUco 사전 |
| marker_size | 0.10 | 마커 크기 (m) |
| publish_debug_image | true | 디버그 이미지 |

### lane_detector
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| line_color | black | 차선 색상 |
| roi_top_ratio | 0.5 | ROI 상단 비율 |
| sobel_threshold | 50 | Sobel 임계값 |
| kalman_q | 0.01 | 프로세스 노이즈 |
| kalman_r | 0.05 | 측정 노이즈 |

### motion_controller
차선 추종에 시각 서보 PID 사용 (엔코더 기반 속도 PID 아님)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| max_vx | 0.05 | 최대 전진 속도 (m/s) |
| max_vy | 0.05 | 최대 횡방향 속도 (m/s) |
| max_wz | 0.5 | 최대 회전 속도 (rad/s) |
| lane_vx | 0.03 | 차선 추종 속도 (m/s) |
| lane_vy_kp | 0.03 | 차선 오프셋 P 게인 (시각 서보) |
| lane_wz_kp | 0.3 | 차선 각도 P 게인 (시각 서보) |
| marker_reach_distance | 0.15 | 마커 도달 거리 (m) |
| marker_vx_kp | 0.1 | 마커 접근 속도 게인 |
| marker_vy_kp | 0.05 | 마커 횡방향 보정 게인 |
| marker_wz_kp | 0.3 | 마커 방향 보정 게인 |

### arduino_driver (arduino_node)
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| port | /dev/ttyUSB0 | Arduino 시리얼 포트 |
| baudrate | 115200 | 통신 속도 |
| timeout | 0.1 | 시리얼 타임아웃 (초) |
| simulate | false | 시뮬레이션 모드 |
| cmd_rate_hz | 20.0 | 명령 전송 주기 |
| watchdog_timeout | 0.5 | Watchdog 타임아웃 (초) |
| max_vx | 0.10 | 최대 전진 속도 (m/s) |
| max_vy | 0.10 | 최대 횡방향 속도 (m/s) |
| max_wz | 1.0 | 최대 회전 속도 (rad/s) |

### 참고: 메카넘 기구학 파라미터 (Arduino 펌웨어 내장)
| 파라미터 | 값 | 설명 |
|---------|-----|------|
| WHEEL_RADIUS | 0.040 | 휠 반경 (m) |
| WHEEL_BASE_X | 0.075 | 중심~앞/뒤 바퀴 거리 (m) |
| WHEEL_BASE_Y | 0.090 | 중심~좌/우 바퀴 거리 (m) |
| MAX_PWM | 2000 | 최대 PWM (조정 가능) |
| MIN_PWM | 300 | 최소 PWM (데드존) |

## Arduino Protocol (115200 baud)

Arduino UNO + MoebiusTech Motor Hat 기반 Open-loop 제어

### TX (Jetson → Arduino)
```
V vx vy wz\n       # 속도 명령 (m/s, rad/s) - 역기구학 Arduino에서 처리
D nx ny nw\n       # 정규화 속도 (-1 ~ +1)
S\n                # 긴급 정지
P max_pwm\n        # 최대 PWM 설정 (300~4000)
?\n                # 상태 조회
```

### RX (Arduino → Jetson)
```
OK\n               # 명령 확인
READY\n            # 부팅 완료
STOPPED\n          # 정지됨
ERR\n              # 에러
ROS2_BRIDGE v1.0   # 상태 응답
```

### 주의사항
- **엔코더 미사용**: Motor Hat이 엔코더 신호를 통과시키지 않음
- **IMU 별도**: MPU6050은 Jetson I2C에 직접 연결 (imu_mpu6050_node)
- **Watchdog**: 300ms 내 명령 없으면 자동 정지
