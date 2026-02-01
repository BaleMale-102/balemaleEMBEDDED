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
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| max_vx | 0.05 | 최대 전진 속도 |
| max_vy | 0.05 | 최대 횡방향 속도 |
| max_wz | 0.5 | 최대 회전 속도 |
| lane_vx | 0.03 | 차선 추종 속도 |
| marker_reach_distance | 0.15 | 마커 도달 거리 |

### wheel_controller
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| wheel_radius | 0.03 | 휠 반경 (m) |
| wheel_base_x | 0.08 | X축 휠 거리 |
| wheel_base_y | 0.06 | Y축 휠 거리 |
| max_pwm | 3000 | 최대 PWM |
| serial_port | /dev/ttyUSB0 | Arduino 포트 |
| watchdog_timeout | 0.3 | 명령 타임아웃 |

## Arduino Protocol

### TX (Jetson → Arduino)
```
M,FL,FR,RL,RR\n    # 모터 명령 (PWM 값)
```

### RX (Arduino → Jetson)
```
I,ax,ay,az,gx,gy,gz,yaw\n    # IMU 데이터
E,fl,fr,rl,rr\n              # 엔코더 데이터
S,status\n                    # 상태 메시지
```
