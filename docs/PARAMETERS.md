# 파라미터 설정 (robot_params.yaml)

## 센서 드라이버

```yaml
/cam_front:
  device: "/dev/video0"
  width: 640, height: 480, fps: 30.0
  autofocus: false, focus: 40

/cam_side:
  device: "/dev/video2"
  width: 640, height: 480, fps: 30.0

/arduino_driver:
  port: "/dev/ttyUSB0"
  baudrate: 115200
  watchdog_timeout: 0.15
  max_vx: 0.011, max_vy: 0.020, max_wz: 0.187
  wz_offset: 0.002             # 직진 보정 (왼쪽 휘어서 양수)
  max_pwm: 3520                # 7.4V 배터리 +10% 보상

/imu_node:
  i2c_bus: 7
  publish_rate_hz: 100.0
  calib_samples: 500
  comp_alpha: 0.98

/loader_driver:
  port: "/dev/ttyACM0"
  baudrate: 9600
  use_bridge: true
  operation_duration: 15.0
```

## 인식

```yaml
/marker_detector:
  dictionary: "DICT_4X4_50"
  marker_size: 0.04

/marker_tracker:
  process_noise: 0.01
  measurement_noise: 0.05
  prediction_timeout: 2.0
  camera_offset_x: 0.10
```

## 제어

```yaml
/motion_controller:
  control_rate: 20.0
  max_vx: 0.009, max_vy: 0.020, max_wz: 0.099
  reverse_vx: 0.020

  # Marker approach (vx+vy+wz 동시 제어)
  marker_vx_kp: 0.017, marker_vy_kp: 0.022, marker_wz_kp: 0.055
  marker_reach_distance: 0.25
  marker_align_threshold: 0.1    # TURN에서 마커 정렬 완료 기준 (rad)
  advance_vx: 0.024, advance_time: 0.3

  # Turn (PD 제어 + stall boost + 마커 정렬)
  turn_wz: 0.066, turn_wz_min: 0.028
  turn_tolerance: 0.07
  turn_kp: 1.5, turn_kd: 0.3
  turn_scale: 1.0

  # Parking
  park_creep_vx: 0.009
  park_align_kp: 0.044, park_final_kp: 0.044
  park_marker_threshold: 0.02
  park_target_distance: 0.15     # 목표 주차 거리 (m)
  park_align_forward: 0.002      # 정렬 후 전진 (m)

  # Approach/retreat
  approach_speed: 0.01            # 차량 접근/후퇴 속도 (비활성화 상태)

  # Stall detection
  stall_check_interval: 0.4
  stall_boost_increment: 0.0033
  stall_max_boost: 0.022
  stall_pulse_duration: 0.22

  # Drive search creep (먼 거리: 4↔10, 9↔15)
  drive_search_creep_vx: 0.002
  drive_search_creep_delay: 0.5
```

## 계획

```yaml
/mission_manager:
  update_rate: 10.0
  home_marker_id: 0
  auto_start_waiting: true
  car_id: "car_01"

  # Timeouts
  timeout_align_to_marker: 10.0
  timeout_turning: 30.0
  timeout_recognize: 60.0
  timeout_load: 30.0
  timeout_unload: 30.0

  # Approach/retreat
  approach_distance: 0.30         # 차량 접근 거리 (비활성화 상태)
  approach_speed: 0.01
  retreat_from_slot_distance: 0.05  # 슬롯→도로 복귀 (m, +vy)

  # Anomaly detection
  anomaly_stop_distance: 30.0    # 정지 거리 (cm)
  anomaly_clear_timeout: 3.0     # 재개 대기 (초)

/server_bridge:
  mqtt_host: "43.202.0.116"
  mqtt_port: 8000
  robot_id: 1
  heartbeat_rate: 1.0
```
