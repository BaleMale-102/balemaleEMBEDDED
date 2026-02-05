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
  max_vx: 0.01, max_vy: 0.015, max_wz: 0.15

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
  max_vx: 0.008, max_vy: 0.015, max_wz: 0.075

  # Marker approach
  marker_vx_kp: 0.015, marker_vy_kp: 0.01, marker_wz_kp: 0.025
  marker_reach_distance: 0.25
  marker_align_threshold: 0.1

  # Turn
  turn_wz: 0.06, turn_wz_min: 0.0225
  turn_tolerance: 0.07
  turn_kp: 1.5, turn_kd: 0.3
  turn_scale: 1.0

  # Parking
  park_creep_vx: 0.003
  park_marker_threshold: 0.05
  park_target_distance: 0.15

  # Stall detection
  stall_check_interval: 0.5
  stall_boost_increment: 0.002
```

## 계획

```yaml
/mission_manager:
  update_rate: 10.0
  home_marker_id: 0
  auto_start_waiting: true
  car_id: "car_01"

  # Timeouts
  timeout_recognize: 60.0
  timeout_load: 30.0
  timeout_unload: 30.0
  timeout_turning: 30.0

/server_bridge:
  mqtt_host: "43.202.0.116"
  mqtt_port: 8000
  robot_id: 1
  heartbeat_rate: 1.0
```
