# 시스템 아키텍처

## 패키지 구조
```
balemaleEMBEDDED/
├── Arduino/
│   ├── Motor/Arduino_MoebiusTech_v2.ino
│   └── Lift/carrying.ino
├── src/
│   ├── bringup/robot_bringup/
│   │   ├── config/ (robot_params.yaml, marker_map.yaml)
│   │   └── launch/ (system, sensors, robot, server_test)
│   ├── drivers/
│   │   ├── camera_driver/
│   │   ├── arduino_driver/
│   │   ├── loader_driver/
│   │   └── imu_driver/
│   ├── perception/
│   │   ├── marker_detector/
│   │   ├── marker_tracker/
│   │   ├── anpr_detector/ (anomaly_node, ocr_node, detector_node)
│   │   └── slot_line_detector/ (비활성화)
│   ├── control/motion_controller/
│   ├── planning/
│   │   ├── mission_manager/
│   │   └── server_bridge/
│   └── interfaces/robot_interfaces/
├── scripts/ (full.sh 등 테스트 스크립트)
└── docs/
```

## 노드 실행 매핑
| 패키지 | 노드명 | Launch |
|--------|--------|--------|
| camera_driver | cam_front, cam_side | sensors |
| arduino_driver | arduino_driver | sensors |
| imu_driver | imu_node | sensors |
| marker_detector | marker_detector, side_marker_detector | robot |
| marker_tracker | marker_tracker | robot |
| motion_controller | motion_controller | robot |
| loader_driver | loader_driver | robot |
| mission_manager | mission_manager | robot |
| server_bridge | server_bridge | robot |
| anpr_detector | anomaly_detector | robot (launch 포함) |
| anpr_detector | ocr_detector | robot (launch 포함) |
| anpr_detector | anpr_detector (ANPR) | **별도 (conda)** |

---

## 미션 FSM

### 입고 플로우 (PARK)
```
IDLE → WAIT_VEHICLE → RECOGNIZE → LOAD → DRIVE → PARK_* → UNLOAD → RETURN_HOME
                                    ↑                                    ↓
                                    │                         RETREAT_FROM_SLOT
                                    │                                    ↓
                                    │                              TURNING(180°)
                                    │                                    ↓
                                    └────── WAIT_VEHICLE ← DRIVE(역순) ←┘
```

### 출차 플로우 (EXIT)
```
서버 출차명령 (슬롯ID + 경로)
    ↓
DRIVE → PARK_* → LOAD → RETURN_HOME → RETREAT_FROM_SLOT → TURNING(180°)
(슬롯으로)  (정렬)  (적재)                                        ↓
                                    WAIT_VEHICLE ← UNLOAD ← DRIVE(역순)
```

### 주행 사이클
```
DRIVE → STOP_AT_MARKER → ADVANCE_TO_CENTER → STOP_BUMP → TURNING → ALIGN_TO_MARKER → DRIVE
                                                             │
                                                     (먼 거리 전환: 4↔10, 9↔15)
                                                             └──→ DRIVE (ALIGN 건너뜀)
```

### DRIVE 제어 방식
```
항상 vx + vy + wz 동시 적용:
  vx = marker_vx_kp × (distance - min_dist) × angle_factor
  vy = -marker_vy_kp × angle
  wz = marker_wz_kp × angle × 0.3 (부드러운 조향)
  angle_factor = max(0.5, 1.0 - |angle| / 60°)  # 각도 클수록 감속
```

### TURN 제어 방식
```
1. 마커 미발견: 고정 속도 회전 (turn_wz × direction) + stall boost
2. 마커 발견: marker_wz_kp × angle로 정렬, |angle| < threshold(0.1rad)까지
3. IMU fallback: 마커 한 번도 안 보이면 목표각 90% 도달 시 완료
```

### 주차 사이클
```
PARK_DETECT → PARK_ALIGN_MARKER → PARK_FINAL
      ↓ (잘못된 zone)
  PARK_RECOVERY
```

### RETURN_HOME 복귀 로직
```
1. waypoints 역순 생성 (현재 마커 스킵, HOME 마커 제거)
2. current_waypoint_idx = -1
3. RETREAT_FROM_SLOT: +vy로 슬롯에서 도로로 횡이동 (0.05m)
4. TURNING: 180° 회전 (역방향)
5. DRIVE: 역순 waypoints 따라 HOME(마커 0)으로
```

### 이상탐지 (Emergency Stop)
```
전방 카메라: /perception/anomaly/detections (person/box/cone)
사이드 카메라: /perception/side_anomaly/detections (person/box/cone)
  ↓ distance < anomaly_stop_distance (30cm)
  ↓
PAUSE (cmd_vel=0) → 3초간 미감지 → RESUME
  ↓
/server/anomaly_report (JSON: type, distance, event_type, state, marker_id)
```

### 상태 설명
| 상태 | 입고(PARK) | 출차(EXIT) |
|------|-----------|-----------|
| WAIT_VEHICLE | 홈에서 차량 대기 (ANPR) | 출차 하역 후 대기 |
| RECOGNIZE | 서버에 번호판 조회 | - |
| DRIVE | 홈→슬롯 주행 | 홈→슬롯 / 슬롯→홈 |
| PARK_* | 슬롯 정렬 (하역 전) | 슬롯 정렬 (적재 전) |
| LOAD | 홈에서 차량 적재 | 슬롯에서 차량 적재 |
| UNLOAD | 슬롯에서 차량 하역 | 홈에서 차량 하역 |
| RETURN_HOME | 슬롯→홈 복귀 설정 | 슬롯→홈 복귀 설정 |
| RETREAT_FROM_SLOT | 슬롯→도로 횡이동 | 슬롯→도로 횡이동 |
| TURNING | 제자리 회전 | 제자리 회전 |

---

## 마커 맵

### 레이아웃 (cm)
```
y=18   |   0   |         ← 시작 (홈)
y=54   2---1---3         ← 갈림길
y=82.5 | 16-19 |         ← 주차 A
y=111  4-5-6-7-8-9       ← 중간
y=136  | 20-23 |         ← 주차 B
y=162  | 24-27 |         ← 주차 C
y=187  10-11-14-15       ← 하단
```

### 슬롯 그룹
- A: ID 16-19 (y=82.5)
- B: ID 20-23 (y=136)
- C: ID 24-27 (y=162)

### 먼 거리 전환 쌍 (ALIGN 건너뜀 + search creep)
- 4 ↔ 10, 9 ↔ 15

---

## TF 프레임
```
odom
└── base_link
    ├── camera_front (0.10m 전방)
    ├── camera_side (0.10m 좌측, yaw +90°)
    └── imu_link
```

---

## 모터 방향 보정

**중요: vy와 wz 부호가 ROS 표준과 반대**

```python
vy = -marker_vy_kp * angle   # 좌우 이동 반전
cmd.angular.z = -wz          # 회전 반전
turned = start_yaw - current_yaw  # IMU 반전
```

## 스톨 감지 시스템

| 모드 | 감지 방식 | 부스트 |
|------|----------|--------|
| TURN | yaw 변화 < 0.005rad | wz boost + pulse(×3) |
| PARK_ALIGN | angle/offset_x 변화 | vx boost + pulse(×3) |
| PARK_FINAL | distance 변화 < 1mm | vy boost + pulse(×3) |
| DRIVE | **없음** (삭제됨) | - |
