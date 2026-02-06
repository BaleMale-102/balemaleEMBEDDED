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
│   │   ├── anpr_detector/
│   │   └── slot_line_detector/ (비활성화)
│   ├── control/motion_controller/
│   ├── planning/
│   │   ├── mission_manager/
│   │   └── server_bridge/
│   └── interfaces/robot_interfaces/
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
| anpr_detector | anpr_detector | **별도 (conda)** |

---

## 미션 FSM

### 입고 플로우 (PARK)
```
IDLE → WAIT_VEHICLE → RECOGNIZE → LOAD → DRIVE → PARK → UNLOAD → RETURN_HOME
                                    ↑                              ↓
                                    └──────────────────────────────┘
```

### 출차 플로우 (EXIT)
```
서버 출차명령 (슬롯ID + 경로)
    ↓
DRIVE → PARK_* → LOAD → RETURN_HOME → UNLOAD → WAIT_VEHICLE
 (슬롯으로)  (정렬)  (적재)   (홈 복귀)    (하역)
```

### 주행 사이클
```
DRIVE → STOP_AT_MARKER → ADVANCE_TO_CENTER → STOP_BUMP → TURNING → ALIGN_TO_MARKER → DRIVE
```

### 주차 사이클
```
PARK_DETECT → PARK_ALIGN_MARKER → PARK_FINAL
      ↓ (잘못된 zone)
  PARK_RECOVERY
```

### 상태 설명
| 상태 | 입고(PARK) | 출차(EXIT) |
|------|-----------|-----------|
| WAIT_VEHICLE | 홈에서 차량 대기 (ANPR) | 출차 하역 후 대기 |
| RECOGNIZE | 서버에 번호판 조회 | - |
| DRIVE | 홈→슬롯 주행 | 홈→슬롯 주행 / 슬롯→홈 복귀 |
| PARK_* | 슬롯 정렬 (하역 전) | 슬롯 정렬 (적재 전) |
| LOAD | 홈에서 차량 적재 | 슬롯에서 차량 적재 |
| UNLOAD | 슬롯에서 차량 하역 | 홈에서 차량 하역 |
| RETURN_HOME | 슬롯→홈 복귀 | 슬롯→홈 복귀 (차량 탑재) |
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
