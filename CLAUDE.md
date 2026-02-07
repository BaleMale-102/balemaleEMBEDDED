# CLAUDE.md - RC Car Autonomous System

## 필수 규칙
- **좌표계**: ROS 표준 (X-전방, Y-좌측, Z-상방, 반시계 양수)
- **단위**: 거리 m, 각도 rad, 속도 m/s
- **모터 방향**: vy, wz 부호 반전 필요 (실제 로봇 테스트 결과)

## 현재 상태
- **시스템**: 주차 로봇 (차량 적재/하역)
- **주행**: 마커 전용 (라인 인식 비활성화), vx+vy+wz 동시 제어
- **Loader**: serial_bridge 모드 (`~/testArduino/serial_bridge.py` 별도 실행)
- **ANPR**: conda anpr_310 환경에서 별도 실행
- **이상탐지**: anomaly_detector (전방) + ocr_detector (사이드, anomaly 겸용) launch에 포함
- **APPROACH/RETREAT_VEHICLE**: 코드 존재하나 **비활성화** (주석처리)

## 미션 플로우
```
입고(PARK): WAIT_VEHICLE → RECOGNIZE → LOAD → DRIVE → PARK_* → UNLOAD → RETURN_HOME → RETREAT_FROM_SLOT → TURNING → DRIVE(역순) → WAIT_VEHICLE
출차(EXIT): DRIVE → PARK_* → LOAD → RETURN_HOME → RETREAT_FROM_SLOT → TURNING → DRIVE(역순) → UNLOAD → WAIT_VEHICLE
```
- **RETREAT_FROM_SLOT**: 주차 슬롯에서 도로로 횡이동 (+vy, 하드코딩 0.02m/s)
- **RETURN_HOME**: waypoints 역순 설정 후 즉시 RETREAT_FROM_SLOT 전환
- **DRIVE 제어**: 항상 전진(vx) + 횡이동(vy) + 부드러운 조향(wz*0.3) 동시 적용
- **TURN 제어**: 마커 발견 시 angle < threshold까지 정렬, IMU fallback (마커 미감지 시 90% 목표각)
- 파라미터: `retreat_from_slot_distance` (m), `approach_speed` (m/s)

---

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mission/state` | String | FSM 상태 |
| `/mission/target_marker` | Int32 | 목표 마커 ID |
| `/control/cmd_vel` | Twist | 모터 명령 |
| `/perception/tracked_marker` | TrackedMarker | 추적 마커 |
| `/perception/side_markers` | MarkerArray | 주차용 마커 |
| `/perception/anomaly/detections` | DetectionArray | 전방 장애물 (person/box/cone) |
| `/perception/side_anomaly/detections` | DetectionArray | 사이드 장애물 (person/box/cone) |
| `/server/anomaly_report` | String (JSON) | 서버에 이상 보고 |
| `/loader/command` | LoaderCommand | 적재 명령 (LOAD/UNLOAD) |
| `/loader/status` | LoaderStatus | 적재 상태 |
| `/motor_cmd` | String | Arduino 명령 (serial_bridge) |
| `/loading_status` | String | Arduino 응답 (serial_bridge) |
| `/plate/query` | PlateQuery | 번호판 조회 |
| `/plate/response` | PlateResponse | 서버 응답 |

---

## 빌드 및 실행

```bash
# 빌드
cd ~/balemaleEMBEDDED && colcon build && source install/setup.bash

# 실행 (3개 터미널)
ros2 launch robot_bringup system.launch.py          # 터미널 1: 메인 (anomaly/ocr 포함)
cd ~/testArduino && python3 serial_bridge.py        # 터미널 2: Loader
conda activate anpr_310 && ros2 run anpr_detector detector_node  # 터미널 3: ANPR

# 풀 미션 스크립트 (입고)
./scripts/full.sh 0,1,5 17                          # 0→1→5→slot17 → 하역 → 복귀
./scripts/full.sh 0,1,5 17 12가3456                  # 번호판 지정

# 서버 통신만 테스트
ros2 launch robot_bringup server_test.launch.py simulation:=true
```

---

## 디버깅 명령

```bash
# 상태 확인
ros2 topic echo /mission/state
ros2 topic echo /loader/status
ros2 topic echo /perception/tracked_marker
ros2 topic echo /perception/anomaly/detections

# 테스트 미션 (입고: WAIT→RECOGNIZE→LOAD→DRIVE→PARK→UNLOAD→RETURN→WAIT)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'WAIT'"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'PLATE 12가3456'"
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'VERIFY 17 0,1,5'"

# 테스트 미션 (출차: DRIVE→PARK→LOAD→RETURN→DRIVE(역순)→UNLOAD→WAIT)
ros2 topic pub --once /mission/test_cmd std_msgs/String "data: 'EXIT 1,5 17'"

# 로더 테스트
ros2 topic pub --once /loader/command robot_interfaces/LoaderCommand "{command: 'LOAD'}"
ros2 topic pub --once /loading_status std_msgs/String "data: 'completed'"  # 응답 시뮬

# 긴급 정지
ros2 topic pub /control/cmd_vel geometry_msgs/Twist "{}"
```

---

## 코드 작성

```python
# 메시지 import
from robot_interfaces.msg import (
    Marker, MarkerArray, TrackedMarker,
    MissionCommand, MissionStatus, DrivingState,
    LoaderCommand, LoaderStatus,
    PlateQuery, PlateResponse,
    Detection, DetectionArray
)

# 속도 제한
def clamp(v, lo, hi): return max(lo, min(hi, v))

# 각도 정규화
def wrap_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a
```

---

## 상세 문서 (docs/)
- `docs/ARCHITECTURE.md` - 패키지 구조, FSM, 마커 맵
- `docs/PROTOCOLS.md` - Arduino, MQTT 프로토콜
- `docs/PARAMETERS.md` - robot_params.yaml 설정
- `docs/MESSAGES.md` - 커스텀 메시지 정의