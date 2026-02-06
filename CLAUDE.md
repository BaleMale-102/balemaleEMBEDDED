# CLAUDE.md - RC Car Autonomous System

## 필수 규칙
- **좌표계**: ROS 표준 (X-전방, Y-좌측, Z-상방, 반시계 양수)
- **단위**: 거리 m, 각도 rad, 속도 m/s
- **모터 방향**: vy, wz 부호 반전 필요 (실제 로봇 테스트 결과)

## 현재 상태
- **시스템**: 주차 로봇 (차량 적재/하역)
- **주행**: 마커 전용 (라인 인식 비활성화)
- **Loader**: serial_bridge 모드 (`~/testArduino/serial_bridge.py` 별도 실행)
- **ANPR**: conda anpr_310 환경에서 별도 실행

## 미션 플로우
```
입고(PARK): WAIT_VEHICLE → RECOGNIZE → APPROACH_VEHICLE → LOAD → RETREAT_FROM_VEHICLE → DRIVE → PARK → UNLOAD → RETURN_HOME → DRIVE(역순) → WAIT_VEHICLE
출차(EXIT): DRIVE → PARK_* → LOAD → RETURN_HOME → DRIVE(역순) → UNLOAD → WAIT_VEHICLE
```
- **APPROACH_VEHICLE**: 차량 방향 전진 (하드코딩 거리, 비전 미사용)
- **RETREAT_FROM_VEHICLE**: 적재 후 원위치 후퇴 (동일 거리)
- **RETURN_HOME**: waypoints 역순 설정 후 즉시 DRIVE 전환
- 파라미터: `approach_distance` (m), `approach_speed` (m/s)

---

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mission/state` | String | FSM 상태 |
| `/mission/target_marker` | Int32 | 목표 마커 ID |
| `/control/cmd_vel` | Twist | 모터 명령 |
| `/perception/tracked_marker` | TrackedMarker | 추적 마커 |
| `/perception/side_markers` | MarkerArray | 주차용 마커 |
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
ros2 launch robot_bringup system.launch.py          # 터미널 1: 메인
cd ~/testArduino && python3 serial_bridge.py        # 터미널 2: Loader
conda activate anpr_310 && ros2 run anpr_detector detector_node  # 터미널 3: ANPR

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

# 테스트 미션 (입고: WAIT→RECOGNIZE→APPROACH→LOAD→RETREAT→DRIVE→PARK→UNLOAD→RETURN→WAIT)
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
