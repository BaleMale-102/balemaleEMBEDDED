# fix_marker_align_and_parking 브랜치 수정 사항

## 브랜치 정보
- **브랜치명:** `fix_marker_align_and_parking`
- **생성일:** 2026-02-06
- **기준 브랜치:** `main`

---

## 커밋 내역

### 1. `d4a972c` - 마커 기반 회전 종료 및 주차 정렬 개선

#### 수정 파일
- `src/control/motion_controller/motion_controller/controller_node.py`
- `src/planning/mission_manager/mission_manager/state_machine.py`

#### 수정 내용

| 항목 | 기존 | 수정 |
|------|------|------|
| 회전 종료 조건 | IMU 기반 90도 고정 | 마커 감지 시 즉시 종료 |
| ALIGN 순서 | VY → WZ | WZ → VY |
| PARK_ALIGN_MARKER vx | `vx = kp * angle` | `vx = -kp * angle` (부호 반전) |
| PARK_ALIGN_RECT | 실행 | 스킵 (PARK_FINAL로 직행) |

#### 상세 설명

**1) 회전 중 마커 감지 시 즉시 종료**
- 문제: IMU가 부정확해서 90도 회전이 정확하지 않음
- 해결: `_turn_control()`에서 타겟 마커 감지(confidence > 0.5) 시 즉시 turn_done 발행
- ALIGN 모드에서 정밀 정렬 수행

**2) ALIGN 순서 변경 (WZ → VY)**
- 기존: 좌우이동(VY) 먼저 → 회전(WZ) 나중
- 변경: 회전(WZ) 먼저 → 좌우이동(VY) 나중
- 방향 먼저 맞추고 측면 정렬하는 방식

**3) PARK_ALIGN_MARKER side camera 좌표계 수정**
- Side camera가 왼쪽을 바라봄 (yaw = +90도)
- `angle = atan2(-tvec[0], tvec[2])` 에서:
  - angle > 0: 마커가 카메라 왼쪽 = 로봇 뒤쪽 → 후진 필요
  - angle < 0: 마커가 카메라 오른쪽 = 로봇 앞쪽 → 전진 필요
- 부호 반전 적용: `vx = -park_align_kp * angle`

**4) PARK_ALIGN_RECT 스킵**
- slot_line_detector 미사용으로 해당 단계 제거
- PARK_ALIGN_MARKER → PARK_FINAL 직접 전환

---

### 2. `d9a8450` - 마지막 waypoint에서 불필요한 TURNING 제거

#### 수정 파일
- `src/planning/mission_manager/mission_manager/state_machine.py`

#### 수정 내용

**문제 상황:**
```
waypoint_ids = [1, 5], final_goal_id = 17 (주차슬롯)

마커5 도착 → STOP_BUMP
- idx = 1, is_last_waypoint = (1 >= 2) = False
- → TURNING 실행! (slot 방향으로 회전)
```

**원인:**
- `advance_waypoint()`가 TURNING 완료 후 호출됨
- STOP_BUMP 시점에서는 아직 idx가 증가하지 않음
- `is_last_waypoint` 판정이 잘못됨

**수정:**
```python
# 기존
if self._context.is_last_waypoint:

# 수정
next_idx = self._context.current_waypoint_idx + 1
has_more_waypoints = next_idx < len(self._context.waypoint_ids)
if not has_more_waypoints:
```

**수정 후 플로우:**
```
마커5 도착 → STOP_BUMP → PARK_DETECT (TURNING 없이!)
```

---

### 3. `de2cba6` - RETURN_HOME 시 왔던 길 역순으로 복귀

#### 수정 파일
- `src/planning/mission_manager/mission_manager/state_machine.py`

#### 수정 내용

**기존:**
```python
self._context.waypoint_ids = []  # 빈 리스트
self._context.final_goal_id = HOME_MARKER_ID
```
- 중간 경유지 없이 마커 0 직행 시도

**수정:**
```python
original_waypoints = self._context.waypoint_ids.copy()
self._context.waypoint_ids = list(reversed(original_waypoints))
self._context.final_goal_id = HOME_MARKER_ID
```

**플로우 예시:**
```
갈 때: waypoints [1, 5] → slot 17
올 때: waypoints [5, 1] → home 0
```

---

### 4. 7.4V 배터리 보상 및 제어 개선 (다수 커밋)

#### 수정 파일
- `src/control/motion_controller/motion_controller/controller_node.py`
- `src/planning/mission_manager/mission_manager/state_machine.py`
- `src/planning/mission_manager/mission_manager/manager_node.py`
- `src/bringup/robot_bringup/config/robot_params.yaml`

#### 수정 내용

**1) 7.4V 배터리 속도 보상**
- 12V→7.4V 변경으로 전체 속도 파라미터 ~1.15x 조정
- Arduino max_pwm: 2000 → 3200
- 주차 관련 속도 2배 강화 (park_align_kp, park_max_vx 등)

**2) ADVANCE_TO_CENTER 스킵**
- `advance_time: 0.0` → 해당 단계 즉시 완료

**3) Move-Pause-Observe 패턴 (카메라 딜레이 보상)**
- ALIGN-WZ, ALIGN-VY, PARK_ALIGN_MARKER, PARK_FINAL에 적용
- `align_move_duration: 0.15` (이동), `align_settle_duration: 0.3` (관찰)
- 이동 → 멈춤 → 카메라 안정화 → 측정 → 이동 반복

**4) Waypoint 라우팅 버그 수정**
- `test_drive 1,3,9` 시 마커 3에서 FINISH되는 문제
- `_stop_bump_transition`에서 `is_last_waypoint` 판정 수정
- 마지막 waypoint ≠ final_goal일 때 계속 주행

**5) Turn 방향 레이스 컨디션 수정**
- TURNING 중 `current_target_marker`가 이전 마커를 가리키는 문제
- manager_node: TURNING 상태에서 다음 타겟 마커 발행
- controller_node: `_turn_last_target` 변경 감지 → 타겟 변경 시 턴 재초기화
- IMU 턴 완료 시 `_marker_seen_during_turn=True` → 반대 방향 탐색 유도

**6) START_PARK 직접 진입**
- 빈 waypoint + PARK 태스크 시 DRIVE 대신 PARK_DETECT로 직행

**7) PARK_ALIGN_MARKER 2단계 정렬**
- Phase 1 (ANGLE): `marker.angle` 기반 대략적 전후 정렬 → `park_marker_threshold` (0.02rad) 이하 시 전환
- Phase 2 (CENTER): `marker.pose.position.x` (tvec[0]) 기반 미세 중앙 정렬 → `park_center_threshold` (1cm) 이하 + 0.5초 안정 시 완료
- angle 크게 벗어나면 Phase 1로 자동 복귀

**8) PARK_FINAL 완료 후 고정 전진**
- 거리 조정 완료 후 `park_align_forward` (2mm)만큼 전진 후 done
- 0으로 설정 시 스킵

#### 추가된 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `align_move_duration` | 0.15 | 정렬 시 이동 시간 (초) |
| `align_settle_duration` | 0.3 | 정렬 시 관찰 시간 (초) |
| `park_center_threshold` | 0.01 | 중앙정렬 완료 임계값 (m) |
| `park_side_camera_offset_x` | 0.0 | side camera→로봇 중앙 전방 오프셋 (m) |
| `park_align_forward` | 0.002 | PARK_FINAL 후 전진 거리 (m) |

---

## 테스트 시 확인 사항

### 1. 회전 및 정렬
- [ ] 회전 중 타겟 마커가 보이면 즉시 멈추고 ALIGN 모드 진입하는지
- [ ] ALIGN에서 WZ(회전) 먼저 → VY(좌우) 나중 순서로 정렬하는지
- [ ] `/mission/state` 토픽에서 상태 전환 확인

### 2. 주차 진입
- [ ] 마지막 waypoint에서 TURNING 없이 바로 PARK_DETECT 진입하는지
- [ ] 로봇 heading이 유지되는지 (slot 방향으로 회전하지 않는지)

### 3. 주차 정렬 (PARK_ALIGN_MARKER)
- [x] side camera 마커 angle로 전후진(vx)만 수행하는지
- [x] 회전(wz) 없이 heading 유지하는지
- [x] 부호 방향 확인: angle > 0 → 후진, angle < 0 → 전진
- [x] Phase 1 (ANGLE) → Phase 2 (CENTER) 순서로 진행
- [x] move-pause-observe 패턴 적용

### 4. 주차 최종 (PARK_FINAL)
- [ ] side camera 마커 distance로 측면이동(vy)하는지
- [ ] target distance (15cm)에 도달하는지
- [ ] 거리 조정 후 고정 전진 (park_align_forward) 수행하는지

### 5. 복귀
- [ ] UNLOAD 완료 후 자동으로 RETURN_HOME 전환하는지
- [ ] 왔던 길 역순으로 waypoint 경유하는지
- [ ] home(마커 0)에 도착 후 WAIT_VEHICLE로 전환하는지

---

## 디버깅 명령어

```bash
# 상태 모니터링
ros2 topic echo /mission/state

# 속도 명령 확인
ros2 topic echo /control/cmd_vel

# side marker 확인
ros2 topic echo /perception/side_markers

# 전체 로그
ros2 topic echo /driving/state
```

---

## 롤백 방법

```bash
# main으로 복귀
git checkout main

# 또는 특정 커밋으로 복귀
git checkout d4a972c  # 첫 번째 수정만 적용
```
