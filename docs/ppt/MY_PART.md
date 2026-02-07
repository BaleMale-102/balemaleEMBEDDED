# 담당 파트 정리

## 1. 하드웨어 구성 및 연동

| 구성요소 | 역할 | 통신 |
|----------|------|------|
| Jetson Orin Nano | 메인 컴퓨터 | - |
| Arduino UNO + PCA9685 | 메카넘 휠 제어 | Serial 115200 |
| Logitech C920 / Brio 100 | 전방/측면 카메라 | USB |
| MPU6050 | IMU (회전 감지) | I2C |

---

## 2. 소프트웨어 개발

### 2.1 드라이버 (drivers/)
- **camera_driver**: USB 카메라 V4L2 연동
- **arduino_driver**: 모터 시리얼 통신, Watchdog
- **imu_driver**: MPU6050 I2C, Complementary Filter
- **loader_driver**: 리프트 제어 (serial_bridge 연동)

### 2.2 인식 (perception/)
- **marker_detector**: ArUco 마커 검출 (DICT_4X4_50)
- **marker_tracker**: 칼만 필터 추적, 2초 예측

### 2.3 제어 (control/)
- **motion_controller**: 주행/회전/주차 모드 통합

### 2.4 미션 (planning/)
- **mission_manager**: FSM 상태 관리

---

## 3. 핵심 알고리즘

### 3.1 메카넘 휠 믹싱
```
FL = +vx +vy +wz    FR = +vx -vy -wz
RL = +vx -vy +wz    RR = +vx +vy -wz
```

### 3.2 칼만 필터 (마커 추적)
- 검출 불안정 → 예측으로 보완
- 마커 미검출시 2초간 예측 위치 사용

### 3.3 IMU 기반 회전 제어
- Complementary Filter로 Yaw 추정
- PD 제어 (kp=1.5, kd=0.3)

### 3.4 FSM (유한 상태 기계)
```
WAIT → LOAD → DRIVE → PARK → UNLOAD → RETURN
```

---

## 4. 주요 문제 해결

| 문제 | 원인 | 해결 |
|------|------|------|
| 로봇 반대로 이동 | 모터 방향 불일치 | vy, wz **부호 반전** |
| 직진시 휘어짐 | 모터 출력 차이 | Motor 출력 **스케일링** |
| 마커 깜빡임 | 검출 불안정 | **칼만 필터** 적용 |
| 회전 부정확 | 미끄러짐/관성 | **IMU PD 제어** |
| 모터 정지 감지 불가 | 엔코더 없음 | **IMU/마커 변화량** 모니터링 |
| 저전압시 멈춤 | 배터리 전압 저하 | **MAX_PWM** 파라미터 추가 전압에 따라 조정|

---

## 5. 개발 과정 요약

| 단계 | 내용 |
|------|------|
| 1 | 아키텍처 설계 + 드라이버 개발 |
| 2 | 마커 인식 + 칼만 필터 구현 |
| 3 | 메카넘 휠 제어 (방향 보정) |
| 4 | IMU 회전 제어 완성 |
| 5 | FSM 미션 관리자 구현 |
| 6 | 주차 정렬 알고리즘 (3단계) |
| 7 | 버그 수정 및 파라미터 튜닝 |
