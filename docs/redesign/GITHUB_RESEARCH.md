# GitHub 레포지토리 조사 및 베스트 프랙티스

## 조사 일시
- 날짜: 2026-02-02
- 브랜치: redesign/full-rewrite-20260202

---

## 1. 마커 기반 네비게이션

### 1.1 Auto-Marker-Docking (dawan0111)
- **URL**: https://github.com/dawan0111/Auto-Marker-Docking
- **언어**: C++ 89%, Python 2.6%

**핵심 기능**:
1. ArUco 마커 검출 및 포즈 추정
2. Z축 회전 정렬 (도킹)
3. **마커 손실 예측**: 로봇의 현재 속도와 각속도 기반으로 마커 위치 예측
4. **Kalman Filter**: 검출 신뢰도 향상 및 예측 정확도 개선

**적용할 패턴**:
```python
# MarkerPredictor 개념
class MarkerPredictor:
    def predict_when_lost(self, robot_velocity, robot_angular_velocity, dt):
        # 마커가 사라졌을 때 로봇 움직임 기반 예측
        predicted_x = last_x - robot_velocity.x * dt
        predicted_z = last_z - robot_velocity.z * dt
        return predicted_pose
```

### 1.2 ros2_aruco (JMU-ROBOTICS-VIVA)
- **URL**: https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
- **언어**: Python 91.2%

**패키지 구조**:
```
ros2_aruco/
├── ros2_aruco/          # 검출 노드
└── ros2_aruco_interfaces/  # 커스텀 메시지
```

**메시지 정의**:
- `ArucoMarkers.msg`: 마커 ID 배열 + 포즈 배열

**노드 아키텍처**:
- 입력: `/camera/image_raw`, `/camera/camera_info`
- 출력: `PoseArray` (RViz용), `ArucoMarkers` (ID+포즈)

**적용할 패턴**:
- 카메라 프레임 ID를 CameraInfo에서 자동 추출
- opencv-contrib-python, transforms3d 의존성

### 1.3 ros2-aruco-pose-estimation (AIRLab-POLIMI)
- **URL**: https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation
- **언어**: Python 94.5%

**특징**:
- RGB + Depth 이미지 지원
- ROS2 Humble/Iron 호환
- YAML 기반 파라미터 설정

**토픽 구조**:
```yaml
Subscriptions:
  /camera/image_raw: sensor_msgs/Image
  /camera/depth/image_rect_raw: sensor_msgs/Image
  /camera/camera_info: sensor_msgs/CameraInfo

Publications:
  /aruco/poses: geometry_msgs/PoseArray
  /aruco/markers: aruco_interfaces/ArucoMarkers
  /aruco/image: sensor_msgs/Image (디버그)
```

### 1.4 ROS2-Aruco-TargetTracking (lapo5)
- **URL**: https://github.com/lapo5/ROS2-Aruco-TargetTracking

**출력 메시지**:
- `geometry_msgs/TransformStamped`: 마커 TF
- `std_msgs/Bool`: 마커 존재 여부

**적용할 패턴**:
- TF2 통합으로 마커→카메라 변환 퍼블리시
- Grid 기반 검출로 성능 최적화

---

## 2. 라인 검출

### 2.1 lane_detection (aelmiger)
- **URL**: https://github.com/aelmiger/lane_detection
- **타겟**: Jetson Nano 실시간 처리

**전처리 파이프라인**:
```python
def preprocess(image):
    # 1. 왜곡 보정 (Undistortion)
    # 2. 그레이스케일 변환
    # 3. 가우시안 블러
    # 4. Sobel 수평 엣지 검출 (수직선 추출)
    # 5. 이진화
    # 6. 모폴로지 침식
    return binary
```

**라인 모델**:
- **Hyperbola-pair 모델** (Bird's-eye가 아닌 원근 뷰)
- 파라미터: k(곡률), h(수평선), b(라인 오프셋), c(전체 오프셋)
- 좌우 라인 동시 피팅으로 강건성 향상

**Kalman Filter**:
- `kalman_filter.py`로 시간적 스무딩
- 프레임 간 라인 파라미터 안정화

**성능 최적화**:
- `preprocessing.py`: 표준 구현
- `preprocessing_cuda.py`: CUDA 가속

**적용할 패턴**:
```python
# Sobel 엣지 + 이진화
sobel_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
binary = (np.abs(sobel_x) > threshold).astype(np.uint8) * 255
```

---

## 3. 메카넘 로봇

### 3.1 ros2-mecanum-bot (deborggraever)
- **URL**: https://github.com/deborggraever/ros2-mecanum-bot
- **환경**: Ubuntu 22.04, ROS2 Humble
- **언어**: C++ 80%, Python 7%

**패키지 구조**:
```
mecanumbot/
├── mecanumbot_bringup/      # Launch 파일
├── mecanumbot_control/      # 제어 시스템
├── mecanumbot_controller/   # 모터 제어
├── mecanumbot_description/  # URDF
├── mecanumbot_hardware/     # HW 인터페이스
└── mecanumbot_teleop/       # 텔레옵
```

### 3.2 메카넘 역기구학 공식

**파라미터 정의**:
- `wheel_radius` (r): 바퀴 반경 [m]
- `lx`: 로봇 중심 → 바퀴 X 거리 [m]
- `ly`: 로봇 중심 → 바퀴 Y 거리 [m]
- `WHEEL_GEOMETRY = lx + ly`

**역기구학 (Velocity → Wheel Speeds)**:
```python
def mecanum_inverse_kinematics(vx, vy, wz, r, lx, ly):
    """
    vx: 전진 속도 (m/s, 양수=전방)
    vy: 횡이동 속도 (m/s, 양수=좌측)
    wz: 회전 속도 (rad/s, 양수=반시계)
    r: 바퀴 반경 (m)
    lx, ly: 바퀴 간격 (m)

    Returns: 4개 바퀴 각속도 (rad/s)
    """
    k = lx + ly  # WHEEL_GEOMETRY

    # 각 바퀴 각속도 계산
    w_fl = (vx - vy - k * wz) / r  # Front Left
    w_fr = (vx + vy + k * wz) / r  # Front Right
    w_rl = (vx + vy - k * wz) / r  # Rear Left
    w_rr = (vx - vy + k * wz) / r  # Rear Right

    return w_fl, w_fr, w_rl, w_rr
```

**순기구학 (Wheel Speeds → Velocity)**:
```python
def mecanum_forward_kinematics(w_fl, w_fr, w_rl, w_rr, r, lx, ly):
    """
    4개 바퀴 각속도 → 로봇 속도
    """
    k = lx + ly

    vx = r * (w_fl + w_fr + w_rl + w_rr) / 4
    vy = r * (-w_fl + w_fr + w_rl - w_rr) / 4
    wz = r * (-w_fl + w_fr - w_rl + w_rr) / (4 * k)

    return vx, vy, wz
```

### 3.3 ros2_controllers - mecanum_drive_controller
- **URL**: https://control.ros.org/humble/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html

**파라미터**:
- `wheels_radius`: 바퀴 반경 (> 0.0)
- `sum_of_robot_center_projection_on_X_Y_axis`: lx + ly

**입력 토픽**:
- `<controller>/linear/x/velocity` (m/s)
- `<controller>/linear/y/velocity` (m/s)
- `<controller>/angular/z/velocity` (rad/s)

**출력**:
- `<wheel_name>/velocity` (rad/s)

---

## 4. Jetson 로봇

### 4.1 jetbot_ros (dusty-nv)
- **URL**: https://github.com/dusty-nv/jetbot_ros

**구조**:
```
jetbot_ros/
├── jetbot_ros/    # 코어 노드
├── gazebo/        # 시뮬레이션
├── launch/        # Launch 파일
├── data/          # 모델/데이터
└── docker/        # 컨테이너
```

**패턴**:
- Docker 컨테이너화로 의존성 격리
- Gazebo 시뮬레이션 지원
- PyTorch 모델 저장 (.pth)

**텔레옵**:
- w/x: 선속도 증가/감소
- a/d: 각속도 증가/감소

---

## 5. Kalman Filter 구현

### 5.1 2D 위치 + 속도 추적 Kalman Filter

```python
import numpy as np

class KalmanFilter2D:
    """
    상태: [x, z, vx, vz] (4차원)
    측정: [x, z] (2차원)
    """

    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        # 상태 벡터: [x, z, vx, vz]
        self.x = np.zeros(4)

        # 상태 전이 행렬 (dt는 predict에서 업데이트)
        self.F = np.eye(4)

        # 측정 행렬: 위치만 측정
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # 공분산 행렬
        self.P = np.eye(4) * 1.0

        # 프로세스 노이즈
        self.Q = np.eye(4) * process_noise

        # 측정 노이즈
        self.R = np.eye(2) * measurement_noise

    def predict(self, dt):
        """시간 dt 후 상태 예측"""
        # 상태 전이 행렬 업데이트
        self.F[0, 2] = dt  # x += vx * dt
        self.F[1, 3] = dt  # z += vz * dt

        # 예측
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        return self.x[:2]  # 위치만 반환

    def update(self, z):
        """측정값으로 상태 업데이트"""
        # 칼만 게인
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 상태 업데이트
        y = z - self.H @ self.x  # 잔차
        self.x = self.x + K @ y

        # 공분산 업데이트
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

        return self.x[:2]

    def get_velocity(self):
        """추정된 속도 반환"""
        return self.x[2], self.x[3]  # vx, vz
```

### 5.2 1D Kalman Filter (간단 버전)

```python
class SimpleKalman1D:
    """1D Kalman Filter with velocity estimation"""

    def __init__(self, q_pos=0.01, q_vel=0.001, r=0.1):
        self.x = 0.0   # 위치
        self.v = 0.0   # 속도
        self.p = 1.0   # 공분산
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r = r
        self.last_t = None

    def update(self, z, t):
        """측정값 z를 시간 t에 업데이트"""
        if self.last_t is not None:
            dt = t - self.last_t
            if dt > 0.001:
                # 속도 추정 (EMA)
                new_v = (z - self.x) / dt
                self.v = 0.7 * self.v + 0.3 * new_v

        self.last_t = t

        # Kalman 업데이트
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)

        return self.x

    def predict(self, dt):
        """dt 후 위치 예측"""
        self.p += self.q_pos + self.q_vel * dt
        return self.x + self.v * dt
```

---

## 6. 적용할 베스트 프랙티스

### 6.1 패키지 구조
```
src/
├── interfaces/         # 메시지/서비스 정의
├── drivers/           # 하드웨어 드라이버
├── perception/        # 인식 (마커, 라인)
├── control/           # 모션 제어
├── planning/          # 경로/미션 계획
└── bringup/           # Launch, 설정
```

### 6.2 메시지 설계 패턴
- 표준 메시지 최대한 활용 (`geometry_msgs`, `sensor_msgs`)
- 커스텀 메시지는 도메인 특화 정보만
- Header 필수 포함 (타임스탬프, 프레임 ID)

### 6.3 노드 설계 패턴
- 단일 책임 원칙: 노드당 하나의 기능
- 파라미터화: YAML 설정 파일
- 시뮬레이션 모드 지원

### 6.4 제어 패턴
- FSM 기반 상태 관리
- Watchdog으로 안전 정지
- 속도 제한 및 가속 제한

### 6.5 인식 패턴
- Kalman Filter로 노이즈 필터링
- 예측 모드로 일시적 손실 대응
- 디버그 이미지 퍼블리시

---

## 7. 라이선스 확인

| 레포 | 라이선스 | 코드 사용 가능 |
|------|---------|---------------|
| Auto-Marker-Docking | 명시 안됨 | 참고만 |
| ros2_aruco | MIT 추정 | 가능 |
| ros2-mecanum-bot | GPL-3.0 | 가능 (GPL 조건) |
| lane_detection | MIT | 가능 |
| jetbot_ros | MIT | 가능 |
| ros2_controllers | Apache-2.0 | 가능 |

---

## 8. 결론 및 적용 계획

### 마커 검출
- ros2_aruco 패턴 참고하여 Python 구현
- aruco_interfaces 메시지 구조 채택

### 마커 추적
- Auto-Marker-Docking의 Kalman 예측 패턴 적용
- 2D Kalman Filter로 위치+속도 추적

### 라인 검출
- Sobel + 이진화 파이프라인 적용
- SimpleKalman1D로 오프셋 스무딩

### 메카넘 제어
- 역기구학 공식 직접 구현
- PWM 변환 레이어 분리

### 전체 구조
- 모듈화된 패키지 구조
- YAML 기반 파라미터 관리
- 시뮬레이션 모드 지원
