#!/usr/bin/env python3
"""
kalman_filter.py - 2D Kalman Filter for marker tracking

상태: [x, z, vx, vz]  - 위치와 속도
측정: [x, z]          - 위치만 측정
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class KalmanState:
    """Kalman 필터 상태"""
    x: float = 0.0      # X 위치
    z: float = 0.0      # Z 위치 (거리)
    vx: float = 0.0     # X 속도
    vz: float = 0.0     # Z 속도 (접근 속도)
    confidence: float = 0.0


class KalmanFilter2D:
    """
    2D 위치 + 속도 추적 Kalman Filter

    마커 위치를 추적하고 사라졌을 때 예측합니다.
    """

    def __init__(
        self,
        process_noise: float = 0.01,
        measurement_noise: float = 0.05,
        initial_covariance: float = 1.0
    ):
        """
        Args:
            process_noise: 프로세스 노이즈 (작을수록 부드러움)
            measurement_noise: 측정 노이즈 (작을수록 측정값 신뢰)
            initial_covariance: 초기 공분산
        """
        # 상태 벡터: [x, z, vx, vz]
        self.x = np.zeros(4)

        # 상태 전이 행렬 (dt는 predict에서 업데이트)
        self.F = np.eye(4)

        # 측정 행렬: 위치만 측정
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float64)

        # 공분산 행렬
        self.P = np.eye(4) * initial_covariance

        # 프로세스 노이즈 행렬
        self.process_noise = process_noise
        self.Q = np.eye(4) * process_noise

        # 측정 노이즈 행렬
        self.measurement_noise = measurement_noise
        self.R = np.eye(2) * measurement_noise

        # 초기화 상태
        self._initialized = False
        self._last_time = 0.0

    def reset(self, x: float = 0.0, z: float = 0.0, time: float = 0.0):
        """필터 리셋"""
        self.x = np.array([x, z, 0.0, 0.0])
        self.P = np.eye(4) * 1.0
        self._initialized = True
        self._last_time = time

    def predict(self, dt: float) -> KalmanState:
        """
        상태 예측 (측정 없이)

        Args:
            dt: 시간 간격 (초)

        Returns:
            예측된 상태
        """
        if not self._initialized or dt <= 0:
            return KalmanState(
                x=self.x[0], z=self.x[1],
                vx=self.x[2], vz=self.x[3],
                confidence=0.0
            )

        # 상태 전이 행렬 업데이트
        self.F[0, 2] = dt  # x += vx * dt
        self.F[1, 3] = dt  # z += vz * dt

        # 예측
        self.x = self.F @ self.x

        # 공분산 예측
        Q_scaled = self.Q * (1 + dt)  # 시간에 따라 불확실성 증가
        self.P = self.F @ self.P @ self.F.T + Q_scaled

        # 신뢰도 계산 (공분산 기반)
        trace = np.trace(self.P[:2, :2])
        confidence = max(0.0, 1.0 - trace * 0.1)

        return KalmanState(
            x=float(self.x[0]),
            z=float(self.x[1]),
            vx=float(self.x[2]),
            vz=float(self.x[3]),
            confidence=confidence
        )

    def update(self, x_meas: float, z_meas: float, time: float) -> KalmanState:
        """
        측정값으로 상태 업데이트

        Args:
            x_meas: 측정된 X 위치
            z_meas: 측정된 Z 위치
            time: 현재 시간

        Returns:
            업데이트된 상태
        """
        z = np.array([x_meas, z_meas])

        if not self._initialized:
            # 첫 측정으로 초기화
            self.reset(x_meas, z_meas, time)
            return KalmanState(
                x=x_meas, z=z_meas,
                vx=0.0, vz=0.0,
                confidence=1.0
            )

        # 시간 간격
        dt = time - self._last_time
        self._last_time = time

        if dt > 0:
            # 먼저 예측
            self.predict(dt)

        # 칼만 게인
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 잔차
        y = z - self.H @ self.x

        # 상태 업데이트
        self.x = self.x + K @ y

        # 공분산 업데이트
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

        return KalmanState(
            x=float(self.x[0]),
            z=float(self.x[1]),
            vx=float(self.x[2]),
            vz=float(self.x[3]),
            confidence=1.0
        )

    def get_state(self) -> KalmanState:
        """현재 상태 반환"""
        trace = np.trace(self.P[:2, :2])
        confidence = max(0.0, 1.0 - trace * 0.1) if self._initialized else 0.0

        return KalmanState(
            x=float(self.x[0]),
            z=float(self.x[1]),
            vx=float(self.x[2]),
            vz=float(self.x[3]),
            confidence=confidence
        )

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def velocity(self) -> Tuple[float, float]:
        """추정된 속도 (vx, vz)"""
        return float(self.x[2]), float(self.x[3])


class SimpleKalman1D:
    """간단한 1D Kalman Filter (각도, 오프셋 등에 사용)"""

    def __init__(self, q: float = 0.01, r: float = 0.1):
        self.q = q  # 프로세스 노이즈
        self.r = r  # 측정 노이즈
        self.x = 0.0  # 상태
        self.p = 1.0  # 공분산
        self._initialized = False

    def update(self, z: float) -> float:
        """측정값으로 업데이트"""
        if not self._initialized:
            self.x = z
            self._initialized = True
            return self.x

        # 예측
        self.p += self.q

        # 업데이트
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)

        return self.x

    def predict(self) -> float:
        """상태 예측 (변화 없음 가정)"""
        self.p += self.q
        return self.x

    def reset(self, value: float = 0.0):
        """리셋"""
        self.x = value
        self.p = 1.0
        self._initialized = True

    @property
    def value(self) -> float:
        return self.x
