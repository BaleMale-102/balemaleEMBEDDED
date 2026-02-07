#!/usr/bin/env python3
"""
aruco_detector.py - ArUco 마커 검출 알고리즘

기능:
- OpenCV ArUco 마커 검출
- 포즈 추정 (solvePnP)
- 카메라→로봇 좌표 변환
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
import math


@dataclass
class DetectedMarker:
    """검출된 마커 정보"""
    id: int
    corners: np.ndarray          # 4개 코너 픽셀 좌표 (4, 2)
    tvec: np.ndarray             # 변환 벡터 (x, y, z) - 카메라 좌표계
    rvec: np.ndarray             # 회전 벡터
    distance: float              # 마커까지 거리 (m)
    angle: float                 # 마커 방향 (rad, 양수=왼쪽)
    yaw: float                   # 마커의 yaw (rad)
    confidence: float            # 검출 신뢰도 (0~1)


class ArucoDetector:
    """ArUco 마커 검출기"""

    # 지원되는 ArUco 딕셔너리
    DICT_MAP = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
        'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    }

    def __init__(
        self,
        dictionary: str = 'DICT_4X4_50',
        marker_size: float = 0.10,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None
    ):
        """
        Args:
            dictionary: ArUco 딕셔너리 이름
            marker_size: 마커 크기 (m)
            camera_matrix: 카메라 내부 파라미터 (3x3)
            dist_coeffs: 왜곡 계수 (5,)
        """
        # ArUco 딕셔너리
        if dictionary not in self.DICT_MAP:
            raise ValueError(f'Unknown dictionary: {dictionary}')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.DICT_MAP[dictionary])

        # OpenCV 버전에 따른 API 분기
        # 4.7+: DetectorParameters(), ArucoDetector()
        # 4.6-: DetectorParameters_create(), detectMarkers()
        self._use_new_api = hasattr(cv2.aruco, 'DetectorParameters')

        if self._use_new_api:
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_params.adaptiveThreshConstant = 7
            self.aruco_params.minMarkerPerimeterRate = 0.03
            self.aruco_params.maxMarkerPerimeterRate = 4.0
            self.aruco_params.polygonalApproxAccuracyRate = 0.03
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            # 구버전 OpenCV (4.6 이하)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_params.adaptiveThreshConstant = 7
            self.aruco_params.minMarkerPerimeterRate = 0.03
            self.aruco_params.maxMarkerPerimeterRate = 4.0
            self.aruco_params.polygonalApproxAccuracyRate = 0.03
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = None

        self.marker_size = marker_size

        # 카메라 파라미터 (기본값: 640x480 근사)
        if camera_matrix is not None:
            self.camera_matrix = camera_matrix
        else:
            self.camera_matrix = np.array([
                [640.0, 0.0, 320.0],
                [0.0, 640.0, 240.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)

        if dist_coeffs is not None:
            self.dist_coeffs = dist_coeffs
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float64)

    def set_camera_params(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """카메라 파라미터 설정"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def detect(self, image: np.ndarray) -> List[DetectedMarker]:
        """
        이미지에서 ArUco 마커 검출

        Args:
            image: BGR 이미지

        Returns:
            검출된 마커 리스트
        """
        # 그레이스케일 변환
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # 마커 검출 (OpenCV 버전에 따라 분기)
        if self._use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        markers = []

        if ids is None or len(ids) == 0:
            return markers

        # 포즈 추정
        for i, marker_id in enumerate(ids.flatten()):
            corner = corners[i][0]  # (4, 2)

            # solvePnP로 포즈 추정
            tvec, rvec = self._estimate_pose(corner)

            if tvec is None:
                continue

            # 거리 및 각도 계산
            distance = np.linalg.norm(tvec)
            angle = math.atan2(-tvec[0], tvec[2])  # 양수=왼쪽

            # 마커의 yaw (2D 회전 기반 - 카메라 pitch에 덜 민감)
            # 카메라가 비스듬히 아래를 봐도 마커의 이미지 상 기울기로 heading 판단
            yaw = self._compute_2d_rotation(corner)

            # 신뢰도 계산 (마커 크기 기반)
            perimeter = cv2.arcLength(corner, True)
            confidence = min(1.0, perimeter / 200.0)  # 정규화

            marker = DetectedMarker(
                id=int(marker_id),
                corners=corner,
                tvec=tvec,
                rvec=rvec,
                distance=distance,
                angle=angle,
                yaw=yaw,
                confidence=confidence
            )
            markers.append(marker)

        return markers

    def _estimate_pose(self, corners: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """solvePnP로 마커 포즈 추정"""
        # 마커의 3D 좌표 (마커 중심이 원점)
        half_size = self.marker_size / 2.0
        obj_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float64)

        # 이미지 좌표
        img_points = corners.astype(np.float64)

        # solvePnP
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            return None, None

        return tvec.flatten(), rvec.flatten()

    def _extract_yaw(self, rvec: np.ndarray) -> float:
        """회전 벡터에서 yaw 추출 (레거시 - 3D pose 기반)"""
        R, _ = cv2.Rodrigues(rvec)

        # Yaw (Z축 회전)
        yaw = math.atan2(R[1, 0], R[0, 0])

        return yaw

    def _compute_2d_rotation(self, corners: np.ndarray) -> float:
        """마커 코너에서 2D 회전 각도 계산 (이미지 평면 기준)

        카메라가 비스듬히 내려다봐도 이 값은 영향을 덜 받음.
        마커가 이미지에서 수평이면 0, 왼쪽으로 기울면 양수.

        Args:
            corners: 마커의 4개 코너 (4, 2) - [top-left, top-right, bottom-right, bottom-left]

        Returns:
            회전 각도 (rad) - 양수=CCW (마커가 왼쪽으로 기울어짐 = 로봇이 CW로 돌아있음)
        """
        # 상단 두 코너를 이용 (top-left, top-right)
        top_left = corners[0]
        top_right = corners[1]

        # 상단 가장자리의 기울기 (이미지 좌표계: Y 아래로 증가)
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]

        # 회전 각도: 수평 기준
        # 마커가 정면이면 dx > 0, dy ≈ 0 → rotation ≈ 0
        # 마커가 왼쪽으로 기울면 (로봇이 CW) → dy > 0 → rotation > 0
        rotation = math.atan2(dy, dx)

        return rotation

    def draw_markers(
        self,
        image: np.ndarray,
        markers: List[DetectedMarker],
        draw_axes: bool = True
    ) -> np.ndarray:
        """이미지에 마커 시각화"""
        output = image.copy()

        for marker in markers:
            # 코너 그리기
            corners = marker.corners.astype(np.int32)
            cv2.polylines(output, [corners], True, (0, 255, 0), 2)

            # ID 표시
            center = corners.mean(axis=0).astype(int)
            cv2.putText(
                output, f'ID:{marker.id}',
                (center[0] - 20, center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

            # 거리 표시
            cv2.putText(
                output, f'{marker.distance:.2f}m',
                (center[0] - 20, center[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1
            )

            # 좌표축 그리기
            if draw_axes:
                try:
                    cv2.drawFrameAxes(
                        output, self.camera_matrix, self.dist_coeffs,
                        marker.rvec, marker.tvec, self.marker_size * 0.5
                    )
                except Exception:
                    pass

        return output
