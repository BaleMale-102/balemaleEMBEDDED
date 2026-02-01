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
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 검출 파라미터 최적화
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

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

        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(gray)

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

            # 마커의 yaw (회전 행렬에서 추출)
            yaw = self._extract_yaw(rvec)

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
        """회전 벡터에서 yaw 추출"""
        R, _ = cv2.Rodrigues(rvec)

        # Yaw (Z축 회전)
        yaw = math.atan2(R[1, 0], R[0, 0])

        return yaw

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
