#!/usr/bin/env python3
"""
lane_detector.py - 라인 검출 알고리즘

기능:
- 이미지에서 바닥 라인 검출
- Sobel 엣지 + 적응형 임계값
- 라인 중심 오프셋 및 각도 계산
"""

import cv2
import numpy as np
import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class LaneDetectionResult:
    """라인 검출 결과"""
    valid: bool = False
    offset: float = 0.0           # 오프셋 (m, 양수=오른쪽)
    offset_normalized: float = 0.0  # 정규화 오프셋 (-1~+1)
    angle: float = 0.0            # 라인 각도 (rad)
    curvature: float = 0.0        # 곡률
    confidence: float = 0.0       # 신뢰도 (0~1)
    debug_image: Optional[np.ndarray] = None


class LaneDetector:
    """바닥 라인 검출기"""

    def __init__(
        self,
        roi_top_ratio: float = 0.5,
        roi_bottom_ratio: float = 1.0,
        sobel_threshold: int = 50,
        min_line_pixels: int = 100,
        line_color: str = 'black',
        image_width_m: float = 0.3  # 이미지가 커버하는 실제 너비 (m)
    ):
        """
        Args:
            roi_top_ratio: ROI 상단 비율 (0~1)
            roi_bottom_ratio: ROI 하단 비율 (0~1)
            sobel_threshold: Sobel 엣지 임계값
            min_line_pixels: 최소 라인 픽셀 수
            line_color: 라인 색상 ('black' or 'white')
            image_width_m: 이미지가 커버하는 실제 너비
        """
        self.roi_top_ratio = roi_top_ratio
        self.roi_bottom_ratio = roi_bottom_ratio
        self.sobel_threshold = sobel_threshold
        self.min_line_pixels = min_line_pixels
        self.line_color = line_color
        self.image_width_m = image_width_m

        # CLAHE for contrast enhancement
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def detect(self, image: np.ndarray, generate_debug: bool = False) -> LaneDetectionResult:
        """
        이미지에서 라인 검출

        Args:
            image: BGR 이미지
            generate_debug: 디버그 이미지 생성 여부

        Returns:
            검출 결과
        """
        h, w = image.shape[:2]

        # ROI 추출
        roi_top = int(h * self.roi_top_ratio)
        roi_bottom = int(h * self.roi_bottom_ratio)
        roi = image[roi_top:roi_bottom, :]

        # 전처리
        binary = self._preprocess(roi)

        # 라인 검출
        offset_px, angle, confidence, n_pixels = self._find_line(binary)

        # 유효성 체크
        valid = n_pixels >= self.min_line_pixels and confidence > 0.2

        # 픽셀 → 실제 단위 변환
        offset_normalized = offset_px / (w / 2) if w > 0 else 0.0
        offset_m = offset_normalized * (self.image_width_m / 2)

        # 디버그 이미지
        debug_image = None
        if generate_debug:
            debug_image = self._generate_debug_image(roi, binary, offset_px, angle, valid)

        return LaneDetectionResult(
            valid=valid,
            offset=offset_m,
            offset_normalized=offset_normalized,
            angle=angle,
            curvature=0.0,  # 간단 구현에서는 곡률 계산 생략
            confidence=confidence,
            debug_image=debug_image
        )

    def _preprocess(self, roi: np.ndarray) -> np.ndarray:
        """이미지 전처리"""
        # 그레이스케일
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi.copy()

        # CLAHE 적용
        enhanced = self.clahe.apply(gray)

        # 가우시안 블러
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)

        # 적응형 임계값 또는 Sobel
        if self.line_color == 'black':
            # 검은 라인: 적응형 임계값 (더 엄격하게)
            binary = cv2.adaptiveThreshold(
                blurred, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV,
                31, 15  # blockSize 증가, C 증가 → 더 어두운 것만 검출
            )
        else:
            # 흰 라인: Sobel 엣지
            sobel = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
            abs_sobel = np.abs(sobel)
            binary = (abs_sobel > self.sobel_threshold).astype(np.uint8) * 255

        # 모폴로지 연산 (노이즈 제거 강화)
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_large)   # 작은 노이즈 제거
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_small)  # 라인 연결

        return binary

    def _find_line(self, binary: np.ndarray) -> Tuple[float, float, float, int]:
        """
        이진 이미지에서 라인 찾기

        Returns:
            offset_px: 중심 오프셋 (픽셀, 양수=오른쪽)
            angle: 라인 각도 (rad)
            confidence: 신뢰도
            n_pixels: 라인 픽셀 수
        """
        h, w = binary.shape
        total_pixels = h * w

        # 라인 픽셀 찾기
        nonzero = binary.nonzero()
        line_y = np.array(nonzero[0])
        line_x = np.array(nonzero[1])

        n_pixels = len(line_x)

        if n_pixels < self.min_line_pixels:
            return 0.0, 0.0, 0.0, n_pixels

        # 픽셀 비율 체크 (너무 많으면 노이즈, 너무 적으면 라인 없음)
        pixel_ratio = n_pixels / total_pixels
        if pixel_ratio > 0.5 or pixel_ratio < 0.005:
            # 50% 이상이면 전체가 검정 또는 노이즈, 0.5% 미만이면 라인 없음
            return 0.0, 0.0, 0.0, 0

        # 라인 중심 (x 평균)
        line_center_x = np.mean(line_x)
        image_center_x = w / 2

        # 오프셋 (양수 = 라인이 오른쪽에 있음)
        offset_px = line_center_x - image_center_x

        # 라인 각도 및 직선성 체크 (선형 회귀)
        angle = 0.0
        linearity = 0.0

        if n_pixels > 200:
            try:
                # y에 대한 x의 기울기
                fit = np.polyfit(line_y, line_x, 1)
                angle = math.atan(fit[0])

                # 직선성 체크: R² 값 (잔차 기반)
                predicted_x = np.polyval(fit, line_y)
                ss_res = np.sum((line_x - predicted_x) ** 2)
                ss_tot = np.sum((line_x - np.mean(line_x)) ** 2)
                if ss_tot > 0:
                    r_squared = 1 - (ss_res / ss_tot)
                    linearity = max(0, r_squared)
            except Exception:
                pass

        # 신뢰도 계산 (픽셀 수 + 직선성)
        pixel_confidence = min(1.0, n_pixels / 2000.0)
        confidence = pixel_confidence * (0.5 + 0.5 * linearity)

        # 직선성이 낮으면 라인이 아닐 가능성 높음
        if linearity < 0.2 and n_pixels > 1000:
            # 많은 픽셀인데 직선이 아니면 노이즈
            confidence *= 0.5

        return offset_px, angle, confidence, n_pixels

    def _generate_debug_image(
        self,
        roi: np.ndarray,
        binary: np.ndarray,
        offset_px: float,
        angle: float,
        valid: bool
    ) -> np.ndarray:
        """디버그 이미지 생성"""
        # 컬러 이미지로 변환
        if len(roi.shape) == 2:
            debug = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        else:
            debug = roi.copy()

        h, w = debug.shape[:2]

        # 이진 이미지 오버레이
        binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        binary_color[:, :, 0] = 0  # Blue=0
        binary_color[:, :, 2] = 0  # Red=0
        debug = cv2.addWeighted(debug, 0.7, binary_color, 0.3, 0)

        # 중심선
        center_x = w // 2
        cv2.line(debug, (center_x, 0), (center_x, h), (0, 0, 255), 2)

        # 검출된 라인 위치
        if valid:
            line_x = int(center_x + offset_px)
            cv2.line(debug, (line_x, 0), (line_x, h), (0, 255, 0), 2)

        # 정보 표시
        color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.putText(debug, f'Offset: {offset_px:.1f}px', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(debug, f'Angle: {math.degrees(angle):.1f}deg', (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(debug, f'Valid: {valid}', (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        return debug
