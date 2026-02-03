#!/usr/bin/env python3
"""
rectangle_detector.py - Yellow Rectangle Detection

Detects yellow tape rectangles (parking slot boundaries) using HSV color filtering.
Returns rectangle center offset, angle, and dimensions.
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class RectangleResult:
    """Yellow rectangle detection result."""
    valid: bool = False
    center_x: float = 0.0        # Center X in image (pixels)
    center_y: float = 0.0        # Center Y in image (pixels)
    offset_x: float = 0.0        # Offset from image center (pixels)
    offset_y: float = 0.0        # Offset from image center (pixels)
    angle: float = 0.0           # Rotation angle (rad)
    width: float = 0.0           # Rectangle width (pixels)
    height: float = 0.0          # Rectangle height (pixels)
    confidence: float = 0.0      # Detection confidence (0-1)
    contour: Optional[np.ndarray] = None  # For debug visualization


class RectangleDetector:
    """HSV-based yellow rectangle detector."""

    def __init__(
        self,
        hsv_low: Tuple[int, int, int] = (20, 100, 100),
        hsv_high: Tuple[int, int, int] = (35, 255, 255),
        min_area: int = 500,
        max_area: int = 50000,
        min_aspect_ratio: float = 0.3,
        max_aspect_ratio: float = 3.0,
        blur_kernel: int = 5,
        morph_kernel: int = 5,
    ):
        """
        Initialize rectangle detector.

        Args:
            hsv_low: Lower HSV threshold for yellow (H, S, V)
            hsv_high: Upper HSV threshold for yellow (H, S, V)
            min_area: Minimum contour area (pixels^2)
            max_area: Maximum contour area (pixels^2)
            min_aspect_ratio: Minimum width/height ratio
            max_aspect_ratio: Maximum width/height ratio
            blur_kernel: Gaussian blur kernel size
            morph_kernel: Morphological operations kernel size
        """
        self.hsv_low = np.array(hsv_low)
        self.hsv_high = np.array(hsv_high)
        self.min_area = min_area
        self.max_area = max_area
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.blur_kernel = blur_kernel
        self.morph_kernel = morph_kernel

    def detect(self, image: np.ndarray) -> RectangleResult:
        """
        Detect yellow rectangle in image.

        Args:
            image: BGR image from camera

        Returns:
            RectangleResult with detection data
        """
        result = RectangleResult()

        if image is None or image.size == 0:
            return result

        h, w = image.shape[:2]
        img_center_x = w / 2
        img_center_y = h / 2

        # Preprocessing
        blurred = cv2.GaussianBlur(image, (self.blur_kernel, self.blur_kernel), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Yellow color mask
        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)

        # Morphological operations to clean up
        kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return result

        # Find best rectangle candidate
        best_contour = None
        best_score = 0.0
        best_rect = None

        for contour in contours:
            area = cv2.contourArea(contour)

            # Area filter
            if area < self.min_area or area > self.max_area:
                continue

            # Get minimum area rectangle
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), angle = rect

            # Ensure width > height for consistent angle
            if rh > rw:
                rw, rh = rh, rw
                angle = angle + 90

            # Aspect ratio filter
            if rh == 0:
                continue
            aspect_ratio = rw / rh
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue

            # Calculate rectangularity (how well it fits a rectangle)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            rect_area = rw * rh
            if rect_area == 0:
                continue
            rectangularity = area / rect_area

            # Score based on rectangularity and area
            score = rectangularity * (area / self.max_area)

            if score > best_score:
                best_score = score
                best_contour = contour
                best_rect = (cx, cy, rw, rh, angle)

        if best_rect is None:
            return result

        cx, cy, rw, rh, angle = best_rect

        # Convert angle to radians (OpenCV uses degrees, -90 to 0)
        angle_rad = np.deg2rad(angle)

        # Calculate offset from image center
        offset_x = cx - img_center_x
        offset_y = cy - img_center_y

        result.valid = True
        result.center_x = cx
        result.center_y = cy
        result.offset_x = offset_x
        result.offset_y = offset_y
        result.angle = angle_rad
        result.width = rw
        result.height = rh
        result.confidence = min(1.0, best_score * 2)  # Scale confidence
        result.contour = best_contour

        return result

    def draw_debug(self, image: np.ndarray, result: RectangleResult) -> np.ndarray:
        """
        Draw debug visualization on image.

        Args:
            image: Original BGR image
            result: Detection result

        Returns:
            Image with debug visualization
        """
        debug_img = image.copy()
        h, w = debug_img.shape[:2]

        # Draw image center crosshair
        cv2.line(debug_img, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (255, 255, 255), 1)
        cv2.line(debug_img, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (255, 255, 255), 1)

        if result.valid:
            # Draw detected rectangle
            rect = ((result.center_x, result.center_y),
                    (result.width, result.height),
                    np.rad2deg(result.angle))
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(debug_img, [box], 0, (0, 255, 0), 2)

            # Draw center point
            cx, cy = int(result.center_x), int(result.center_y)
            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)

            # Draw offset line
            cv2.line(debug_img, (w // 2, h // 2), (cx, cy), (0, 255, 255), 2)

            # Info text
            cv2.putText(debug_img, f"Offset: ({result.offset_x:.1f}, {result.offset_y:.1f})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_img, f"Angle: {np.rad2deg(result.angle):.1f} deg",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_img, f"Size: {result.width:.0f}x{result.height:.0f}",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_img, f"Conf: {result.confidence:.2f}",
                        (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(debug_img, "No rectangle detected",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return debug_img

    def get_mask(self, image: np.ndarray) -> np.ndarray:
        """
        Get yellow color mask for debugging.

        Args:
            image: BGR image

        Returns:
            Binary mask of yellow regions
        """
        blurred = cv2.GaussianBlur(image, (self.blur_kernel, self.blur_kernel), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)

        kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask
