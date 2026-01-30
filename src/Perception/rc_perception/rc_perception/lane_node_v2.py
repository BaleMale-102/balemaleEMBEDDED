#!/usr/bin/env python3
"""
lane_node_v2.py

Adaptive Threshold + Kalman Filter Lane Detection

Algorithm:
1. ROI extraction (exclude top portion)
2. Grayscale conversion
3. Adaptive Threshold (handles lighting variations)
4. Morphology (connect broken lines)
5. Sliding Window tracking
6. Kalman Filter smoothing

Topics:
  Subscribe:
    - /cam_bottom/image_raw (Image)

  Publish:
    - /perception/lane (LaneStatus)
    - /perception/lane/debug (Image) [optional]
"""

import math
import time
import threading
from typing import Optional, Tuple, List

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rc_interfaces.msg import LaneStatus


class KalmanFilter1D:
    """1D Kalman Filter for smoothing lane offset"""

    def __init__(self, q: float = 0.005, r: float = 0.05):
        self.q = q  # process noise
        self.r = r  # measurement noise
        self.x = 0.0  # state estimate
        self.p = 1.0  # error covariance
        self.initialized = False

    def update(self, z: float) -> float:
        if not self.initialized:
            self.x = z
            self.initialized = True
            return self.x

        # Predict
        self.p = self.p + self.q

        # Update
        k = self.p / (self.p + self.r)  # Kalman gain
        self.x = self.x + k * (z - self.x)
        self.p = (1 - k) * self.p

        return self.x

    def reset(self):
        self.x = 0.0
        self.p = 1.0
        self.initialized = False


class SlidingWindowTracker:
    """Sliding window lane tracker"""

    def __init__(self, n_windows: int = 9, margin: int = 50, min_pixels: int = 30):
        self.n_windows = n_windows
        self.margin = margin
        self.min_pixels = min_pixels
        self.last_base_x = None

    def find_lane(self, binary: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[float], float]:
        """
        Find lane using sliding window.

        Returns:
            lane_points: Array of (x, y) lane center points
            avg_x: Average x position (normalized -1 to 1)
            quality: Detection quality (0 to 1)
        """
        h, w = binary.shape

        # Histogram of bottom half
        histogram = np.sum(binary[h//2:, :], axis=0)

        # Find peak (lane position)
        if np.max(histogram) < self.min_pixels:
            return None, None, 0.0

        # Use previous position or find max
        if self.last_base_x is not None and 0 < self.last_base_x < w:
            # Search around previous position
            search_margin = self.margin * 2
            left = max(0, int(self.last_base_x - search_margin))
            right = min(w, int(self.last_base_x + search_margin))
            if np.max(histogram[left:right]) > self.min_pixels:
                base_x = left + np.argmax(histogram[left:right])
            else:
                base_x = np.argmax(histogram)
        else:
            base_x = np.argmax(histogram)

        # Sliding windows
        window_height = h // self.n_windows
        lane_points = []
        current_x = base_x
        total_pixels = 0

        for win_idx in range(self.n_windows):
            win_y_low = h - (win_idx + 1) * window_height
            win_y_high = h - win_idx * window_height
            win_x_low = max(0, current_x - self.margin)
            win_x_high = min(w, current_x + self.margin)

            # Find pixels in window
            nonzero = binary[win_y_low:win_y_high, win_x_low:win_x_high].nonzero()

            if len(nonzero[0]) > 0:
                # Get x positions (relative to window)
                good_x = nonzero[1] + win_x_low
                good_y = nonzero[0] + win_y_low

                # Update center
                if len(good_x) >= self.min_pixels:
                    current_x = int(np.mean(good_x))

                total_pixels += len(good_x)
                lane_points.append((current_x, (win_y_low + win_y_high) // 2))

        if len(lane_points) < 3:
            return None, None, 0.0

        # Calculate average x and quality
        avg_x = np.mean([p[0] for p in lane_points])
        avg_x_norm = (avg_x - w / 2) / (w / 2)  # -1 to 1
        avg_x_norm = max(-1.0, min(1.0, avg_x_norm))

        # Quality based on number of detected windows and pixels
        quality = min(1.0, len(lane_points) / self.n_windows)
        quality *= min(1.0, total_pixels / (self.n_windows * self.min_pixels * 3))

        self.last_base_x = avg_x

        return np.array(lane_points), avg_x_norm, quality

    def reset(self):
        self.last_base_x = None


class LaneNodeV2(Node):
    def __init__(self):
        super().__init__('lane_node')
        self.bridge = CvBridge()

        # ===== Parameters =====
        # Topics
        self.declare_parameter('image_topic', '/cam_bottom/image_raw')
        self.declare_parameter('output_topic', '/perception/lane')
        self.declare_parameter('debug_topic', '/perception/lane/debug')

        # Debug
        self.declare_parameter('show_debug', False)
        self.declare_parameter('publish_debug_image', False)

        # ROI
        self.declare_parameter('roi_top_ratio', 0.3)
        self.declare_parameter('roi_bottom_ratio', 1.0)

        # Line color
        self.declare_parameter('line_color', 'black')  # 'black' or 'white'

        # Adaptive threshold
        self.declare_parameter('use_adaptive', True)
        self.declare_parameter('adaptive_block_size', 25)
        self.declare_parameter('adaptive_c', 10)

        # Fixed threshold (when use_adaptive=false)
        self.declare_parameter('fixed_thresh_low', 0)
        self.declare_parameter('fixed_thresh_high', 60)

        # Morphology
        self.declare_parameter('morph_kernel_w', 5)
        self.declare_parameter('morph_kernel_h', 15)
        self.declare_parameter('morph_close_iter', 2)
        self.declare_parameter('morph_open_iter', 1)

        # Sliding window
        self.declare_parameter('sw_n_windows', 9)
        self.declare_parameter('sw_margin', 50)
        self.declare_parameter('sw_min_pixels', 30)

        # Kalman filter
        self.declare_parameter('kalman_q', 0.005)
        self.declare_parameter('kalman_r', 0.05)

        # Output
        self.declare_parameter('angle_max_deg', 45.0)
        self.declare_parameter('min_quality', 0.1)
        self.declare_parameter('frame_id', 'camera_bottom')

        # Processing
        self.declare_parameter('process_rate_hz', 30.0)
        self.declare_parameter('log_throttle_sec', 2.0)

        # Load parameters
        self._load_params()

        # State
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._last_log_t = 0.0

        # Lane tracker and filter
        self.tracker = SlidingWindowTracker(
            n_windows=self.sw_n_windows,
            margin=self.sw_margin,
            min_pixels=self.sw_min_pixels
        )
        self.kalman_offset = KalmanFilter1D(q=self.kalman_q, r=self.kalman_r)
        self.kalman_angle = KalmanFilter1D(q=self.kalman_q, r=self.kalman_r)

        # Publishers
        self.pub_lane = self.create_publisher(
            LaneStatus,
            self.get_parameter('output_topic').value,
            10
        )

        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image,
                self.get_parameter('debug_topic').value,
                1
            )

        # Subscriber
        self.sub_img = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.cb_image,
            qos_profile_sensor_data
        )

        # Timer
        period = 1.0 / max(1.0, self.get_parameter('process_rate_hz').value)
        self.timer = self.create_timer(period, self.process_frame)

        self.get_logger().info(
            f"lane_node_v2 started\n"
            f"  image_topic: {self.get_parameter('image_topic').value}\n"
            f"  output_topic: {self.get_parameter('output_topic').value}\n"
            f"  line_color: {self.line_color}\n"
            f"  use_adaptive: {self.use_adaptive}"
        )

    def _load_params(self):
        """Load parameters from ROS"""
        self.show_debug = self.get_parameter('show_debug').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        self.roi_top = self.get_parameter('roi_top_ratio').value
        self.roi_bottom = self.get_parameter('roi_bottom_ratio').value

        self.line_color = self.get_parameter('line_color').value

        self.use_adaptive = self.get_parameter('use_adaptive').value
        self.adaptive_block = self.get_parameter('adaptive_block_size').value
        self.adaptive_c = self.get_parameter('adaptive_c').value

        self.thresh_low = self.get_parameter('fixed_thresh_low').value
        self.thresh_high = self.get_parameter('fixed_thresh_high').value

        self.morph_kernel_w = self.get_parameter('morph_kernel_w').value
        self.morph_kernel_h = self.get_parameter('morph_kernel_h').value
        self.morph_close_iter = self.get_parameter('morph_close_iter').value
        self.morph_open_iter = self.get_parameter('morph_open_iter').value

        self.sw_n_windows = self.get_parameter('sw_n_windows').value
        self.sw_margin = self.get_parameter('sw_margin').value
        self.sw_min_pixels = self.get_parameter('sw_min_pixels').value

        self.kalman_q = self.get_parameter('kalman_q').value
        self.kalman_r = self.get_parameter('kalman_r').value

        self.angle_max = math.radians(self.get_parameter('angle_max_deg').value)
        self.min_quality = self.get_parameter('min_quality').value
        self.frame_id = self.get_parameter('frame_id').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value

    def _log_throttle(self, msg: str):
        now = time.time()
        if now - self._last_log_t >= self.log_throttle:
            self._last_log_t = now
            self.get_logger().info(msg)

    def cb_image(self, msg: Image):
        """Image callback"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._frame = frame
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")

    def process_frame(self):
        """Main processing loop"""
        with self._lock:
            if self._frame is None:
                self._publish_invalid()
                return
            frame = self._frame.copy()

        h, w = frame.shape[:2]

        # 1. ROI extraction
        roi_y_start = int(h * self.roi_top)
        roi_y_end = int(h * self.roi_bottom)
        roi = frame[roi_y_start:roi_y_end, :]
        roi_h, roi_w = roi.shape[:2]

        # 2. Grayscale conversion
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # 3. Threshold (adaptive or fixed)
        if self.use_adaptive:
            binary = self._adaptive_threshold(gray)
        else:
            binary = self._fixed_threshold(gray)

        # Invert for black lines
        if self.line_color == 'black':
            binary = cv2.bitwise_not(binary)

        # 4. Morphology (connect broken lines)
        binary = self._apply_morphology(binary)

        # 5. Sliding window tracking
        lane_points, offset_norm, quality = self.tracker.find_lane(binary)

        if lane_points is None or quality < self.min_quality:
            self._publish_invalid()
            if self.show_debug:
                self._show_debug(roi, binary, None, None, None)
            return

        # Calculate angle from lane points
        angle = self._calculate_angle(lane_points)

        # 6. Kalman filter smoothing
        offset_filtered = self.kalman_offset.update(offset_norm)
        angle_filtered = self.kalman_angle.update(angle)

        # Clamp angle
        angle_filtered = max(-self.angle_max, min(self.angle_max, angle_filtered))

        # Calculate pixel offset
        offset_px = offset_filtered * (roi_w / 2)

        # Publish result
        self._publish_lane(offset_px, offset_filtered, angle_filtered, quality)

        # Debug visualization
        if self.show_debug or self.publish_debug:
            self._show_debug(roi, binary, lane_points, offset_filtered, angle_filtered)

    def _adaptive_threshold(self, gray: np.ndarray) -> np.ndarray:
        """Apply adaptive threshold"""
        # Ensure block size is odd
        block_size = self.adaptive_block
        if block_size % 2 == 0:
            block_size += 1

        # Gaussian blur for noise reduction
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Adaptive threshold
        binary = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block_size,
            self.adaptive_c
        )

        return binary

    def _fixed_threshold(self, gray: np.ndarray) -> np.ndarray:
        """Apply fixed threshold"""
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blur, self.thresh_high, 255, cv2.THRESH_BINARY)
        return binary

    def _apply_morphology(self, binary: np.ndarray) -> np.ndarray:
        """Apply morphological operations to connect broken lines"""
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.morph_kernel_w, self.morph_kernel_h)
        )

        # Close (fill gaps)
        if self.morph_close_iter > 0:
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel,
                                      iterations=self.morph_close_iter)

        # Open (remove noise)
        if self.morph_open_iter > 0:
            kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_small,
                                      iterations=self.morph_open_iter)

        return binary

    def _calculate_angle(self, lane_points: np.ndarray) -> float:
        """Calculate lane angle from detected points using linear regression"""
        if len(lane_points) < 2:
            return 0.0

        x_coords = lane_points[:, 0]
        y_coords = lane_points[:, 1]

        # Fit line: x = a*y + b (since lane is mostly vertical)
        coeffs = np.polyfit(y_coords, x_coords, 1)
        slope = coeffs[0]  # dx/dy

        # Convert to angle (rad)
        # angle > 0 means lane curves right (counterclockwise rotation)
        angle = math.atan(slope)

        return angle

    def _publish_lane(self, offset_px: float, offset_norm: float,
                      angle: float, quality: float):
        """Publish lane status"""
        msg = LaneStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.in_lane = True
        msg.offset_px = float(offset_px)
        msg.offset_norm = float(offset_norm)
        msg.angle = float(angle)
        msg.quality = float(quality)
        self.pub_lane.publish(msg)

    def _publish_invalid(self):
        """Publish invalid lane status"""
        msg = LaneStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.in_lane = False
        msg.quality = 0.0
        self.pub_lane.publish(msg)

    def _show_debug(self, roi: np.ndarray, binary: np.ndarray,
                    lane_points: Optional[np.ndarray],
                    offset: Optional[float], angle: Optional[float]):
        """Show debug visualization"""
        debug = roi.copy()
        h, w = debug.shape[:2]

        # Draw center line
        cv2.line(debug, (w // 2, 0), (w // 2, h), (128, 128, 128), 1)

        # Draw lane points
        if lane_points is not None:
            for pt in lane_points:
                cv2.circle(debug, (int(pt[0]), int(pt[1])), 5, (0, 255, 0), -1)

            # Draw fitted line
            if len(lane_points) >= 2:
                cv2.polylines(debug, [lane_points.astype(np.int32)], False, (0, 255, 255), 2)

        # Draw detected offset
        if offset is not None:
            offset_x = int(w / 2 + offset * w / 2)
            cv2.line(debug, (offset_x, 0), (offset_x, h), (0, 0, 255), 2)

            # Text info
            text = f"off:{offset:.2f}"
            if angle is not None:
                text += f" ang:{math.degrees(angle):.1f}deg"
            cv2.putText(debug, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                       0.6, (255, 255, 0), 2)

        # Combine debug image with binary
        binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([debug, binary_color])

        if self.show_debug:
            cv2.imshow("lane_node_v2", combined)
            cv2.waitKey(1)

        if self.publish_debug:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
                self.pub_debug.publish(debug_msg)
            except Exception as e:
                self.get_logger().warn(f"Debug image publish error: {e}")

    def destroy_node(self):
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LaneNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
