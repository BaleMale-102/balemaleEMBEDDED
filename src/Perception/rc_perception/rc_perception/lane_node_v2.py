#!/usr/bin/env python3
"""
lane_node_v2.py - 차선 인식 노드

기능:
  - 하단 카메라로 검은색/흰색 차선 검출
  - Adaptive threshold + Sliding window
  - Kalman filter 스무딩
  - LaneStatus 메시지 발행
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rc_interfaces.msg import LaneStatus


class KalmanFilter1D:
    """1D Kalman Filter for smoothing"""
    def __init__(self, q=0.01, r=0.1):
        self.q = q  # process noise
        self.r = r  # measurement noise
        self.x = 0.0  # state
        self.p = 1.0  # covariance
        
    def update(self, z):
        # Prediction
        self.p += self.q
        # Update
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x
    
    def reset(self, value=0.0):
        self.x = value
        self.p = 1.0


class LaneNodeV2(Node):
    def __init__(self):
        super().__init__('lane_node_v2')
        
        # Parameters
        self.declare_parameter('image_topic', '/cam_bottom/image_raw')
        self.declare_parameter('output_topic', '/perception/lane')
        self.declare_parameter('show_debug', False)
        self.declare_parameter('publish_debug_image', False)
        
        # Lane detection params
        self.declare_parameter('line_color', 'black')  # black or white
        self.declare_parameter('use_adaptive', True)
        self.declare_parameter('adaptive_block_size', 25)
        self.declare_parameter('adaptive_c', 10)
        self.declare_parameter('roi_top_ratio', 0.3)  # ignore top 30%
        self.declare_parameter('roi_bottom_ratio', 1.0)
        
        # Sliding window params
        self.declare_parameter('sw_n_windows', 9)
        self.declare_parameter('sw_margin', 50)
        self.declare_parameter('sw_minpix', 30)
        
        # Kalman params
        self.declare_parameter('kalman_q', 0.005)
        self.declare_parameter('kalman_r', 0.05)
        
        # Get params
        self.image_topic = self.get_parameter('image_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.show_debug = self.get_parameter('show_debug').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        
        self.line_color = self.get_parameter('line_color').value
        self.use_adaptive = self.get_parameter('use_adaptive').value
        self.adaptive_block_size = self.get_parameter('adaptive_block_size').value
        self.adaptive_c = self.get_parameter('adaptive_c').value
        self.roi_top_ratio = self.get_parameter('roi_top_ratio').value
        self.roi_bottom_ratio = self.get_parameter('roi_bottom_ratio').value
        
        self.sw_n_windows = self.get_parameter('sw_n_windows').value
        self.sw_margin = self.get_parameter('sw_margin').value
        self.sw_minpix = self.get_parameter('sw_minpix').value
        
        kalman_q = self.get_parameter('kalman_q').value
        kalman_r = self.get_parameter('kalman_r').value
        
        # Kalman filters
        self.kf_offset = KalmanFilter1D(kalman_q, kalman_r)
        self.kf_angle = KalmanFilter1D(kalman_q, kalman_r)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.pub_lane = self.create_publisher(LaneStatus, self.output_topic, 10)
        
        if self.publish_debug_image:
            self.pub_debug = self.create_publisher(Image, '/perception/lane/debug', 10)
        
        # Subscriber
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)
        
        # State
        self.last_left_fit = None
        self.last_right_fit = None
        self.frame_count = 0
        
        self.get_logger().info(f'LaneNodeV2 started: {self.image_topic} -> {self.output_topic}')
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        h, w = frame.shape[:2]
        
        # ROI
        roi_top = int(h * self.roi_top_ratio)
        roi_bottom = int(h * self.roi_bottom_ratio)
        roi = frame[roi_top:roi_bottom, :]
        
        # Preprocess
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Threshold
        if self.use_adaptive:
            binary = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY_INV if self.line_color == 'black' else cv2.THRESH_BINARY,
                self.adaptive_block_size, self.adaptive_c)
        else:
            _, binary = cv2.threshold(gray, 127, 255, 
                cv2.THRESH_BINARY_INV if self.line_color == 'black' else cv2.THRESH_BINARY)
        
        # Morphology
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Find lane
        offset_norm, angle, quality, debug_img = self.find_lane(binary, roi)
        
        # Kalman filter
        if quality > 0.3:
            offset_filtered = self.kf_offset.update(offset_norm)
            angle_filtered = self.kf_angle.update(angle)
            valid = True
        else:
            offset_filtered = self.kf_offset.x
            angle_filtered = self.kf_angle.x
            valid = quality > 0.1
        
        # Publish
        lane_msg = LaneStatus()
        lane_msg.header.stamp = self.get_clock().now().to_msg()
        lane_msg.header.frame_id = 'cam_bottom'
        lane_msg.valid = valid
        lane_msg.offset_norm = float(offset_filtered)
        lane_msg.angle = float(angle_filtered)
        lane_msg.quality = float(quality)
        lane_msg.in_lane = abs(offset_filtered) < 0.8
        
        self.pub_lane.publish(lane_msg)
        
        # Debug display
        if self.show_debug and debug_img is not None:
            cv2.putText(debug_img, f'Offset: {offset_filtered:.2f}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_img, f'Angle: {angle_filtered:.2f}', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_img, f'Quality: {quality:.2f}', (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Lane Detection', debug_img)
            cv2.waitKey(1)
        
        if self.publish_debug_image and debug_img is not None:
            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
            except:
                pass
        
        self.frame_count += 1
    
    def find_lane(self, binary, roi):
        """Sliding window lane detection"""
        h, w = binary.shape
        
        # Histogram
        histogram = np.sum(binary[h//2:, :], axis=0)
        
        midpoint = w // 2
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # Sliding window
        n_windows = self.sw_n_windows
        window_height = h // n_windows
        margin = self.sw_margin
        minpix = self.sw_minpix
        
        # Find nonzero pixels
        nonzero = binary.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        
        left_current = left_base
        right_current = right_base
        
        left_lane_inds = []
        right_lane_inds = []
        
        debug_img = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB) if roi.ndim == 3 else cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        
        for window in range(n_windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            
            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin
            win_xright_low = right_current - margin
            win_xright_high = right_current + margin
            
            # Draw windows
            cv2.rectangle(debug_img, (win_xleft_low, win_y_low), 
                         (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(debug_img, (win_xright_low, win_y_low),
                         (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            # Find pixels in window
            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                             (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                              (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # Recenter
            if len(good_left_inds) > minpix:
                left_current = int(np.mean(nonzero_x[good_left_inds]))
            if len(good_right_inds) > minpix:
                right_current = int(np.mean(nonzero_x[good_right_inds]))
        
        # Concatenate
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except:
            left_lane_inds = np.array([])
            right_lane_inds = np.array([])
        
        # Calculate offset and angle
        left_count = len(left_lane_inds)
        right_count = len(right_lane_inds)
        total_count = left_count + right_count
        
        quality = min(1.0, total_count / 1000.0)
        
        if total_count < 100:
            return 0.0, 0.0, 0.0, debug_img
        
        # Lane center
        if left_count > 50 and right_count > 50:
            left_x = np.mean(nonzero_x[left_lane_inds])
            right_x = np.mean(nonzero_x[right_lane_inds])
            lane_center = (left_x + right_x) / 2
        elif left_count > 50:
            left_x = np.mean(nonzero_x[left_lane_inds])
            lane_center = left_x + w * 0.3  # assume right lane
        elif right_count > 50:
            right_x = np.mean(nonzero_x[right_lane_inds])
            lane_center = right_x - w * 0.3  # assume left lane
        else:
            return 0.0, 0.0, 0.0, debug_img
        
        # Offset (normalized: -1 = left, +1 = right)
        image_center = w / 2
        offset_pixels = lane_center - image_center
        offset_norm = offset_pixels / (w / 2)  # -1 to +1
        
        # Angle estimation (simple: difference between top and bottom lane positions)
        angle = 0.0
        if left_count > 100:
            left_y = nonzero_y[left_lane_inds]
            left_x = nonzero_x[left_lane_inds]
            try:
                fit = np.polyfit(left_y, left_x, 1)
                angle = -np.arctan(fit[0])  # slope to angle
            except:
                pass
        
        # Draw lane center
        cv2.line(debug_img, (int(lane_center), 0), (int(lane_center), h), (255, 0, 0), 2)
        cv2.line(debug_img, (int(image_center), 0), (int(image_center), h), (0, 0, 255), 2)
        
        return offset_norm, angle, quality, debug_img


def main(args=None):
    rclpy.init(args=args)
    node = LaneNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()