#!/usr/bin/env python3
"""
lane_node_v3.py - Front camera 차선 인식

front_cam에서 바닥 라인 검출:
- 화면 하단 ROI에서 검은 라인 검출
- offset: 차량 중심 대비 라인 위치
- angle: 라인 기울기

토픽:
  Subscribe: /cam_front/image_raw
  Publish: /perception/lane (LaneStatus)
"""

import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rc_interfaces.msg import LaneStatus


class KalmanFilter1D:
    """1D Kalman Filter for smoothing"""
    def __init__(self, q=0.01, r=0.1):
        self.q = q
        self.r = r
        self.x = 0.0
        self.p = 1.0
        
    def update(self, z):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x
    
    def reset(self, value=0.0):
        self.x = value
        self.p = 1.0


class LaneNodeV3(Node):
    def __init__(self):
        super().__init__('lane_node_v3')
        
        # Parameters
        self.declare_parameter('image_topic', '/cam_front/image_raw')
        self.declare_parameter('output_topic', '/perception/lane')
        self.declare_parameter('show_debug', False)
        
        # ROI - front cam에서 바닥 보이는 영역 (하단)
        self.declare_parameter('roi_top_ratio', 0.6)     # 상단 60% 무시
        self.declare_parameter('roi_bottom_ratio', 0.95) # 하단 5% 무시 (차체)
        
        # 라인 검출 파라미터
        self.declare_parameter('line_color', 'black')
        self.declare_parameter('adaptive_block_size', 25)
        self.declare_parameter('adaptive_c', 10)
        self.declare_parameter('min_line_pixels', 100)
        
        # Kalman
        self.declare_parameter('kalman_q', 0.01)
        self.declare_parameter('kalman_r', 0.05)
        
        # CLAHE
        self.declare_parameter('use_clahe', True)
        
        # Angle 보정 (직선일때 angle 오프셋)
        self.declare_parameter('angle_offset_deg', 0.0)
        
        # Deadzone (offset이 이 이하면 0으로 처리)
        self.declare_parameter('offset_deadzone', 0.1)  # 10% 이내면 중앙으로 간주
        
        # Load params
        self.image_topic = self.get_parameter('image_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.show_debug = self.get_parameter('show_debug').value
        
        self.roi_top_ratio = self.get_parameter('roi_top_ratio').value
        self.roi_bottom_ratio = self.get_parameter('roi_bottom_ratio').value
        
        self.line_color = self.get_parameter('line_color').value
        self.adaptive_block_size = self.get_parameter('adaptive_block_size').value
        self.adaptive_c = self.get_parameter('adaptive_c').value
        self.min_line_pixels = self.get_parameter('min_line_pixels').value
        
        self.use_clahe = self.get_parameter('use_clahe').value
        self.angle_offset = math.radians(self.get_parameter('angle_offset_deg').value)
        self.offset_deadzone = self.get_parameter('offset_deadzone').value
        
        kalman_q = self.get_parameter('kalman_q').value
        kalman_r = self.get_parameter('kalman_r').value
        
        # Kalman filters
        self.kf_offset = KalmanFilter1D(kalman_q, kalman_r)
        self.kf_angle = KalmanFilter1D(kalman_q, kalman_r)
        
        # CLAHE
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.pub_lane = self.create_publisher(LaneStatus, self.output_topic, 10)
        
        # Subscriber
        self.sub_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        
        self.get_logger().info(f'LaneNodeV3 (front_cam) started: {self.image_topic}')
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        try:
            h, w = frame.shape[:2]
            
            # ROI (하단 영역 - 바닥 라인)
            roi_top = int(h * self.roi_top_ratio)
            roi_bottom = int(h * self.roi_bottom_ratio)
            roi = frame[roi_top:roi_bottom, :]
            
            # Grayscale + CLAHE
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            if self.use_clahe:
                gray = self.clahe.apply(gray)
            
            # Adaptive threshold (검은 라인 검출)
            if self.line_color == 'black':
                binary = cv2.adaptiveThreshold(
                    gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY_INV,
                    self.adaptive_block_size, self.adaptive_c)
            else:
                binary = cv2.adaptiveThreshold(
                    gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY,
                    self.adaptive_block_size, self.adaptive_c)
            
            # Morphology - 노이즈 제거
            kernel = np.ones((3, 3), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            
            # 라인 검출
            offset_norm, angle, quality, in_lane = self.detect_line(binary, roi)
            
            # Kalman filter
            if quality > 0.3:
                offset_filtered = self.kf_offset.update(offset_norm)
                angle_filtered = self.kf_angle.update(angle - self.angle_offset)
            else:
                offset_filtered = self.kf_offset.x
                angle_filtered = self.kf_angle.x
            
            # Deadzone 적용 - 작은 오차는 0으로
            if abs(offset_filtered) < self.offset_deadzone:
                offset_filtered = 0.0
            if abs(angle_filtered) < math.radians(3.0):  # 3도 이내면 0
                angle_filtered = 0.0
            
            # Publish
            lane_msg = LaneStatus()
            lane_msg.header.stamp = self.get_clock().now().to_msg()
            lane_msg.header.frame_id = 'camera_front'
            lane_msg.offset_norm = float(offset_filtered)
            lane_msg.angle = float(angle_filtered)
            lane_msg.quality = float(quality)
            lane_msg.in_lane = bool(in_lane and quality > 0.2)
            
            self.pub_lane.publish(lane_msg)
            
            # Debug
            if self.show_debug:
                debug = roi.copy()
                
                # Binary overlay
                binary_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                debug = cv2.addWeighted(debug, 0.7, binary_color, 0.3, 0)
                
                # Center line
                center_x = w // 2
                cv2.line(debug, (center_x, 0), (center_x, debug.shape[0]), (0, 0, 255), 2)
                
                # Detected line position
                line_x = int(center_x + offset_filtered * (w / 2))
                cv2.line(debug, (line_x, 0), (line_x, debug.shape[0]), (0, 255, 0), 2)
                
                # Info
                cv2.putText(debug, f'Offset: {offset_filtered:.2f}', (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(debug, f'Angle: {math.degrees(angle_filtered):.1f}deg', (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(debug, f'Quality: {quality:.2f}', (10, 75),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(debug, f'In lane: {in_lane}', (10, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                cv2.imshow('Lane (front_cam)', debug)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().warn(f'Processing error: {e}')
    
    def detect_line(self, binary, roi):
        """
        라인 검출 - 중심선 기준 offset과 angle 계산
        
        Returns:
            offset_norm: -1(왼쪽) ~ +1(오른쪽)
            angle: 라인 기울기 (rad)
            quality: 검출 품질 0~1
            in_lane: 라인 위에 있는지
        """
        h, w = binary.shape
        
        # 라인 픽셀 찾기
        try:
            nonzero = binary.nonzero()
            line_y = np.array(nonzero[0])
            line_x = np.array(nonzero[1])
        except:
            return 0.0, 0.0, 0.0, False
        
        n_pixels = len(line_x)
        
        if n_pixels < self.min_line_pixels:
            return 0.0, 0.0, 0.0, False
        
        # Quality
        quality = min(1.0, n_pixels / 2000.0)
        
        # 라인 중심 (x 평균)
        try:
            line_center_x = np.mean(line_x)
        except:
            return 0.0, 0.0, 0.0, False
        
        # Offset (정규화)
        image_center_x = w / 2
        offset_pixels = line_center_x - image_center_x
        offset_norm = offset_pixels / (w / 2)  # -1 ~ +1
        
        # Angle (선형 회귀) - 수직 라인이면 0
        angle = 0.0
        if n_pixels > 200:
            try:
                # y에 대한 x의 기울기 (수직=0, 오른쪽 기울임=양수)
                fit = np.polyfit(line_y, line_x, 1)
                angle = math.atan(fit[0])
            except:
                pass
        
        # In lane: 라인이 중앙 근처에 있으면 True
        in_lane = abs(offset_norm) < 0.5 and quality > 0.3
        
        return offset_norm, angle, quality, in_lane
    
    def destroy_node(self):
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneNodeV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()