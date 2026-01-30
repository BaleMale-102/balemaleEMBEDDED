#!/usr/bin/env python3
"""
parking_line_node.py

주차선 인식 (Canny + HoughLines)

토픽:
  Subscribe:
    - /cam_side/image_raw (Image)
  
  Publish:
    - /perception/parking_line (ParkingLineStatus)
"""

import math
import time
import threading
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rc_interfaces.msg import ParkingLineStatus


class ParkingLineNode(Node):
    def __init__(self):
        super().__init__('parking_line_node')
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('image_topic', '/cam_side/image_raw')
        self.declare_parameter('output_topic', '/perception/parking_line')
        self.declare_parameter('frame_id', 'camera_side')
        
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_min_length', 50)
        self.declare_parameter('hough_max_gap', 10)
        
        self.declare_parameter('show_debug', False)
        self.declare_parameter('process_rate_hz', 20.0)
        
        # Load params
        self.frame_id = self.get_parameter('frame_id').value
        self.canny_low = self.get_parameter('canny_low').value
        self.canny_high = self.get_parameter('canny_high').value
        self.hough_thresh = self.get_parameter('hough_threshold').value
        self.hough_min = self.get_parameter('hough_min_length').value
        self.hough_gap = self.get_parameter('hough_max_gap').value
        self.show_debug = self.get_parameter('show_debug').value
        
        # State
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        
        # Publishers
        self.pub = self.create_publisher(
            ParkingLineStatus,
            self.get_parameter('output_topic').value,
            10
        )
        
        # Subscribers
        self.sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.cb_image,
            qos_profile_sensor_data
        )
        
        # Timer
        period = 1.0 / max(1.0, self.get_parameter('process_rate_hz').value)
        self.timer = self.create_timer(period, self.process_frame)
        
        self.get_logger().info("parking_line_node started")
    
    def cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._frame = frame
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
    
    def process_frame(self):
        with self._lock:
            if self._frame is None:
                self._publish_invalid()
                return
            frame = self._frame.copy()
        
        h, w = frame.shape[:2]
        
        # Grayscale + Blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)
        
        # HoughLinesP
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_thresh,
            minLineLength=self.hough_min,
            maxLineGap=self.hough_gap
        )
        
        if lines is None or len(lines) == 0:
            self._publish_invalid()
            if self.show_debug:
                cv2.imshow("parking_line", frame)
                cv2.waitKey(1)
            return
        
        # 수직에 가까운 라인 찾기 (주차선은 보통 세로)
        best_line = None
        best_score = 0
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            length = math.sqrt(dx*dx + dy*dy)
            
            # 수직도 (|dy| / length)
            verticality = abs(dy) / (length + 1e-6)
            score = length * verticality
            
            if score > best_score:
                best_score = score
                best_line = (x1, y1, x2, y2)
        
        if best_line is None:
            self._publish_invalid()
            return
        
        x1, y1, x2, y2 = best_line
        
        # Offset: 라인 중심과 이미지 중심 차이 (정규화)
        line_cx = (x1 + x2) / 2
        offset_px = line_cx - w / 2
        offset_norm = offset_px / (w / 2)
        offset_norm = max(-1.0, min(1.0, offset_norm))
        
        # Angle: 수직 기준 기울기
        angle = math.atan2(x2 - x1, y2 - y1)  # 수직 기준
        
        # Quality: 라인 길이 기반
        length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        quality = min(1.0, length / (h * 0.5))
        
        # 발행
        msg = ParkingLineStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.valid = True
        msg.offset_norm = float(offset_norm)
        msg.angle = float(angle)
        msg.quality = float(quality)
        self.pub.publish(msg)
        
        # Debug
        if self.show_debug:
            debug = frame.copy()
            cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(debug, (int(line_cx), (y1+y2)//2), 5, (0, 0, 255), -1)
            cv2.putText(debug, f"off:{offset_norm:.2f} ang:{math.degrees(angle):.1f}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("parking_line", debug)
            cv2.waitKey(1)
    
    def _publish_invalid(self):
        msg = ParkingLineStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.valid = False
        msg.quality = 0.0
        self.pub.publish(msg)
    
    def destroy_node(self):
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ParkingLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
