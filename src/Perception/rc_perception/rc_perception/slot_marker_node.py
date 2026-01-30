#!/usr/bin/env python3
"""
slot_marker_node.py

주차칸 ArUco 마커 인식 (ID 16-99, 2cm)

토픽:
  Subscribe:
    - /cam_side/image_raw (Image)
  
  Publish:
    - /perception/slot_marker_pose (PoseStamped)
"""

import math
import time
import threading
from typing import Optional
import yaml

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class SlotMarkerNode(Node):
    def __init__(self):
        super().__init__('slot_marker_node')
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('image_topic', '/cam_side/image_raw')
        self.declare_parameter('output_topic', '/perception/slot_marker_pose')
        self.declare_parameter('frame_id', 'camera_side')
        
        self.declare_parameter('camera_calib_yaml', '')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.02)  # 2cm
        self.declare_parameter('slot_marker_min_id', 16)
        self.declare_parameter('slot_marker_max_id', 99)
        
        self.declare_parameter('show_debug', False)
        self.declare_parameter('process_rate_hz', 20.0)
        
        # Load params
        self.frame_id = self.get_parameter('frame_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.min_id = self.get_parameter('slot_marker_min_id').value
        self.max_id = self.get_parameter('slot_marker_max_id').value
        self.show_debug = self.get_parameter('show_debug').value
        
        # ArUco
        dict_name = self.get_parameter('aruco_dict').value
        aruco_dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera intrinsics
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._load_camera_calib()
        
        # State
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        
        # Publishers
        self.pub = self.create_publisher(
            PoseStamped,
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
        
        self.get_logger().info(f"slot_marker_node started (ID {self.min_id}-{self.max_id})")
    
    def _load_camera_calib(self):
        path = self.get_parameter('camera_calib_yaml').value
        if not path:
            # 기본값
            self.camera_matrix = np.array([
                [600, 0, 320],
                [0, 600, 240],
                [0, 0, 1]
            ], dtype=np.float64)
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
            return
        
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(data['distortion_coefficients']['data'], dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f"Failed to load camera calib: {e}")
            self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
    
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
                return
            frame = self._frame.copy()
        
        if self.camera_matrix is None:
            return
        
        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(frame)
        
        if ids is None or len(ids) == 0:
            if self.show_debug:
                cv2.imshow("slot_marker", frame)
                cv2.waitKey(1)
            return
        
        # 슬롯 마커 필터 (ID 16-99)
        slot_indices = []
        for i, mid in enumerate(ids):
            if self.min_id <= mid[0] <= self.max_id:
                slot_indices.append(i)
        
        if not slot_indices:
            if self.show_debug:
                cv2.imshow("slot_marker", frame)
                cv2.waitKey(1)
            return
        
        # 첫 번째 슬롯 마커 사용
        idx = slot_indices[0]
        marker_corners = corners[idx]
        marker_id = int(ids[idx][0])
        
        # 포즈 추정
        obj_points = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)
        
        img_points = marker_corners.reshape(4, 2).astype(np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            return
        
        tvec = tvec.flatten()
        rvec = rvec.flatten()
        
        # ROS 좌표계로 변환
        # OpenCV: X-right, Y-down, Z-forward
        # ROS: X-forward, Y-left, Z-up
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(tvec[2])   # forward
        pose.pose.position.y = float(-tvec[0])  # left
        pose.pose.position.z = float(-tvec[1])  # up
        
        # Yaw
        rmat, _ = cv2.Rodrigues(rvec)
        yaw = math.atan2(rmat[1, 0], rmat[0, 0])
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.pub.publish(pose)
        
        # Debug
        if self.show_debug:
            debug = frame.copy()
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.02)
            cv2.putText(debug, f"Slot ID:{marker_id} x:{pose.pose.position.x:.3f}m",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("slot_marker", debug)
            cv2.waitKey(1)
    
    def destroy_node(self):
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = SlotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
