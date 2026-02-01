#!/usr/bin/env python3
"""
marker_pose_node.py (v2 - 기울어진 마커 인식 개선)

개선사항:
- DetectorParameters 튜닝 (기울어진 마커)
- Corner refinement (subpix)
- CLAHE 전처리 (조명 변화에 강함)
- 더 관대한 검출 파라미터

토픽:
  Subscribe:
    - /cam_front/image_raw (Image)
    - /cam_front/camera_info (CameraInfo) [선택]
  
  Publish:
    - /perception/marker_pose (PoseStamped)
    - /perception/marker_status (MarkerStatus)
"""

import math
import time
import threading
from typing import Optional, Tuple
import yaml

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge

from rc_interfaces.msg import MarkerStatus

import tf2_ros


class MarkerPoseNode(Node):
    def __init__(self):
        super().__init__('marker_pose_node')
        self.bridge = CvBridge()
        
        # ===== Parameters =====
        self.declare_parameter('image_topic', '/cam_front/image_raw')
        self.declare_parameter('camera_info_topic', '/cam_front/camera_info')
        self.declare_parameter('pose_topic', '/perception/marker_pose')
        self.declare_parameter('status_topic', '/perception/marker_status')
        
        self.declare_parameter('camera_calib_yaml', '')
        
        # ArUco 설정
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.04)
        self.declare_parameter('marker_size_road', 0.04)
        self.declare_parameter('marker_size_slot', 0.02)
        self.declare_parameter('road_marker_max_id', 15)
        
        # EMA smoothing
        self.declare_parameter('ema_alpha', 0.5)
        
        # 품질 계산
        self.declare_parameter('quality_area_min', 200.0)
        self.declare_parameter('quality_area_max', 50000.0)
        
        self.declare_parameter('frame_id', 'camera_front')
        self.declare_parameter('publish_tf', True)
        
        self.declare_parameter('process_rate_hz', 30.0)
        self.declare_parameter('show_debug', False)
        
        # Load params
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_size_road = self.get_parameter('marker_size_road').value
        self.marker_size_slot = self.get_parameter('marker_size_slot').value
        self.road_max_id = self.get_parameter('road_marker_max_id').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.area_min = self.get_parameter('quality_area_min').value
        self.area_max = self.get_parameter('quality_area_max').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.show_debug = self.get_parameter('show_debug').value
        
        # ArUco detector with tuned parameters
        dict_name = self.get_parameter('aruco_dict').value
        aruco_dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        
        # ===== 파라미터 튜닝 - 기울어진 마커 인식 개선 =====
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Adaptive threshold - 조명 변화에 강하게
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 30
        self.aruco_params.adaptiveThreshWinSizeStep = 5
        self.aruco_params.adaptiveThreshConstant = 7
        
        # 마커 크기 범위 확장 (멀리/가까이 모두)
        self.aruco_params.minMarkerPerimeterRate = 0.01  # 더 작은 마커도 검출
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        
        # 기울어진 마커 인식 개선
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.1
        self.aruco_params.perspectiveRemovePixelPerCell = 8
        
        # 코너 검출 관대하게
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.minCornerDistanceRate = 0.02
        self.aruco_params.minDistanceToBorder = 2
        
        # 에러 보정 관대하게
        self.aruco_params.errorCorrectionRate = 0.9
        
        # Corner refinement (subpixel 정확도)
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 50
        self.aruco_params.cornerRefinementMinAccuracy = 0.01
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera intrinsics
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._load_camera_calib()
        
        # State
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        
        # EMA state
        self._ema_pose = {}
        
        # CLAHE for contrast enhancement
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishers
        self.pub_pose = self.create_publisher(PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.pub_status = self.create_publisher(MarkerStatus, self.get_parameter('status_topic').value, 10)
        
        # Subscribers
        self.sub_img = self.create_subscription(Image, self.get_parameter('image_topic').value, self.cb_image, qos_profile_sensor_data)
        self.sub_info = self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.cb_camera_info, 10)
        
        # Timer
        period = 1.0 / max(1.0, self.get_parameter('process_rate_hz').value)
        self.timer = self.create_timer(period, self.process_frame)
        
        self.get_logger().info(f"marker_pose_node v2 started (improved detection)")
    
    def _load_camera_calib(self):
        path = self.get_parameter('camera_calib_yaml').value
        if not path:
            self.camera_matrix = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
            self.get_logger().warn("Using default camera intrinsics")
            return
        
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(data['distortion_coefficients']['data'], dtype=np.float64)
            self.get_logger().info(f"Loaded camera calib: {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load camera calib: {e}")
            self.camera_matrix = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
    
    def cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                self._frame = frame
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
    
    def cb_camera_info(self, msg: CameraInfo):
        if self.camera_matrix is None or np.allclose(self.camera_matrix[0, 0], 500):
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if len(msg.d) > 0:
                self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(f"Got camera info: fx={msg.k[0]:.0f}")
    
    def process_frame(self):
        with self._lock:
            if self._frame is None:
                self._publish_invalid()
                return
            frame = self._frame.copy()
        
        if self.camera_matrix is None:
            self._publish_invalid()
            return
        
        # 전처리: Grayscale + CLAHE
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray)
        
        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is None or len(ids) == 0:
            self._publish_invalid()
            if self.show_debug:
                cv2.putText(frame, "No markers", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("marker_pose", frame)
                cv2.waitKey(1)
            return
        
        # 가장 큰 마커 선택
        best_idx = 0
        best_area = 0
        for i, corner in enumerate(corners):
            area = cv2.contourArea(corner)
            if area > best_area:
                best_area = area
                best_idx = i
        
        marker_id = int(ids[best_idx][0])
        marker_corners = corners[best_idx]
        
        # 마커 크기
        marker_size = self.marker_size_road if marker_id <= self.road_max_id else self.marker_size_slot
        
        # 포즈 추정
        rvec, tvec = self._estimate_pose(marker_corners, marker_size)
        
        if rvec is None:
            self._publish_invalid()
            return
        
        rel_x = float(tvec[0])
        rel_y = float(tvec[1])
        rel_z = float(tvec[2])
        
        # Yaw
        rmat, _ = cv2.Rodrigues(rvec)
        yaw = math.atan2(rmat[1, 0], rmat[0, 0])
        
        # EMA
        if marker_id in self._ema_pose:
            prev = self._ema_pose[marker_id]
            a = self.ema_alpha
            rel_x = a * rel_x + (1 - a) * prev[0]
            rel_y = a * rel_y + (1 - a) * prev[1]
            rel_z = a * rel_z + (1 - a) * prev[2]
            yaw = a * yaw + (1 - a) * prev[3]
        
        self._ema_pose[marker_id] = (rel_x, rel_y, rel_z, yaw)
        
        # 품질
        quality = (best_area - self.area_min) / (self.area_max - self.area_min)
        quality = max(0.0, min(1.0, quality))
        
        # 발행
        stamp = self.get_clock().now().to_msg()
        
        status = MarkerStatus()
        status.header.stamp = stamp
        status.header.frame_id = self.frame_id
        status.valid = True
        status.id = marker_id
        status.rel_x = float(rel_x)
        status.rel_y = float(rel_y)
        status.rel_z = float(rel_z)
        status.rel_yaw = float(yaw)
        status.quality = float(quality)
        self.pub_status.publish(status)
        
        pose = PoseStamped()
        pose.header = status.header
        pose.pose.position.x = rel_z
        pose.pose.position.y = -rel_x
        pose.pose.position.z = -rel_y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.pub_pose.publish(pose)
        
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.frame_id
            tf.child_frame_id = f"marker_{marker_id}"
            tf.transform.translation.x = pose.pose.position.x
            tf.transform.translation.y = pose.pose.position.y
            tf.transform.translation.z = pose.pose.position.z
            tf.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)
        
        if self.show_debug:
            debug = frame.copy()
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
            
            info = f"ID:{marker_id} z:{rel_z:.2f}m x:{rel_x:.3f}m q:{quality:.2f}"
            cv2.putText(debug, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug, f"Detected: {len(ids)}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            cv2.imshow("marker_pose", debug)
            cv2.waitKey(1)
    
    def _estimate_pose(self, corners, marker_size) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        obj_points = np.array([
            [-marker_size/2,  marker_size/2, 0],
            [ marker_size/2,  marker_size/2, 0],
            [ marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)
        
        img_points = corners.reshape(4, 2).astype(np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            return None, None
        
        return rvec.flatten(), tvec.flatten()
    
    def _publish_invalid(self):
        status = MarkerStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.frame_id
        status.valid = False
        status.id = -1
        status.quality = 0.0
        self.pub_status.publish(status)
    
    def destroy_node(self):
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = MarkerPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()