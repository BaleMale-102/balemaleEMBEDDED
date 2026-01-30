#!/usr/bin/env python3
"""
ekf_localization_node.py

Extended Kalman Filter 기반 위치 추정

센서 융합:
  - ArUco Marker: 절대 위치 + yaw (업데이트)
  - IMU: angular velocity (예측)
  - Wheel Odometry: linear/angular velocity (예측)

상태 벡터:
  x = [x, y, θ, vx, vy, ω]^T

토픽:
  Subscribe:
    - /perception/marker_status (MarkerStatus) - 마커 포즈
    - /imu/data (Imu) - IMU
    - /odom/wheel (Odometry) - 휠 오도메트리
  
  Publish:
    - /localization/pose (PoseStamped) - 추정 위치
    - /localization/odom (Odometry) - 추정 오도메트리 (covariance 포함)
    - /localization/fix_valid (Bool) - 마커 기반 업데이트 유효성
    - TF: map -> base_link (또는 odom)
"""

import math
import time
import threading
from typing import Optional, Dict
from dataclasses import dataclass

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from rc_interfaces.msg import MarkerStatus

import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


@dataclass
class MarkerMapEntry:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class EKFLocalization:
    """
    Extended Kalman Filter for 2D robot localization
    
    State: [x, y, θ, vx, vy, ω]
    """
    
    def __init__(self):
        # State vector [x, y, θ, vx, vy, ω]
        self.x = np.zeros(6)
        
        # State covariance
        self.P = np.eye(6) * 0.1
        
        # Process noise (튜닝 필요)
        self.Q = np.diag([
            0.01,   # x
            0.01,   # y
            0.005,  # θ
            0.05,   # vx
            0.05,   # vy
            0.02    # ω
        ])
        
        # Measurement noise - Marker (튜닝 필요)
        self.R_marker = np.diag([
            0.02,   # x
            0.02,   # y
            0.05    # θ
        ])
        
        # Measurement noise - Odometry velocity
        self.R_odom = np.diag([
            0.1,    # vx
            0.1,    # vy
            0.05    # ω
        ])
        
        self.initialized = False
    
    def initialize(self, x: float, y: float, yaw: float):
        """초기 위치 설정"""
        self.x = np.array([x, y, yaw, 0.0, 0.0, 0.0])
        self.P = np.eye(6) * 0.1
        self.initialized = True
    
    def predict(self, dt: float, imu_omega: Optional[float] = None):
        """
        예측 단계
        
        모션 모델:
          x' = x + vx*cos(θ)*dt - vy*sin(θ)*dt
          y' = y + vx*sin(θ)*dt + vy*cos(θ)*dt
          θ' = θ + ω*dt
          vx' = vx (constant velocity)
          vy' = vy
          ω' = ω or imu_omega
        """
        if not self.initialized or dt <= 0:
            return
        
        x, y, theta, vx, vy, omega = self.x
        
        # IMU omega 사용 시 대체
        if imu_omega is not None:
            omega = imu_omega
        
        c = math.cos(theta)
        s = math.sin(theta)
        
        # 상태 예측
        x_new = x + (vx * c - vy * s) * dt
        y_new = y + (vx * s + vy * c) * dt
        theta_new = wrap_angle(theta + omega * dt)
        
        self.x = np.array([x_new, y_new, theta_new, vx, vy, omega])
        
        # Jacobian F
        F = np.eye(6)
        F[0, 2] = (-vx * s - vy * c) * dt
        F[0, 3] = c * dt
        F[0, 4] = -s * dt
        F[1, 2] = (vx * c - vy * s) * dt
        F[1, 3] = s * dt
        F[1, 4] = c * dt
        F[2, 5] = dt
        
        # Covariance 예측
        self.P = F @ self.P @ F.T + self.Q * dt
    
    def update_marker(self, meas_x: float, meas_y: float, meas_yaw: float):
        """
        마커 측정으로 업데이트
        """
        if not self.initialized:
            self.initialize(meas_x, meas_y, meas_yaw)
            return
        
        # Measurement vector
        z = np.array([meas_x, meas_y, meas_yaw])
        
        # Predicted measurement
        h = np.array([self.x[0], self.x[1], self.x[2]])
        
        # Innovation
        y = z - h
        y[2] = wrap_angle(y[2])  # angle wrapping
        
        # Measurement Jacobian H
        H = np.zeros((3, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_marker
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        
        # Update covariance
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
    
    def update_odom_velocity(self, vx: float, vy: float, omega: float):
        """
        오도메트리 속도로 업데이트
        """
        if not self.initialized:
            return
        
        # Measurement vector (velocity)
        z = np.array([vx, vy, omega])
        
        # Predicted measurement
        h = np.array([self.x[3], self.x[4], self.x[5]])
        
        # Innovation
        y = z - h
        
        # Measurement Jacobian H
        H = np.zeros((3, 6))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        H[2, 5] = 1.0
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_odom
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
    
    def get_pose(self):
        """현재 추정 포즈 반환"""
        return self.x[0], self.x[1], self.x[2]
    
    def get_velocity(self):
        """현재 추정 속도 반환"""
        return self.x[3], self.x[4], self.x[5]
    
    def get_covariance_pose(self):
        """Position covariance (x, y, θ)"""
        return self.P[:3, :3].copy()


class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')
        
        # ===== Parameters =====
        self.declare_parameter('marker_status_topic', '/perception/marker_status')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom/wheel')
        
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('odom_out_topic', '/localization/odom')
        self.declare_parameter('fix_valid_topic', '/localization/fix_valid')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_map_to_odom', True)  # map->odom TF 발행
        
        self.declare_parameter('marker_map_yaml', '')
        self.declare_parameter('map_unit_scale', 0.01)  # cm -> m
        
        # 카메라 오프셋 (base_link -> camera_front)
        self.declare_parameter('cam_offset_x', 0.06)
        self.declare_parameter('cam_offset_y', 0.0)
        
        self.declare_parameter('min_marker_quality', 0.25)
        self.declare_parameter('marker_timeout_sec', 1.0)
        self.declare_parameter('imu_timeout_sec', 0.3)
        self.declare_parameter('odom_timeout_sec', 0.5)
        
        self.declare_parameter('predict_rate_hz', 50.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        
        self.declare_parameter('log_throttle_sec', 2.0)
        
        # Load params
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.tf_map_odom = self.get_parameter('tf_map_to_odom').value
        
        self.map_scale = self.get_parameter('map_unit_scale').value
        self.cam_offset_x = self.get_parameter('cam_offset_x').value
        self.cam_offset_y = self.get_parameter('cam_offset_y').value
        
        self.min_quality = self.get_parameter('min_marker_quality').value
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value
        self.imu_timeout = self.get_parameter('imu_timeout_sec').value
        self.odom_timeout = self.get_parameter('odom_timeout_sec').value
        
        self.log_throttle_sec = self.get_parameter('log_throttle_sec').value
        
        # State
        self._lock = threading.Lock()
        self._last_log_t = 0.0
        
        # EKF
        self.ekf = EKFLocalization()
        
        # Marker map
        self.marker_map: Dict[int, MarkerMapEntry] = {}
        self._load_marker_map()
        
        # Sensor timestamps
        self._last_marker_t = 0.0
        self._last_imu_t = 0.0
        self._last_odom_t = 0.0
        self._last_predict_t = time.time()
        
        # Latest sensor data
        self._latest_imu_omega: Optional[float] = None
        self._latest_odom_vx: Optional[float] = None
        self._latest_odom_vy: Optional[float] = None
        self._latest_odom_omega: Optional[float] = None
        
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Odom frame offset (map -> odom transform)
        self._odom_offset_x = 0.0
        self._odom_offset_y = 0.0
        self._odom_offset_yaw = 0.0
        
        # Publishers
        self.pub_pose = self.create_publisher(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            10
        )
        self.pub_odom = self.create_publisher(
            Odometry,
            self.get_parameter('odom_out_topic').value,
            10
        )
        self.pub_fix = self.create_publisher(
            Bool,
            self.get_parameter('fix_valid_topic').value,
            10
        )
        
        # Subscribers
        self.sub_marker = self.create_subscription(
            MarkerStatus,
            self.get_parameter('marker_status_topic').value,
            self.cb_marker,
            10
        )
        self.sub_imu = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.cb_imu,
            qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.cb_odom,
            10
        )
        
        # Timers
        predict_period = 1.0 / max(1.0, self.get_parameter('predict_rate_hz').value)
        publish_period = 1.0 / max(1.0, self.get_parameter('publish_rate_hz').value)
        
        self.timer_predict = self.create_timer(predict_period, self.predict_step)
        self.timer_publish = self.create_timer(publish_period, self.publish_state)
        
        self.get_logger().info(
            f"ekf_localization_node started\n"
            f"  marker_map entries: {len(self.marker_map)}\n"
            f"  map_frame: {self.map_frame}\n"
            f"  base_frame: {self.base_frame}"
        )
    
    def _load_marker_map(self):
        """marker_map.yaml 로드"""
        path = self.get_parameter('marker_map_yaml').value
        if not path:
            self.get_logger().warn("marker_map_yaml not specified")
            return
        
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            
            markers = data.get('markers', [])
            for m in markers:
                mid = m.get('id')
                if mid is None:
                    continue
                
                x = m.get('x')
                y = m.get('y')
                if x is None or y is None:
                    continue
                
                yaw = m.get('yaw', 0.0)
                
                self.marker_map[mid] = MarkerMapEntry(
                    x=float(x),
                    y=float(y),
                    yaw=float(yaw)
                )
            
            self.get_logger().info(f"Loaded marker_map: {path} ({len(self.marker_map)} entries)")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load marker_map: {e}")
    
    def _log_throttle(self, msg: str):
        now = time.time()
        if now - self._last_log_t >= self.log_throttle_sec:
            self._last_log_t = now
            self.get_logger().info(msg)
    
    # ===== Callbacks =====
    
    def cb_marker(self, msg: MarkerStatus):
        if not msg.valid:
            return
        
        if msg.quality < self.min_quality:
            return
        
        # marker_map에서 마커 위치 조회
        entry = self.marker_map.get(msg.id)
        if entry is None:
            self._log_throttle(f"Marker id={msg.id} not in map")
            return
        
        with self._lock:
            self._last_marker_t = time.time()
            
            # 마커 map 좌표 (cm -> m)
            marker_x = entry.x * self.map_scale
            marker_y = entry.y * self.map_scale
            marker_yaw = entry.yaw
            
            # 로봇 위치 계산
            # 마커를 보고 있으므로, 로봇은 마커 반대편에 있음
            # rel_z: 마커까지 거리, rel_x: 좌우 오프셋
            
            # 로봇 yaw 계산: 마커 방향 + π - 상대 yaw
            robot_yaw = wrap_angle(marker_yaw + math.pi - msg.rel_yaw)
            
            c = math.cos(robot_yaw)
            s = math.sin(robot_yaw)
            
            # 카메라→마커 벡터를 map 좌표계로 변환
            dx_map = c * msg.rel_z - s * msg.rel_x
            dy_map = s * msg.rel_z + c * msg.rel_x
            
            # 로봇 위치 = 마커 위치 - (카메라→마커 벡터)
            robot_x = marker_x - dx_map
            robot_y = marker_y - dy_map
            
            # 카메라 오프셋 보정
            robot_x -= c * self.cam_offset_x - s * self.cam_offset_y
            robot_y -= s * self.cam_offset_x + c * self.cam_offset_y
            
            # EKF 업데이트
            self.ekf.update_marker(robot_x, robot_y, robot_yaw)
            
            self._log_throttle(
                f"Marker update: id={msg.id} robot=[{robot_x:.3f}, {robot_y:.3f}, {math.degrees(robot_yaw):.1f}°]"
            )
    
    def cb_imu(self, msg: Imu):
        with self._lock:
            self._last_imu_t = time.time()
            self._latest_imu_omega = msg.angular_velocity.z
    
    def cb_odom(self, msg: Odometry):
        with self._lock:
            self._last_odom_t = time.time()
            self._latest_odom_vx = msg.twist.twist.linear.x
            self._latest_odom_vy = msg.twist.twist.linear.y
            self._latest_odom_omega = msg.twist.twist.angular.z
            
            # 속도 업데이트
            if self.ekf.initialized:
                self.ekf.update_odom_velocity(
                    self._latest_odom_vx,
                    self._latest_odom_vy,
                    self._latest_odom_omega
                )
    
    # ===== EKF Steps =====
    
    def predict_step(self):
        """예측 단계 (타이머)"""
        now = time.time()
        
        with self._lock:
            dt = now - self._last_predict_t
            self._last_predict_t = now
            
            if not self.ekf.initialized:
                return
            
            # IMU omega 사용 (유효한 경우)
            imu_omega = None
            if self._latest_imu_omega is not None:
                if (now - self._last_imu_t) < self.imu_timeout:
                    imu_omega = self._latest_imu_omega
            
            self.ekf.predict(dt, imu_omega)
    
    def publish_state(self):
        """상태 발행 (타이머)"""
        now = time.time()
        stamp = self.get_clock().now().to_msg()
        
        with self._lock:
            if not self.ekf.initialized:
                return
            
            x, y, yaw = self.ekf.get_pose()
            vx, vy, omega = self.ekf.get_velocity()
            cov = self.ekf.get_covariance_pose()
            
            # Fix valid 체크
            marker_age = now - self._last_marker_t
            fix_valid = marker_age < self.marker_timeout
        
        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(yaw)
        self.pub_pose.publish(pose_msg)
        
        # Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose
        
        # Covariance (6x6 row-major)
        odom_msg.pose.covariance[0] = cov[0, 0]   # xx
        odom_msg.pose.covariance[1] = cov[0, 1]   # xy
        odom_msg.pose.covariance[5] = cov[0, 2]   # xθ
        odom_msg.pose.covariance[6] = cov[1, 0]
        odom_msg.pose.covariance[7] = cov[1, 1]   # yy
        odom_msg.pose.covariance[11] = cov[1, 2]
        odom_msg.pose.covariance[30] = cov[2, 0]
        odom_msg.pose.covariance[31] = cov[2, 1]
        odom_msg.pose.covariance[35] = cov[2, 2]  # θθ
        
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        
        self.pub_odom.publish(odom_msg)
        
        # Fix valid
        fix_msg = Bool()
        fix_msg.data = fix_valid
        self.pub_fix.publish(fix_msg)
        
        # TF
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
