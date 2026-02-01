#!/usr/bin/env python3
"""
ekf_localization_node.py (v2 - Command velocity prediction)

엔코더 없이 동작하도록 수정:
- 엔코더 odom 대신 DriveCmd 구독해서 명령 속도로 예측
- 마커 업데이트가 메인

센서 융합:
  - ArUco Marker: 절대 위치 (업데이트)
  - IMU: angular velocity (예측 보조)
  - DriveCmd: 명령 속도 (예측) ← 엔코더 대신
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

from rc_interfaces.msg import MarkerStatus, DriveCmd

import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
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
    """6-state EKF: [x, y, θ, vx, vy, ω]"""
    
    def __init__(self):
        self.x = np.zeros(6)
        self.P = np.eye(6) * 0.1
        
        # Process noise - 명령 기반 예측용으로 조정
        self.Q = np.diag([
            0.005,   # x
            0.005,   # y
            0.003,   # θ
            0.02,    # vx (명령 속도 신뢰)
            0.02,    # vy
            0.01     # ω
        ])
        
        # Marker measurement noise
        self.R_marker = np.diag([
            0.015,   # x - 마커 신뢰도 높임
            0.015,   # y
            0.03     # θ
        ])
        
        self.initialized = False
    
    def initialize(self, x: float, y: float, yaw: float):
        self.x = np.array([x, y, yaw, 0.0, 0.0, 0.0])
        self.P = np.eye(6) * 0.1
        self.initialized = True
    
    def predict(self, dt: float, cmd_vx: float = 0.0, cmd_vy: float = 0.0, 
                cmd_omega: float = 0.0, imu_omega: Optional[float] = None):
        """명령 속도 + IMU로 예측"""
        if not self.initialized or dt <= 0:
            return
        
        x, y, theta, vx, vy, omega = self.x
        
        # 명령 속도 사용 (decay 적용)
        vx = cmd_vx * 0.9 + vx * 0.1
        vy = cmd_vy * 0.9 + vy * 0.1
        
        # IMU omega 우선, 없으면 명령 사용
        if imu_omega is not None:
            omega = imu_omega * 0.7 + cmd_omega * 0.3
        else:
            omega = cmd_omega * 0.9 + omega * 0.1
        
        c = math.cos(theta)
        s = math.sin(theta)
        
        x_new = x + (vx * c - vy * s) * dt
        y_new = y + (vx * s + vy * c) * dt
        theta_new = wrap_angle(theta + omega * dt)
        
        self.x = np.array([x_new, y_new, theta_new, vx, vy, omega])
        
        # Jacobian
        F = np.eye(6)
        F[0, 2] = (-vx * s - vy * c) * dt
        F[0, 3] = c * dt
        F[0, 4] = -s * dt
        F[1, 2] = (vx * c - vy * s) * dt
        F[1, 3] = s * dt
        F[1, 4] = c * dt
        F[2, 5] = dt
        
        self.P = F @ self.P @ F.T + self.Q * dt
    
    def update_marker(self, meas_x: float, meas_y: float, meas_yaw: float, quality: float = 1.0):
        """마커 측정으로 업데이트 (quality 가중치)"""
        if not self.initialized:
            self.initialize(meas_x, meas_y, meas_yaw)
            return
        
        z = np.array([meas_x, meas_y, meas_yaw])
        h = np.array([self.x[0], self.x[1], self.x[2]])
        
        y = z - h
        y[2] = wrap_angle(y[2])
        
        H = np.zeros((3, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        
        # Quality에 따라 R 조정 (quality 낮으면 노이즈 증가)
        R = self.R_marker / max(0.3, quality)
        
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.x[2] = wrap_angle(self.x[2])
        
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
    
    def get_pose(self):
        return self.x[0], self.x[1], self.x[2]
    
    def get_velocity(self):
        return self.x[3], self.x[4], self.x[5]
    
    def get_covariance_pose(self):
        return self.P[:3, :3].copy()


class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')
        
        # Parameters
        self.declare_parameter('marker_status_topic', '/perception/marker_status')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('drive_cmd_topic', '/control/drive_cmd_safe')
        
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('odom_out_topic', '/localization/odom')
        self.declare_parameter('fix_valid_topic', '/localization/fix_valid')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        self.declare_parameter('publish_tf', True)
        
        self.declare_parameter('marker_map_yaml', '')
        self.declare_parameter('map_unit_scale', 0.01)
        
        self.declare_parameter('cam_offset_x', 0.06)
        self.declare_parameter('cam_offset_y', 0.0)
        
        self.declare_parameter('min_marker_quality', 0.2)
        self.declare_parameter('marker_timeout_sec', 2.0)
        self.declare_parameter('imu_timeout_sec', 0.3)
        self.declare_parameter('cmd_timeout_sec', 0.3)
        
        self.declare_parameter('predict_rate_hz', 50.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        
        self.declare_parameter('log_throttle_sec', 2.0)
        
        # Load
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.map_scale = self.get_parameter('map_unit_scale').value
        self.cam_offset_x = self.get_parameter('cam_offset_x').value
        self.cam_offset_y = self.get_parameter('cam_offset_y').value
        
        self.min_quality = self.get_parameter('min_marker_quality').value
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value
        self.imu_timeout = self.get_parameter('imu_timeout_sec').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        
        self.log_throttle_sec = self.get_parameter('log_throttle_sec').value
        
        # State
        self._lock = threading.Lock()
        self._last_log_t = 0.0
        
        # EKF
        self.ekf = EKFLocalization()
        
        # Marker map
        self.marker_map: Dict[int, MarkerMapEntry] = {}
        self._load_marker_map()
        
        # Timestamps
        self._last_marker_t = 0.0
        self._last_imu_t = 0.0
        self._last_cmd_t = 0.0
        self._last_predict_t = time.time()
        
        # Latest data
        self._latest_imu_omega: Optional[float] = None
        self._latest_cmd_vx: float = 0.0
        self._latest_cmd_vy: float = 0.0
        self._latest_cmd_omega: float = 0.0
        
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishers
        self.pub_pose = self.create_publisher(PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.pub_odom = self.create_publisher(Odometry, self.get_parameter('odom_out_topic').value, 10)
        self.pub_fix = self.create_publisher(Bool, self.get_parameter('fix_valid_topic').value, 10)
        
        # Subscribers
        self.sub_marker = self.create_subscription(
            MarkerStatus, self.get_parameter('marker_status_topic').value, self.cb_marker, 10)
        self.sub_imu = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.cb_imu, qos_profile_sensor_data)
        self.sub_cmd = self.create_subscription(
            DriveCmd, self.get_parameter('drive_cmd_topic').value, self.cb_cmd, 10)
        
        # Timers
        predict_period = 1.0 / max(1.0, self.get_parameter('predict_rate_hz').value)
        publish_period = 1.0 / max(1.0, self.get_parameter('publish_rate_hz').value)
        
        self.timer_predict = self.create_timer(predict_period, self.predict_step)
        self.timer_publish = self.create_timer(publish_period, self.publish_state)
        
        self.get_logger().info(f"ekf_localization_node v2 (cmd-based) started, markers: {len(self.marker_map)}")
    
    def _load_marker_map(self):
        path = self.get_parameter('marker_map_yaml').value
        if not path:
            self.get_logger().warn("marker_map_yaml not specified")
            return
        
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            
            for m in data.get('markers', []):
                mid = m.get('id')
                if mid is None:
                    continue
                
                x = m.get('x')
                y = m.get('y')
                if x is None or y is None:
                    continue
                
                yaw = m.get('yaw', 0.0)
                self.marker_map[mid] = MarkerMapEntry(x=float(x), y=float(y), yaw=float(yaw))
            
            self.get_logger().info(f"Loaded marker_map: {len(self.marker_map)} entries")
        except Exception as e:
            self.get_logger().error(f"Failed to load marker_map: {e}")
    
    def _log_throttle(self, msg: str):
        now = time.time()
        if now - self._last_log_t >= self.log_throttle_sec:
            self._last_log_t = now
            self.get_logger().info(msg)
    
    def cb_marker(self, msg: MarkerStatus):
        if not msg.valid or msg.quality < self.min_quality:
            return
        
        entry = self.marker_map.get(msg.id)
        if entry is None:
            return
        
        with self._lock:
            self._last_marker_t = time.time()
            
            marker_x = entry.x * self.map_scale
            marker_y = entry.y * self.map_scale
            marker_yaw = entry.yaw
            
            robot_yaw = wrap_angle(marker_yaw + math.pi - msg.rel_yaw)
            
            c = math.cos(robot_yaw)
            s = math.sin(robot_yaw)
            
            dx_map = c * msg.rel_z - s * msg.rel_x
            dy_map = s * msg.rel_z + c * msg.rel_x
            
            robot_x = marker_x - dx_map
            robot_y = marker_y - dy_map
            
            robot_x -= c * self.cam_offset_x - s * self.cam_offset_y
            robot_y -= s * self.cam_offset_x + c * self.cam_offset_y
            
            self.ekf.update_marker(robot_x, robot_y, robot_yaw, msg.quality)
            
            self._log_throttle(f"Marker {msg.id}: robot=[{robot_x:.3f}, {robot_y:.3f}, {math.degrees(robot_yaw):.1f}°]")
    
    def cb_imu(self, msg: Imu):
        with self._lock:
            self._last_imu_t = time.time()
            self._latest_imu_omega = msg.angular_velocity.z
    
    def cb_cmd(self, msg: DriveCmd):
        with self._lock:
            self._last_cmd_t = time.time()
            if msg.enable:
                self._latest_cmd_vx = msg.vx
                self._latest_cmd_vy = msg.vy
                self._latest_cmd_omega = msg.wz
            else:
                self._latest_cmd_vx = 0.0
                self._latest_cmd_vy = 0.0
                self._latest_cmd_omega = 0.0
    
    def predict_step(self):
        now = time.time()
        
        with self._lock:
            dt = now - self._last_predict_t
            self._last_predict_t = now
            
            if not self.ekf.initialized:
                return
            
            # 명령 속도 (timeout 체크)
            if (now - self._last_cmd_t) < self.cmd_timeout:
                cmd_vx = self._latest_cmd_vx
                cmd_vy = self._latest_cmd_vy
                cmd_omega = self._latest_cmd_omega
            else:
                cmd_vx = cmd_vy = cmd_omega = 0.0
            
            # IMU omega
            imu_omega = None
            if (now - self._last_imu_t) < self.imu_timeout:
                imu_omega = self._latest_imu_omega
            
            self.ekf.predict(dt, cmd_vx, cmd_vy, cmd_omega, imu_omega)
    
    def publish_state(self):
        now = time.time()
        stamp = self.get_clock().now().to_msg()
        
        with self._lock:
            if not self.ekf.initialized:
                return
            
            x, y, yaw = self.ekf.get_pose()
            vx, vy, omega = self.ekf.get_velocity()
            cov = self.ekf.get_covariance_pose()
            
            marker_age = now - self._last_marker_t
            fix_valid = marker_age < self.marker_timeout
        
        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = yaw_to_quaternion(yaw)
        self.pub_pose.publish(pose_msg)
        
        # Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.pose.covariance[0] = cov[0, 0]
        odom_msg.pose.covariance[7] = cov[1, 1]
        odom_msg.pose.covariance[35] = cov[2, 2]
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