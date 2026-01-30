#!/usr/bin/env python3
"""
safety_manager_node.py

안전 관리자 + IMU Heading Hold

기능:
  1. Emergency MUX: 긴급 명령 우선순위
  2. Watchdog: 명령 없으면 정지
  3. Slew Rate Limiting: 급가속 방지
  4. Speed Clamp: 최대 속도 제한
  5. IMU Heading Hold: 직진 시 yaw 드리프트 보정

토픽:
  Subscribe:
    - /control/drive_cmd (DriveCmd) - AUTO 명령
    - /control/drive_cmd_emergency (DriveCmd) - 긴급 명령 (우선)
    - /imu/data (Imu) - IMU
  
  Publish:
    - /control/drive_cmd_safe (DriveCmd) - 안전 처리된 명령
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from rc_interfaces.msg import DriveCmd


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def slew_rate(current: float, target: float, max_rate: float, dt: float) -> float:
    """Slew rate limiting"""
    if dt <= 0:
        return target
    
    max_change = max_rate * dt
    diff = target - current
    
    if abs(diff) <= max_change:
        return target
    
    return current + max_change if diff > 0 else current - max_change


class SafetyManagerNode(Node):
    def __init__(self):
        super().__init__('safety_manager_node')
        
        # ===== Parameters =====
        self.declare_parameter('auto_cmd_topic', '/control/drive_cmd')
        self.declare_parameter('emergency_cmd_topic', '/control/drive_cmd_emergency')
        self.declare_parameter('safe_cmd_topic', '/control/drive_cmd_safe')
        self.declare_parameter('imu_topic', '/imu/data')
        
        self.declare_parameter('watchdog_timeout_sec', 0.2)
        self.declare_parameter('emergency_timeout_sec', 0.3)
        
        # Slew rate (units/sec)
        self.declare_parameter('slew_rate_vx', 1.0)
        self.declare_parameter('slew_rate_vy', 1.0)
        self.declare_parameter('slew_rate_wz', 3.0)
        
        # Speed limits
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_wz', 1.5)
        
        # IMU Heading Hold
        self.declare_parameter('use_imu_heading_hold', True)
        self.declare_parameter('heading_hold_kp', 0.5)
        self.declare_parameter('heading_hold_ki', 0.05)
        self.declare_parameter('heading_hold_threshold_wz', 0.12)  # |wz| < 이면 heading hold
        self.declare_parameter('heading_hold_min_v', 0.10)        # |vx| or |vy| > 이면 heading hold
        self.declare_parameter('heading_hold_max_vy', 0.20)       # |vy| < 이면 heading hold
        self.declare_parameter('imu_timeout_sec', 0.2)
        
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('base_frame', 'base_link')
        
        # Load params
        self.watchdog_timeout = self.get_parameter('watchdog_timeout_sec').value
        self.emergency_timeout = self.get_parameter('emergency_timeout_sec').value
        
        self.slew_vx = self.get_parameter('slew_rate_vx').value
        self.slew_vy = self.get_parameter('slew_rate_vy').value
        self.slew_wz = self.get_parameter('slew_rate_wz').value
        
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        
        self.use_heading_hold = self.get_parameter('use_imu_heading_hold').value
        self.heading_kp = self.get_parameter('heading_hold_kp').value
        self.heading_ki = self.get_parameter('heading_hold_ki').value
        self.heading_wz_thresh = self.get_parameter('heading_hold_threshold_wz').value
        self.heading_min_v = self.get_parameter('heading_hold_min_v').value
        self.heading_max_vy = self.get_parameter('heading_hold_max_vy').value
        self.imu_timeout = self.get_parameter('imu_timeout_sec').value
        
        self.base_frame = self.get_parameter('base_frame').value
        
        # State
        self._lock = threading.Lock()
        
        # Current output
        self._out_vx = 0.0
        self._out_vy = 0.0
        self._out_wz = 0.0
        
        # Command inputs
        self._auto_cmd: DriveCmd = None
        self._auto_cmd_t = 0.0
        
        self._emg_cmd: DriveCmd = None
        self._emg_cmd_t = 0.0
        
        # IMU
        self._imu_yaw = 0.0
        self._imu_gyro_z = 0.0
        self._imu_t = 0.0
        
        # Heading hold state
        self._heading_target = 0.0
        self._heading_integral = 0.0
        self._heading_active = False
        
        # Timing
        self._last_update_t = time.time()
        
        # Publishers
        self.pub_safe = self.create_publisher(
            DriveCmd,
            self.get_parameter('safe_cmd_topic').value,
            10
        )
        
        # Subscribers
        self.sub_auto = self.create_subscription(
            DriveCmd,
            self.get_parameter('auto_cmd_topic').value,
            self.cb_auto,
            10
        )
        self.sub_emg = self.create_subscription(
            DriveCmd,
            self.get_parameter('emergency_cmd_topic').value,
            self.cb_emergency,
            10
        )
        self.sub_imu = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.cb_imu,
            qos_profile_sensor_data
        )
        
        # Timer
        period = 1.0 / max(1.0, self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.update_loop)
        
        self.get_logger().info(
            f"safety_manager_node started\n"
            f"  safe_cmd: {self.get_parameter('safe_cmd_topic').value}\n"
            f"  use_heading_hold: {self.use_heading_hold}"
        )
    
    # ===== Callbacks =====
    
    def cb_auto(self, msg: DriveCmd):
        with self._lock:
            self._auto_cmd = msg
            self._auto_cmd_t = time.time()
    
    def cb_emergency(self, msg: DriveCmd):
        with self._lock:
            self._emg_cmd = msg
            self._emg_cmd_t = time.time()
    
    def cb_imu(self, msg: Imu):
        with self._lock:
            self._imu_t = time.time()
            self._imu_gyro_z = msg.angular_velocity.z
            
            # Quaternion to yaw
            q = msg.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._imu_yaw = math.atan2(siny, cosy)
    
    # ===== Main Loop =====
    
    def update_loop(self):
        now = time.time()
        
        with self._lock:
            dt = now - self._last_update_t
            self._last_update_t = now
            
            if dt <= 0 or dt > 0.5:
                dt = 0.02
            
            # 1. MUX: Emergency 우선
            use_emergency = False
            if self._emg_cmd is not None:
                emg_age = now - self._emg_cmd_t
                if emg_age < self.emergency_timeout and self._emg_cmd.enable:
                    use_emergency = True
            
            # 2. 명령 선택
            if use_emergency:
                cmd = self._emg_cmd
                cmd_t = self._emg_cmd_t
            elif self._auto_cmd is not None:
                cmd = self._auto_cmd
                cmd_t = self._auto_cmd_t
            else:
                cmd = None
                cmd_t = 0.0
            
            # 3. Watchdog
            if cmd is None or (now - cmd_t) > self.watchdog_timeout:
                target_vx = 0.0
                target_vy = 0.0
                target_wz = 0.0
                enable = False
            else:
                target_vx = cmd.vx if cmd.enable else 0.0
                target_vy = cmd.vy if cmd.enable else 0.0
                target_wz = cmd.wz if cmd.enable else 0.0
                enable = cmd.enable
            
            # 4. Speed Clamp
            target_vx = clamp(target_vx, -self.max_vx, self.max_vx)
            target_vy = clamp(target_vy, -self.max_vy, self.max_vy)
            target_wz = clamp(target_wz, -self.max_wz, self.max_wz)
            
            # 5. IMU Heading Hold
            if self.use_heading_hold and enable:
                target_wz = self._apply_heading_hold(target_vx, target_vy, target_wz, dt)
            
            # 6. Slew Rate Limiting
            self._out_vx = slew_rate(self._out_vx, target_vx, self.slew_vx, dt)
            self._out_vy = slew_rate(self._out_vy, target_vy, self.slew_vy, dt)
            self._out_wz = slew_rate(self._out_wz, target_wz, self.slew_wz, dt)
            
            # 발행
            out = DriveCmd()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.base_frame
            out.enable = enable
            out.vx = float(self._out_vx)
            out.vy = float(self._out_vy)
            out.wz = float(self._out_wz)
            out.source = "emergency" if use_emergency else "auto"
        
        self.pub_safe.publish(out)
    
    def _apply_heading_hold(self, vx: float, vy: float, wz: float, dt: float) -> float:
        """
        IMU 기반 Heading Hold
        조건: |wz| < threshold, (|vx| or |vy|) > min, |vy| < max
        """
        now = time.time()
        imu_valid = (now - self._imu_t) < self.imu_timeout
        
        # Heading hold 조건 체크
        should_hold = (
            abs(wz) < self.heading_wz_thresh and
            (abs(vx) >= self.heading_min_v or abs(vy) >= self.heading_min_v) and
            abs(vy) < self.heading_max_vy
        )
        
        if not should_hold or not imu_valid:
            # Heading hold 비활성화
            if self._heading_active:
                self._heading_active = False
                self._heading_integral = 0.0
            return wz
        
        # Heading hold 활성화
        if not self._heading_active:
            # 새로 활성화: 현재 yaw를 목표로 설정
            self._heading_target = self._imu_yaw
            self._heading_integral = 0.0
            self._heading_active = True
        
        # 에러 계산
        error = self._heading_target - self._imu_yaw
        
        # Wrap to [-π, π]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        # PI 제어
        self._heading_integral += error * dt
        self._heading_integral = clamp(self._heading_integral, -1.0, 1.0)  # anti-windup
        
        wz_correction = self.heading_kp * error + self.heading_ki * self._heading_integral
        
        return wz + wz_correction


def main():
    rclpy.init()
    node = SafetyManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
