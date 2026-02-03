#!/usr/bin/env python3
"""
imu_node.py - MPU6050 I2C IMU 드라이버

기능:
  - 가속도/자이로 읽기
  - Complementary filter (roll/pitch)
  - Yaw gyro 적분
  - 시작 시 bias 캘리브레이션

토픽:
  Publish:
    - /imu/data_raw (Imu) - raw 센서 데이터
    - /imu/data (Imu) - 필터링된 데이터 (orientation 포함)
"""

import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

try:
    import smbus2 as smbus
except ImportError:
    import smbus


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Euler angles to Quaternion"""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MPU6050:
    """MPU6050 I2C Driver"""

    # Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    TEMP_OUT_H = 0x41
    GYRO_XOUT_H = 0x43

    # Scale factors
    ACCEL_SCALE_2G = 16384.0
    GYRO_SCALE_250 = 131.0

    def __init__(self, bus_num: int = 1, address: int = 0x68):
        self.bus = smbus.SMBus(bus_num)
        self.address = address

        # Wake up
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Sample rate divider (1kHz / (1 + div))
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 0x07)  # 125Hz

        # DLPF (low pass filter)
        self.bus.write_byte_data(self.address, self.CONFIG, 0x06)  # 5Hz bandwidth

        # Gyro config (±250°/s)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Accel config (±2g)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

    def _read_word(self, reg: int) -> int:
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def read_accel(self):
        """가속도 읽기 (m/s^2)"""
        ax = self._read_word(self.ACCEL_XOUT_H) / self.ACCEL_SCALE_2G * 9.81
        ay = self._read_word(self.ACCEL_XOUT_H + 2) / self.ACCEL_SCALE_2G * 9.81
        az = self._read_word(self.ACCEL_XOUT_H + 4) / self.ACCEL_SCALE_2G * 9.81
        return ax, ay, az

    def read_gyro(self):
        """자이로 읽기 (rad/s)"""
        gx = self._read_word(self.GYRO_XOUT_H) / self.GYRO_SCALE_250 * math.pi / 180
        gy = self._read_word(self.GYRO_XOUT_H + 2) / self.GYRO_SCALE_250 * math.pi / 180
        gz = self._read_word(self.GYRO_XOUT_H + 4) / self.GYRO_SCALE_250 * math.pi / 180
        return gx, gy, gz

    def close(self):
        self.bus.close()


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ===== Parameters =====
        self.declare_parameter('i2c_bus', 7)  # Jetson: bus 7
        self.declare_parameter('i2c_address', 0x68)

        self.declare_parameter('raw_topic', '/imu/data_raw')
        self.declare_parameter('data_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')

        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('calib_samples', 200)

        # Complementary filter
        self.declare_parameter('comp_alpha', 0.98)  # gyro weight

        # Load params
        self.frame_id = self.get_parameter('frame_id').value
        self.comp_alpha = self.get_parameter('comp_alpha').value
        self.calib_samples = self.get_parameter('calib_samples').value

        # IMU driver
        try:
            bus = self.get_parameter('i2c_bus').value
            addr = self.get_parameter('i2c_address').value
            self.mpu = MPU6050(bus, addr)
            self.get_logger().info(f"MPU6050 connected on bus {bus}, addr 0x{addr:02X}")
        except Exception as e:
            self.get_logger().error(f"MPU6050 init failed: {e}")
            self.mpu = None
            return

        # State
        self._lock = threading.Lock()
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._last_t = time.time()

        # Bias
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._accel_bias = [0.0, 0.0, 0.0]
        self._calibrated = False

        # Publishers
        self.pub_raw = self.create_publisher(
            Imu,
            self.get_parameter('raw_topic').value,
            qos_profile_sensor_data
        )
        self.pub_data = self.create_publisher(
            Imu,
            self.get_parameter('data_topic').value,
            qos_profile_sensor_data
        )

        # Calibration
        self._calibrate()

        # Timer
        period = 1.0 / max(1.0, self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.read_and_publish)

        self.get_logger().info("ImuNode started")

    def _calibrate(self):
        """시작 시 bias 캘리브레이션"""
        if self.mpu is None:
            return

        self.get_logger().info("IMU calibrating... keep still")

        gx_sum, gy_sum, gz_sum = 0.0, 0.0, 0.0
        ax_sum, ay_sum, az_sum = 0.0, 0.0, 0.0

        for _ in range(self.calib_samples):
            ax, ay, az = self.mpu.read_accel()
            gx, gy, gz = self.mpu.read_gyro()

            ax_sum += ax
            ay_sum += ay
            az_sum += az
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz

            time.sleep(0.01)

        n = self.calib_samples
        self._gyro_bias = [gx_sum/n, gy_sum/n, gz_sum/n]
        # 중력 보정: 칩 Z가 하방이므로 정지 시 az≈+9.81
        self._accel_bias = [ax_sum/n, ay_sum/n, az_sum/n - 9.81]

        self._calibrated = True
        self.get_logger().info(
            f"IMU calibrated: gyro_bias={self._gyro_bias}, accel_bias={self._accel_bias}"
        )

    def read_and_publish(self):
        if self.mpu is None or not self._calibrated:
            return

        now = time.time()
        stamp = self.get_clock().now().to_msg()

        try:
            ax_raw, ay_raw, az_raw = self.mpu.read_accel()
            gx_raw, gy_raw, gz_raw = self.mpu.read_gyro()
        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")
            return

        # Bias 보정 (raw 값에 적용)
        ax_raw -= self._accel_bias[0]
        ay_raw -= self._accel_bias[1]
        az_raw -= self._accel_bias[2]
        gx_raw -= self._gyro_bias[0]
        gy_raw -= self._gyro_bias[1]
        gz_raw -= self._gyro_bias[2]

        # ===== 축 변환 =====
        # 장착: 칩 Y→전방, 칩 Z→하방
        # 변환: Robot X = Chip Y, Robot Y = -Chip X, Robot Z = -Chip Z
        ax = ay_raw
        ay = -ax_raw
        az = -az_raw
        gx = gy_raw
        gy = -gx_raw
        gz = -gz_raw

        # Raw 메시지
        raw_msg = Imu()
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id
        raw_msg.linear_acceleration.x = ax
        raw_msg.linear_acceleration.y = ay
        raw_msg.linear_acceleration.z = az
        raw_msg.angular_velocity.x = gx
        raw_msg.angular_velocity.y = gy
        raw_msg.angular_velocity.z = gz
        self.pub_raw.publish(raw_msg)

        # Complementary filter
        with self._lock:
            dt = now - self._last_t
            self._last_t = now

            if dt > 0.5:
                dt = 0.01  # 너무 긴 dt 방지

            # 가속도 기반 roll/pitch (abs 보정으로 기울어진 상태에서도 안정적)
            accel_roll = math.atan2(ay, az + abs(ax))
            accel_pitch = math.atan2(-ax, az + abs(ay))

            # Gyro 적분
            gyro_roll = self._roll + gx * dt
            gyro_pitch = self._pitch + gy * dt

            # Complementary filter
            alpha = self.comp_alpha
            self._roll = alpha * gyro_roll + (1 - alpha) * accel_roll
            self._pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch

            # Yaw는 gyro 적분만 (드리프트 있음)
            self._yaw += gz * dt

            # Wrap yaw
            while self._yaw > math.pi:
                self._yaw -= 2 * math.pi
            while self._yaw < -math.pi:
                self._yaw += 2 * math.pi

        # Filtered 메시지
        data_msg = Imu()
        data_msg.header = raw_msg.header
        data_msg.linear_acceleration = raw_msg.linear_acceleration
        data_msg.angular_velocity = raw_msg.angular_velocity
        data_msg.orientation = euler_to_quaternion(self._roll, self._pitch, self._yaw)

        # Covariance (대략적)
        data_msg.orientation_covariance[0] = 0.01
        data_msg.orientation_covariance[4] = 0.01
        data_msg.orientation_covariance[8] = 0.03  # yaw는 드리프트
        data_msg.angular_velocity_covariance[0] = 0.001
        data_msg.angular_velocity_covariance[4] = 0.001
        data_msg.angular_velocity_covariance[8] = 0.001
        data_msg.linear_acceleration_covariance[0] = 0.01
        data_msg.linear_acceleration_covariance[4] = 0.01
        data_msg.linear_acceleration_covariance[8] = 0.01

        self.pub_data.publish(data_msg)

    def destroy_node(self):
        if self.mpu:
            self.mpu.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
