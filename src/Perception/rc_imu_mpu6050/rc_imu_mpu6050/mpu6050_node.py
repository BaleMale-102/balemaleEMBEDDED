#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

try:
    from smbus2 import SMBus
except Exception as e:
    raise RuntimeError("smbus2 not installed. Run: pip3 install smbus2") from e


# MPU6050 registers
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


@dataclass
class Bias:
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0


class MPU6050:
    def __init__(self, bus: int, addr: int):
        self.bus_id = bus
        self.addr = addr
        self.bus = SMBus(bus)

    def write_byte(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def read_i16(self, reg_h):
        hi = self.bus.read_byte_data(self.addr, reg_h)
        lo = self.bus.read_byte_data(self.addr, reg_h + 1)
        v = (hi << 8) | lo
        if v >= 0x8000:
            v -= 0x10000
        return v

    def init(self, accel_range_g=2, gyro_range_dps=250, dlpf=3, sample_div=4):
        # Wake up
        self.write_byte(PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        # Sample rate = Gyro output rate / (1 + SMPLRT_DIV)
        self.write_byte(SMPLRT_DIV, sample_div & 0xFF)

        # DLPF
        self.write_byte(CONFIG, dlpf & 0x07)

        # Accel range
        # 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
        a_map = {2: 0, 4: 1, 8: 2, 16: 3}
        a_sel = a_map.get(accel_range_g, 0)
        self.write_byte(ACCEL_CONFIG, (a_sel << 3) & 0x18)

        # Gyro range
        # 0: ±250, 1: ±500, 2: ±1000, 3: ±2000 dps
        g_map = {250: 0, 500: 1, 1000: 2, 2000: 3}
        g_sel = g_map.get(gyro_range_dps, 0)
        self.write_byte(GYRO_CONFIG, (g_sel << 3) & 0x18)

        # Interrupt enable (optional)
        self.write_byte(INT_ENABLE, 0x01)

        self.accel_range_g = accel_range_g
        self.gyro_range_dps = gyro_range_dps

        # LSB scale factors
        self.accel_lsb_per_g = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}[accel_range_g]
        self.gyro_lsb_per_dps = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}[gyro_range_dps]

    def read(self):
        ax = self.read_i16(ACCEL_XOUT_H)
        ay = self.read_i16(ACCEL_XOUT_H + 2)
        az = self.read_i16(ACCEL_XOUT_H + 4)

        gx = self.read_i16(GYRO_XOUT_H)
        gy = self.read_i16(GYRO_XOUT_H + 2)
        gz = self.read_i16(GYRO_XOUT_H + 4)
        return ax, ay, az, gx, gy, gz

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


class Mpu6050ImuNode(Node):
    def __init__(self):
        super().__init__("mpu6050_imu_node")

        # Params (너 프로젝트에 맞게 기본값을 보수적으로 잡음)
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_addr", 0x68)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 100.0)

        self.declare_parameter("accel_range_g", 2)
        self.declare_parameter("gyro_range_dps", 250)
        self.declare_parameter("dlpf", 3)
        self.declare_parameter("sample_div", 4)

        self.declare_parameter("calib_seconds", 2.0)
        self.declare_parameter("use_complementary_filter", True)
        self.declare_parameter("comp_alpha", 0.98)

        self.pub_raw = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.pub = self.create_publisher(Imu, "/imu/data", 10)

        bus = int(self.get_parameter("i2c_bus").value)
        addr = int(self.get_parameter("i2c_addr").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)

        accel_range_g = int(self.get_parameter("accel_range_g").value)
        gyro_range_dps = int(self.get_parameter("gyro_range_dps").value)
        dlpf = int(self.get_parameter("dlpf").value)
        sample_div = int(self.get_parameter("sample_div").value)

        self.use_cf = bool(self.get_parameter("use_complementary_filter").value)
        self.alpha = float(self.get_parameter("comp_alpha").value)
        self.alpha = clamp(self.alpha, 0.0, 1.0)

        self.imu = MPU6050(bus, addr)
        self.imu.init(accel_range_g=accel_range_g, gyro_range_dps=gyro_range_dps, dlpf=dlpf, sample_div=sample_div)

        self.bias = Bias()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        calib_s = float(self.get_parameter("calib_seconds").value)
        self._calibrate(calib_s)

        self.last_t = self.get_clock().now()
        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"MPU6050 IMU started: bus={bus}, addr=0x{addr:02X}, rate={self.rate_hz}Hz, frame_id={self.frame_id}"
        )

    def destroy_node(self):
        try:
            self.imu.close()
        except Exception:
            pass
        super().destroy_node()

    def _calibrate(self, seconds: float):
        # 로봇을 정지/수평 상태로 두고 바이어스 추정
        n = int(max(10, seconds * 200))  # 대충 200Hz 정도로 샘플
        self.get_logger().info(f"Calibrating for {seconds:.1f}s (samples={n})... keep robot still.")

        s_ax = s_ay = s_az = 0.0
        s_gx = s_gy = s_gz = 0.0

        for _ in range(n):
            ax, ay, az, gx, gy, gz = self.imu.read()
            s_ax += ax
            s_ay += ay
            s_az += az
            s_gx += gx
            s_gy += gy
            s_gz += gz
            time.sleep(0.005)

        self.bias.ax = s_ax / n
        self.bias.ay = s_ay / n
        self.bias.az = s_az / n
        self.bias.gx = s_gx / n
        self.bias.gy = s_gy / n
        self.bias.gz = s_gz / n

        self.get_logger().info(
            f"Bias set: "
            f"acc=({self.bias.ax:.1f},{self.bias.ay:.1f},{self.bias.az:.1f}), "
            f"gyro=({self.bias.gx:.1f},{self.bias.gy:.1f},{self.bias.gz:.1f})"
        )

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(1.0, self.rate_hz)
        self.last_t = now

        ax_i16, ay_i16, az_i16, gx_i16, gy_i16, gz_i16 = self.imu.read()

        # Remove bias
        ax = (ax_i16 - self.bias.ax) / self.imu.accel_lsb_per_g  # g
        ay = (ay_i16 - self.bias.ay) / self.imu.accel_lsb_per_g
        az = (az_i16 - self.bias.az) / self.imu.accel_lsb_per_g

        gx_dps = (gx_i16 - self.bias.gx) / self.imu.gyro_lsb_per_dps
        gy_dps = (gy_i16 - self.bias.gy) / self.imu.gyro_lsb_per_dps
        gz_dps = (gz_i16 - self.bias.gz) / self.imu.gyro_lsb_per_dps

        # Publish raw IMU (no orientation)
        raw = Imu()
        raw.header.stamp = now.to_msg()
        raw.header.frame_id = self.frame_id
        raw.linear_acceleration.x = ax * 9.80665
        raw.linear_acceleration.y = ay * 9.80665
        raw.linear_acceleration.z = az * 9.80665
        raw.angular_velocity.x = math.radians(gx_dps)
        raw.angular_velocity.y = math.radians(gy_dps)
        raw.angular_velocity.z = math.radians(gz_dps)
        raw.orientation_covariance[0] = -1.0  # orientation unknown
        self.pub_raw.publish(raw)

        # Complementary filter (roll/pitch). yaw는 gyro 적분(드리프트 있음)
        if self.use_cf:
            # Accel angle (assuming az roughly gravity direction)
            # roll: x-rotation, pitch: y-rotation
            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

            # Gyro integrate
            self.roll += math.radians(gx_dps) * dt
            self.pitch += math.radians(gy_dps) * dt
            self.yaw += math.radians(gz_dps) * dt

            # Blend
            self.roll = self.alpha * self.roll + (1.0 - self.alpha) * roll_acc
            self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * pitch_acc

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        if self.use_cf:
            msg.orientation = quat_from_rpy(self.roll, self.pitch, self.yaw)
        else:
            msg.orientation_covariance[0] = -1.0

        msg.linear_acceleration = raw.linear_acceleration
        msg.angular_velocity = raw.angular_velocity

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Mpu6050ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

