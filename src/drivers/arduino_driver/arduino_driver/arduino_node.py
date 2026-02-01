#!/usr/bin/env python3
"""
arduino_node.py - Arduino 시리얼 브릿지 노드

기능:
- Arduino와 시리얼 통신
- 모터 PWM 명령 전송
- IMU 데이터 수신 및 퍼블리시
- 시뮬레이션 모드 지원

토픽:
  Subscribe:
    /motor/command: robot_interfaces/MotorCommand
  Publish:
    /imu/data: sensor_msgs/Imu
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .serial_protocol import SerialProtocol, IMUData, MotorCommand


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_driver_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('simulate', False)
        self.declare_parameter('imu_frame_id', 'imu_link')

        # Load parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        simulate = self.get_parameter('simulate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        # Serial protocol
        self.protocol = SerialProtocol(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            simulate=simulate,
            logger=self.get_logger()
        )

        # Publishers
        self.pub_imu = self.create_publisher(
            Imu, '/imu/data', qos_profile_sensor_data
        )

        # Subscribers
        # Note: robot_interfaces needs to be imported after sourcing
        try:
            from robot_interfaces.msg import MotorCommand as MotorCommandMsg
            self.sub_motor = self.create_subscription(
                MotorCommandMsg,
                '/motor/command',
                self._motor_callback,
                10
            )
            self._has_motor_interface = True
        except ImportError:
            self.get_logger().warn('robot_interfaces not found, using fallback')
            self._has_motor_interface = False

        # Services
        self.srv_enable = self.create_service(
            SetBool, '/motor/enable', self._enable_callback
        )

        # State
        self._motor_enabled = False

        # Connect
        if not self.protocol.connect():
            self.get_logger().error('Failed to connect to Arduino')

        # Set IMU callback
        self.protocol.set_imu_callback(self._imu_callback)

        # Start read thread
        self.protocol.start_read_thread()

        # Watchdog timer (stop motors if no command received)
        self.declare_parameter('watchdog_timeout', 0.5)
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            f'ArduinoNode started: {port} @ {baudrate} (simulate={simulate})'
        )

    def _imu_callback(self, data: IMUData):
        """IMU 데이터 수신 콜백"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id

        # Linear acceleration
        msg.linear_acceleration.x = data.ax
        msg.linear_acceleration.y = data.ay
        msg.linear_acceleration.z = data.az

        # Angular velocity
        msg.angular_velocity.x = data.gx
        msg.angular_velocity.y = data.gy
        msg.angular_velocity.z = data.gz

        # Orientation from yaw (assuming flat surface)
        yaw = data.yaw
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)

        # Covariance (approximate)
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        self.pub_imu.publish(msg)

    def _motor_callback(self, msg):
        """모터 명령 수신 콜백"""
        self._last_cmd_time = self.get_clock().now()

        cmd = MotorCommand(
            front_left=msg.front_left,
            front_right=msg.front_right,
            rear_left=msg.rear_left,
            rear_right=msg.rear_right,
            enabled=msg.enabled and self._motor_enabled
        )

        self.protocol.send_motor_command(cmd)

    def _enable_callback(self, request, response):
        """모터 활성화 서비스 콜백"""
        self._motor_enabled = request.data

        if not request.data:
            # 비활성화 시 모터 정지
            self.protocol.send_motor_command(MotorCommand(enabled=False))

        response.success = True
        response.message = f'Motor {"enabled" if request.data else "disabled"}'
        self.get_logger().info(response.message)

        return response

    def _watchdog_callback(self):
        """Watchdog 콜백 - 명령 없으면 모터 정지"""
        if not self._motor_enabled:
            return

        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self._watchdog_timeout:
            # 모터 정지
            self.protocol.send_motor_command(MotorCommand(enabled=False))
            self.get_logger().warn(
                f'Watchdog: No command for {elapsed:.2f}s, motors stopped',
                throttle_duration_sec=2.0
            )

    def destroy_node(self):
        """Cleanup"""
        self.protocol.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
