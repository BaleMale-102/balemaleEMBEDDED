#!/usr/bin/env python3
"""
arduino_node.py - Arduino ROS2 Bridge 노드

기능:
- Arduino와 시리얼 통신
- 속도 명령 전송 (vx, vy, wz)
- Watchdog (명령 없으면 정지)

토픽:
  Subscribe:
    /control/cmd_vel: geometry_msgs/Twist - 속도 명령

  Publish:
    /arduino/status: std_msgs/String - 상태

서비스:
  /arduino/stop: std_srvs/Trigger - 긴급 정지
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .serial_protocol import SerialProtocol, VelocityCommand


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_driver_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('simulate', False)
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('watchdog_timeout', 0.15)  # Arduino watchdog: 200ms
        self.declare_parameter('max_vx', 0.01)   # 20% of Arduino MAX_VEL_LINEAR
        self.declare_parameter('max_vy', 0.01)   # 20% of Arduino MAX_VEL_LINEAR
        self.declare_parameter('max_wz', 0.1)    # 20% of Arduino MAX_VEL_ANGULAR
        self.declare_parameter('wz_offset', 0.0)  # 직진 보정용 (왼쪽 돌면 음수, 오른쪽 돌면 양수)

        # Load parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        simulate = self.get_parameter('simulate').value
        cmd_rate = self.get_parameter('cmd_rate_hz').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        self.wz_offset = self.get_parameter('wz_offset').value

        # Serial protocol
        self.protocol = SerialProtocol(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            simulate=simulate,
            logger=self.get_logger()
        )

        # State
        self._last_cmd_time = self.get_clock().now()
        self._current_cmd = VelocityCommand()
        self._enabled = True

        # Publishers
        self.pub_status = self.create_publisher(String, '/arduino/status', 10)

        # Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/control/cmd_vel', self._cmd_vel_callback, 10
        )

        # Services
        self.srv_stop = self.create_service(
            Trigger, '/arduino/stop', self._stop_callback
        )

        # Connect to Arduino
        if self.protocol.connect():
            self.get_logger().info('Connected to Arduino')
            self.protocol.set_response_callback(self._response_callback)
            self.protocol.start_read_thread()
        else:
            self.get_logger().error('Failed to connect to Arduino')

        # Timer for sending commands
        self.cmd_timer = self.create_timer(1.0 / cmd_rate, self._cmd_timer_callback)

        # Timer for watchdog
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            f'ArduinoNode started: {port} @ {baudrate} (simulate={simulate})'
        )

    def _cmd_vel_callback(self, msg: Twist):
        """속도 명령 수신 콜백"""
        self._last_cmd_time = self.get_clock().now()

        # 속도 제한
        self._current_cmd.vx = self._clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self._current_cmd.vy = self._clamp(msg.linear.y, -self.max_vy, self.max_vy)
        self._current_cmd.wz = self._clamp(msg.angular.z, -self.max_wz, self.max_wz)

    def _cmd_timer_callback(self):
        """주기적 명령 전송"""
        if not self._enabled:
            return

        # wz에 보정값 적용 (직진 시 드리프트 보정)
        wz_corrected = self._current_cmd.wz + self.wz_offset

        self.protocol.send_velocity(
            self._current_cmd.vx,
            self._current_cmd.vy,
            wz_corrected
        )

    def _watchdog_callback(self):
        """Watchdog: 명령 없으면 정지"""
        if not self._enabled:
            return

        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9

        if elapsed > self.watchdog_timeout:
            # 속도 0으로 설정
            self._current_cmd.vx = 0.0
            self._current_cmd.vy = 0.0
            self._current_cmd.wz = 0.0

    def _stop_callback(self, request, response):
        """긴급 정지 서비스 콜백"""
        self.protocol.send_stop()
        self._current_cmd = VelocityCommand()

        response.success = True
        response.message = 'Emergency stop sent'
        self.get_logger().warn('Emergency stop!')

        return response

    def _response_callback(self, line: str):
        """Arduino 응답 콜백"""
        msg = String()
        msg.data = line
        self.pub_status.publish(msg)

        if 'ERR' in line:
            self.get_logger().warn(f'Arduino error: {line}')

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    def destroy_node(self):
        """Cleanup"""
        try:
            self.protocol.send_stop()
            self.protocol.disconnect()
        except Exception:
            pass  # 종료 중 에러 무시
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
