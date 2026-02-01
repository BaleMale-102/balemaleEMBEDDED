#!/usr/bin/env python3
"""
controller_node.py - Wheel Controller Node

Converts velocity commands to motor PWMs and communicates with Arduino.

Subscribes:
    /control/cmd_vel: Twist (velocity commands)

Publishes:
    /motor/command: MotorCommand (PWM values)
    /imu/data: Imu (from Arduino)
    /odom/wheel: Odometry (from encoders)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .mecanum_kinematics import MecanumKinematics, MotorPWMs
from .serial_comm import ArduinoSerial, IMUData, EncoderData


class WheelControllerNode(Node):
    """Wheel controller with mecanum kinematics and Arduino interface."""

    def __init__(self):
        super().__init__('wheel_controller_node')

        # Parameters - kinematics
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_base_x', 0.08)
        self.declare_parameter('wheel_base_y', 0.06)
        self.declare_parameter('max_wheel_speed', 10.0)
        self.declare_parameter('max_pwm', 3000)
        self.declare_parameter('pwm_deadzone', 100)

        # Parameters - serial
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('simulation', False)

        # Parameters - safety
        self.declare_parameter('watchdog_timeout', 0.3)
        self.declare_parameter('cmd_rate', 20.0)

        # Load parameters
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        cmd_rate = self.get_parameter('cmd_rate').value
        simulation = self.get_parameter('simulation').value

        # Initialize kinematics
        self.kinematics = MecanumKinematics(
            wheel_radius=self.get_parameter('wheel_radius').value,
            wheel_base_x=self.get_parameter('wheel_base_x').value,
            wheel_base_y=self.get_parameter('wheel_base_y').value,
            max_wheel_speed=self.get_parameter('max_wheel_speed').value,
            max_pwm=self.get_parameter('max_pwm').value,
            pwm_deadzone=self.get_parameter('pwm_deadzone').value
        )

        # Initialize serial
        self.arduino = ArduinoSerial(
            port=self.get_parameter('serial_port').value,
            baud=self.get_parameter('serial_baud').value,
            simulation=simulation
        )
        self.arduino.set_imu_callback(self._imu_callback)
        self.arduino.set_encoder_callback(self._encoder_callback)
        self.arduino.set_status_callback(self._status_callback)

        # State
        self._last_cmd_time = self.get_clock().now()
        self._current_cmd = Twist()
        self._encoder_ticks = [0, 0, 0, 0]
        self._last_encoder_ticks = [0, 0, 0, 0]
        self._last_odom_time = self.get_clock().now()

        # Odometry state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        # Import custom interfaces
        try:
            from robot_interfaces.msg import MotorCommand
            self._has_interface = True
            self._MotorCommand = MotorCommand
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False

        # Publishers
        if self._has_interface:
            self.pub_motor = self.create_publisher(
                self._MotorCommand, '/motor/command', 10
            )

        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/odom/wheel', 10)
        self.pub_status = self.create_publisher(String, '/arduino/status', 10)

        # Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/control/cmd_vel', self._cmd_vel_callback, 10
        )

        # Timer for motor commands
        self.cmd_timer = self.create_timer(1.0 / cmd_rate, self._cmd_timer_callback)

        # Connect to Arduino
        if self.arduino.connect():
            self.get_logger().info('Connected to Arduino')
        else:
            self.get_logger().warn('Failed to connect to Arduino (simulation mode)')

        self.get_logger().info('WheelControllerNode started')

    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity command."""
        self._current_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _cmd_timer_callback(self):
        """Periodic motor command update."""
        # Watchdog check
        now = self.get_clock().now()
        dt = (now - self._last_cmd_time).nanoseconds / 1e9

        if dt > self.watchdog_timeout:
            # No recent command - stop motors
            self.arduino.stop_motors()
            self._publish_motor_command(MotorPWMs())
            return

        # Convert velocity to PWM
        vx = self._current_cmd.linear.x
        vy = self._current_cmd.linear.y
        wz = self._current_cmd.angular.z

        pwms = self.kinematics.velocity_to_pwm(vx, vy, wz)

        # Send to Arduino
        self.arduino.send_motor_command(
            pwms.front_left, pwms.front_right,
            pwms.rear_left, pwms.rear_right
        )

        # Publish motor command message
        self._publish_motor_command(pwms)

    def _imu_callback(self, imu_data: IMUData):
        """Handle IMU data from Arduino."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Linear acceleration
        msg.linear_acceleration.x = imu_data.ax
        msg.linear_acceleration.y = imu_data.ay
        msg.linear_acceleration.z = imu_data.az

        # Angular velocity
        msg.angular_velocity.x = imu_data.gx
        msg.angular_velocity.y = imu_data.gy
        msg.angular_velocity.z = imu_data.gz

        # Orientation from yaw
        yaw = imu_data.yaw
        msg.orientation.z = math.sin(yaw / 2)
        msg.orientation.w = math.cos(yaw / 2)

        self.pub_imu.publish(msg)

    def _encoder_callback(self, encoder_data: EncoderData):
        """Handle encoder data from Arduino."""
        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = now

        if dt <= 0:
            return

        # Calculate wheel velocities from encoder differences
        delta_ticks = [
            encoder_data.ticks[i] - self._last_encoder_ticks[i]
            for i in range(4)
        ]
        self._last_encoder_ticks = encoder_data.ticks.copy()

        # Convert ticks to rad/s (assuming 360 ticks per revolution)
        ticks_per_rev = 360
        wheel_speeds_rad = [
            (delta * 2 * math.pi) / (ticks_per_rev * dt)
            for delta in delta_ticks
        ]

        # Forward kinematics to get body velocity
        from .mecanum_kinematics import WheelSpeeds
        speeds = WheelSpeeds(
            front_left=wheel_speeds_rad[0],
            front_right=wheel_speeds_rad[1],
            rear_left=wheel_speeds_rad[2],
            rear_right=wheel_speeds_rad[3]
        )
        vx, vy, wz = self.kinematics.forward(speeds)

        # Integrate odometry
        delta_x = (vx * math.cos(self._odom_yaw) - vy * math.sin(self._odom_yaw)) * dt
        delta_y = (vx * math.sin(self._odom_yaw) + vy * math.cos(self._odom_yaw)) * dt
        delta_yaw = wz * dt

        self._odom_x += delta_x
        self._odom_y += delta_y
        self._odom_yaw += delta_yaw

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = math.sin(self._odom_yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self._odom_yaw / 2)

        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.pub_odom.publish(odom)

    def _status_callback(self, status: str):
        """Handle status from Arduino."""
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)

    def _publish_motor_command(self, pwms: MotorPWMs):
        """Publish motor command message."""
        if not self._has_interface:
            return

        msg = self._MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.front_left = pwms.front_left
        msg.front_right = pwms.front_right
        msg.rear_left = pwms.rear_left
        msg.rear_right = pwms.rear_right

        self.pub_motor.publish(msg)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.arduino.stop_motors()
        self.arduino.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
