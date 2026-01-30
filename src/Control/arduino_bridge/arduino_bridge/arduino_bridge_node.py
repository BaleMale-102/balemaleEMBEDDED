#!/usr/bin/env python3
"""
arduino_bridge_node.py

ROS2 <-> Arduino UART 브릿지

기능:
  - DriveCmd → Arduino 전송
  - Arduino Odometry → ROS2 발행
  - TF odom→base_link 발행

토픽:
  Sub: /control/drive_cmd_safe (DriveCmd)
  Pub: /odom/wheel (Odometry)
       /arduino/status (String)
"""

import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rc_interfaces.msg import DriveCmd

import serial
import tf2_ros


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timeout_sec', 0.1)
        self.declare_parameter('drive_cmd_topic', '/control/drive_cmd_safe')
        self.declare_parameter('odom_topic', '/odom/wheel')
        self.declare_parameter('status_topic', '/arduino/status')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_pwm', 2500)
        self.declare_parameter('odom_request_hz', 20.0)
        self.declare_parameter('handshake_timeout', 5.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.odom_hz = self.get_parameter('odom_request_hz').value
        self.hs_timeout = self.get_parameter('handshake_timeout').value

        # Serial
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._connected = False

        # Odometry state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_odom_t = self.get_clock().now()

        # Publishers
        self.pub_odom = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.pub_status = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)

        # TF
        self.tf_br = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        # Subscribers
        self.sub_cmd = self.create_subscription(
            DriveCmd, self.get_parameter('drive_cmd_topic').value,
            self.cb_cmd, 10)

        # Timers
        self.create_timer(1.0, self._conn_monitor)
        if self.odom_hz > 0:
            self.create_timer(1.0 / self.odom_hz, self._request_odom)

        # RX Thread
        self._rx_run = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self._connect()
        self.get_logger().info(f"arduino_bridge started: {self.port}")

    def destroy_node(self):
        self._rx_run = False
        self._disconnect()
        super().destroy_node()

    # === Serial ===
    
    def _connect(self) -> bool:
        with self._lock:
            if self._ser:
                return True
            try:
                ser = serial.Serial()
                ser.port = self.port
                ser.baudrate = self.baud
                ser.timeout = self.timeout
                ser.dtr = False
                ser.rts = False
                ser.open()
                self._ser = ser
            except Exception as e:
                self.get_logger().warn(f"Serial open failed: {e}")
                return False

        if not self._handshake():
            self._disconnect()
            return False

        self._connected = True
        self._pub_status("CONNECTED")
        return True

    def _disconnect(self):
        with self._lock:
            if self._ser:
                try:
                    self._ser.close()
                except:
                    pass
                self._ser = None
            self._connected = False

    def _handshake(self) -> bool:
        start = time.time()
        ready = False
        self.get_logger().info("Waiting for READY...")
        
        while (time.time() - start) < self.hs_timeout:
            line = self._read_line()
            if line and 'READY' in line.upper():
                ready = True
                break

        if not ready:
            self.get_logger().error("Handshake timeout")
            return False

        self._write(f"P {self.max_pwm}")
        time.sleep(0.05)
        
        for _ in range(10):
            line = self._read_line()
            if line and 'OK' in line.upper():
                self.get_logger().info(f"Handshake OK")
                return True
        
        return True

    def _conn_monitor(self):
        if not self._connected:
            self._connect()

    def _read_line(self) -> Optional[str]:
        with self._lock:
            if not self._ser:
                return None
            try:
                return self._ser.readline().decode('utf-8', errors='ignore').strip()
            except:
                self._connected = False
                return None

    def _write(self, data: str) -> bool:
        with self._lock:
            if not self._ser:
                return False
            try:
                self._ser.write((data + '\n').encode())
                return True
            except:
                self._connected = False
                return False

    def _rx_loop(self):
        while self._rx_run:
            if not self._connected:
                time.sleep(0.1)
                continue
            line = self._read_line()
            if line:
                self._process_rx(line)
            else:
                time.sleep(0.005)

    def _process_rx(self, line: str):
        parts = line.split()
        if not parts:
            return
        
        cmd = parts[0].upper()
        
        if cmd == 'O' and len(parts) >= 4:
            try:
                vx = float(parts[1])
                vy = float(parts[2])
                wz = float(parts[3])
                self._update_odom(vx, vy, wz)
            except ValueError:
                pass
        elif cmd == 'WATCHDOG':
            self.get_logger().warn("Arduino WATCHDOG")
            self._pub_status("WATCHDOG")

    # === Odometry ===
    
    def _update_odom(self, vx: float, vy: float, wz: float):
        now = self.get_clock().now()
        dt = (now - self._last_odom_t).nanoseconds * 1e-9
        self._last_odom_t = now
        
        if dt <= 0 or dt > 1.0:
            dt = 0.05

        # Integrate
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._x += (vx * c - vy * s) * dt
        self._y += (vx * s + vy * c) * dt
        self._yaw += wz * dt
        
        # Wrap
        while self._yaw > math.pi: self._yaw -= 2*math.pi
        while self._yaw < -math.pi: self._yaw += 2*math.pi

        # Publish
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation = yaw_to_quat(self._yaw)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        self.pub_odom.publish(msg)

        # TF
        if self.tf_br:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_br.sendTransform(tf)

    def _request_odom(self):
        if self._connected:
            self._write("O")

    # === Command ===
    
    def cb_cmd(self, msg: DriveCmd):
        if not self._connected:
            return
        
        if not msg.enable:
            self._write("Z")
            return
        
        self._write(f"V {msg.vx:.4f} {msg.vy:.4f} {msg.wz:.4f}")

    def _pub_status(self, status: str):
        m = String()
        m.data = status
        self.pub_status.publish(m)


def main():
    rclpy.init()
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
