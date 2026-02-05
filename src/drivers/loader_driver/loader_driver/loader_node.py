#!/usr/bin/env python3
"""
loader_node.py - Loader Mechanism Driver Node

Controls the vehicle loader mechanism via serial_bridge node (ROS topic bridge).

Mode 1 (use_bridge=True): Communicates via serial_bridge node
    Publishes: /motor_cmd (String) -> serial_bridge -> Arduino
    Subscribes: /loading_status (String) <- serial_bridge <- Arduino

Mode 2 (use_bridge=False): Direct serial communication
    Serial Protocol (carrying.ino, 9600 baud)

Subscribes:
    /loader/command: LoaderCommand (from mission_manager)

Publishes:
    /loader/status: LoaderStatus (to mission_manager)
"""

import serial
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LoaderDriverNode(Node):
    """ROS2 driver node for loader Arduino (carrying.ino)."""

    # Status constants
    STATUS_IDLE = 'IDLE'
    STATUS_LOADING = 'LOADING'
    STATUS_UNLOADING = 'UNLOADING'
    STATUS_DONE = 'DONE'
    STATUS_ERROR = 'ERROR'
    STATUS_DISCONNECTED = 'DISCONNECTED'

    def __init__(self):
        super().__init__('loader_driver')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('status_rate_hz', 10.0)
        self.declare_parameter('simulate', False)
        self.declare_parameter('operation_duration', 10.0)  # Fallback timeout (seconds)
        self.declare_parameter('use_bridge', True)  # Use serial_bridge node for communication

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.status_rate = self.get_parameter('status_rate_hz').value
        self.simulate = self.get_parameter('simulate').value
        self.operation_duration = self.get_parameter('operation_duration').value
        self.use_bridge = self.get_parameter('use_bridge').value

        # State
        self._serial = None
        self._connected = False
        self._current_status = self.STATUS_IDLE if self.use_bridge else self.STATUS_DISCONNECTED
        self._is_loaded = False
        self._last_message = ''
        self._lock = threading.Lock()

        # Operation timing
        self._operation_start_time = 0.0
        self._current_operation = None  # 'LOAD' or 'UNLOAD'

        # Import custom interfaces
        try:
            from robot_interfaces.msg import LoaderCommand, LoaderStatus
            self._has_interface = True
            self._LoaderCommand = LoaderCommand
            self._LoaderStatus = LoaderStatus
        except ImportError:
            self.get_logger().warn('robot_interfaces not found, using String messages')
            self._has_interface = False

        # Publishers - to mission_manager
        if self._has_interface:
            self.pub_status = self.create_publisher(
                self._LoaderStatus, '/loader/status', 10
            )
        else:
            self.pub_status = self.create_publisher(
                String, '/loader/status', 10
            )

        # Subscribers - from mission_manager
        if self._has_interface:
            self.sub_command = self.create_subscription(
                self._LoaderCommand, '/loader/command',
                self._command_callback, 10
            )
        else:
            self.sub_command = self.create_subscription(
                String, '/loader/command',
                self._command_string_callback, 10
            )

        # Bridge mode: communicate via serial_bridge node
        if self.use_bridge:
            # Publisher to serial_bridge
            self.pub_motor_cmd = self.create_publisher(String, '/motor_cmd', 10)
            # Subscriber from serial_bridge
            self.sub_loading_status = self.create_subscription(
                String, '/loading_status',
                self._loading_status_callback, 10
            )
            self._connected = True
            self.get_logger().info('Using serial_bridge mode (topics: /motor_cmd, /loading_status)')
        elif not self.simulate:
            # Direct serial mode
            self._connect()
        else:
            self._current_status = self.STATUS_IDLE
            self._connected = True
            self.get_logger().info('Running in SIMULATION mode')

        # Status timer
        self.timer = self.create_timer(1.0 / self.status_rate, self._status_callback)

        self.get_logger().info(f'LoaderDriverNode started (bridge={self.use_bridge})')

    def _loading_status_callback(self, msg: String):
        """Handle response from serial_bridge (/loading_status topic)."""
        status = msg.data.strip()
        self.get_logger().info(f'Received from serial_bridge: {status}')

        if status == "completed" and self._current_operation is not None:
            if self._current_operation == 'LOAD':
                self._is_loaded = True
                self._last_message = 'Load complete (bridge confirmed)'
                self.get_logger().info('LOAD operation complete')
            elif self._current_operation == 'UNLOAD':
                self._is_loaded = False
                self._last_message = 'Unload complete (bridge confirmed)'
                self.get_logger().info('UNLOAD operation complete')

            self._current_status = self.STATUS_DONE
            self._current_operation = None

    def _connect(self):
        """Connect to Arduino serial port (direct mode)."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2.0)  # Wait for Arduino reset
            self._connected = True
            self._current_status = self.STATUS_IDLE
            self.get_logger().info(f'Connected to loader Arduino on {self.port}')

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.port}: {e}')
            self._connected = False
            self._current_status = self.STATUS_DISCONNECTED

    def _send_command(self, cmd: str):
        """Send command to Arduino (direct serial or via bridge)."""
        if self.use_bridge:
            # Send via /motor_cmd topic to serial_bridge
            msg = String()
            msg.data = cmd
            self.pub_motor_cmd.publish(msg)
            self.get_logger().info(f'Published to /motor_cmd: {cmd}')
            return True

        # Direct serial mode
        if not self._connected or self._serial is None:
            self.get_logger().warn('Not connected, cannot send command')
            return False

        try:
            with self._lock:
                self._serial.write(f'{cmd}'.encode())
                self._serial.flush()
            self.get_logger().info(f'Sent command: {cmd}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self._connected = False
            return False

    def _start_operation(self, operation: str):
        """Start a load/unload operation with timer-based status tracking."""
        if self._current_operation is not None:
            self.get_logger().warn(f'Operation {self._current_operation} already in progress')
            return False

        self._current_operation = operation
        self._operation_start_time = time.time()

        if operation == 'LOAD':
            self._current_status = self.STATUS_LOADING
            self._last_message = 'Loading vehicle...'
        else:
            self._current_status = self.STATUS_UNLOADING
            self._last_message = 'Unloading vehicle...'

        return True

    def _update_operation_status(self):
        """Update status based on elapsed time (fallback timeout)."""
        if self._current_operation is None:
            return

        elapsed = time.time() - self._operation_start_time
        if elapsed >= self.operation_duration:
            self.get_logger().warn(f'Operation timeout ({self.operation_duration}s), assuming complete')
            if self._current_operation == 'LOAD':
                self._is_loaded = True
                self._last_message = 'Load complete (timeout)'
            elif self._current_operation == 'UNLOAD':
                self._is_loaded = False
                self._last_message = 'Unload complete (timeout)'
            self._current_status = self.STATUS_DONE
            self._current_operation = None

    def _command_callback(self, msg):
        """Handle LoaderCommand message."""
        cmd = msg.command.upper()
        self._execute_command(cmd)

    def _command_string_callback(self, msg: String):
        """Handle String command (fallback)."""
        cmd = msg.data.upper()
        self._execute_command(cmd)

    def _execute_command(self, cmd: str):
        """Execute a command (carrying.ino protocol: '1'=load, '2'=unload)."""
        if cmd == 'LOAD':
            self.get_logger().info('Executing LOAD command')
            if self._start_operation('LOAD'):
                if self.simulate:
                    self.get_logger().info('[SIM] LOAD started')
                else:
                    self._send_command('1')  # Arduino: '1' = load

        elif cmd == 'UNLOAD':
            self.get_logger().info('Executing UNLOAD command')
            if self._start_operation('UNLOAD'):
                if self.simulate:
                    self.get_logger().info('[SIM] UNLOAD started')
                else:
                    self._send_command('2')  # Arduino: '2' = unload

        elif cmd == 'STOP':
            self.get_logger().info('STOP command (state reset)')
            self._current_operation = None
            self._current_status = self.STATUS_IDLE
            self._last_message = 'Stopped'

        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def _status_callback(self):
        """Publish current status."""
        # Update operation status based on timer
        self._update_operation_status()

        if self._has_interface:
            msg = self._LoaderStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.status = self._current_status
            msg.is_loaded = self._is_loaded
            msg.message = self._last_message
        else:
            msg = String()
            msg.data = f'{self._current_status}|{self._is_loaded}|{self._last_message}'

        self.pub_status.publish(msg)

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self._serial and self._serial.is_open:
            # carrying.ino has no stop command, just close serial
            self._serial.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoaderDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
