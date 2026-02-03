#!/usr/bin/env python3
"""
loader_node.py - Loader Mechanism Driver Node

Controls the vehicle loader mechanism via Arduino serial communication.

Serial Protocol:
    Commands (ROS -> Arduino):
        L     Load vehicle
        U     Unload vehicle
        S     Emergency stop
        ?     Status query

    Responses (Arduino -> ROS):
        IDLE      Ready, no operation
        LOADING   Load operation in progress
        UNLOADING Unload operation in progress
        DONE      Operation completed successfully
        ERROR     Operation failed

Subscribes:
    /loader/command: LoaderCommand

Publishes:
    /loader/status: LoaderStatus
"""

import serial
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LoaderDriverNode(Node):
    """ROS2 driver node for loader Arduino."""

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
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('status_rate_hz', 10.0)
        self.declare_parameter('simulate', False)
        self.declare_parameter('simulate_duration', 3.0)  # Simulated operation time

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.status_rate = self.get_parameter('status_rate_hz').value
        self.simulate = self.get_parameter('simulate').value
        self.simulate_duration = self.get_parameter('simulate_duration').value

        # State
        self._serial = None
        self._connected = False
        self._current_status = self.STATUS_DISCONNECTED
        self._is_loaded = False
        self._last_message = ''
        self._lock = threading.Lock()

        # Simulation state
        self._sim_start_time = 0.0
        self._sim_operation = None

        # Import custom interfaces
        try:
            from robot_interfaces.msg import LoaderCommand, LoaderStatus
            self._has_interface = True
            self._LoaderCommand = LoaderCommand
            self._LoaderStatus = LoaderStatus
        except ImportError:
            self.get_logger().warn('robot_interfaces not found, using String messages')
            self._has_interface = False

        # Publishers
        if self._has_interface:
            self.pub_status = self.create_publisher(
                self._LoaderStatus, '/loader/status', 10
            )
        else:
            self.pub_status = self.create_publisher(
                String, '/loader/status', 10
            )

        # Subscribers
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

        # Connect to serial
        if not self.simulate:
            self._connect()
        else:
            self._current_status = self.STATUS_IDLE
            self._connected = True
            self.get_logger().info('Running in SIMULATION mode')

        # Status timer
        self.timer = self.create_timer(1.0 / self.status_rate, self._status_callback)

        # Serial read thread (only if real hardware)
        if not self.simulate and self._connected:
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

        self.get_logger().info(f'LoaderDriverNode started on {self.port}')

    def _connect(self):
        """Connect to Arduino serial port."""
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

            # Query initial status
            self._send_command('?')

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.port}: {e}')
            self._connected = False
            self._current_status = self.STATUS_DISCONNECTED

    def _send_command(self, cmd: str):
        """Send command to Arduino."""
        if self.simulate:
            self._handle_simulated_command(cmd)
            return True

        if not self._connected or self._serial is None:
            self.get_logger().warn('Not connected, cannot send command')
            return False

        try:
            with self._lock:
                self._serial.write(f'{cmd}\n'.encode())
                self._serial.flush()
            self.get_logger().debug(f'Sent command: {cmd}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self._connected = False
            return False

    def _handle_simulated_command(self, cmd: str):
        """Handle command in simulation mode."""
        if cmd == 'L':
            self._current_status = self.STATUS_LOADING
            self._sim_operation = 'LOAD'
            self._sim_start_time = time.time()
            self.get_logger().info('[SIM] Starting LOAD operation')
        elif cmd == 'U':
            self._current_status = self.STATUS_UNLOADING
            self._sim_operation = 'UNLOAD'
            self._sim_start_time = time.time()
            self.get_logger().info('[SIM] Starting UNLOAD operation')
        elif cmd == 'S':
            self._current_status = self.STATUS_IDLE
            self._sim_operation = None
            self.get_logger().info('[SIM] STOP command')
        elif cmd == '?':
            self.get_logger().info(f'[SIM] Status: {self._current_status}')

    def _update_simulation(self):
        """Update simulation state."""
        if self._sim_operation is None:
            return

        elapsed = time.time() - self._sim_start_time
        if elapsed >= self.simulate_duration:
            # Operation complete
            if self._sim_operation == 'LOAD':
                self._is_loaded = True
                self.get_logger().info('[SIM] LOAD complete')
            elif self._sim_operation == 'UNLOAD':
                self._is_loaded = False
                self.get_logger().info('[SIM] UNLOAD complete')

            self._current_status = self.STATUS_DONE
            self._sim_operation = None

    def _read_loop(self):
        """Background thread to read serial responses."""
        while rclpy.ok() and self._connected:
            try:
                if self._serial and self._serial.in_waiting > 0:
                    with self._lock:
                        line = self._serial.readline().decode().strip()
                    if line:
                        self._parse_response(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                self._connected = False
                break
            except Exception as e:
                self.get_logger().error(f'Read loop error: {e}')

            time.sleep(0.01)

    def _parse_response(self, response: str):
        """Parse Arduino response."""
        response = response.upper().strip()

        if response in (self.STATUS_IDLE, self.STATUS_LOADING,
                        self.STATUS_UNLOADING, self.STATUS_DONE, self.STATUS_ERROR):
            prev_status = self._current_status
            self._current_status = response

            # Track loaded state
            if response == self.STATUS_DONE:
                if prev_status == self.STATUS_LOADING:
                    self._is_loaded = True
                    self._last_message = 'Load complete'
                elif prev_status == self.STATUS_UNLOADING:
                    self._is_loaded = False
                    self._last_message = 'Unload complete'

            self.get_logger().debug(f'Status updated: {response}')
        else:
            # Treat as message
            self._last_message = response
            self.get_logger().info(f'Arduino message: {response}')

    def _command_callback(self, msg):
        """Handle LoaderCommand message."""
        cmd = msg.command.upper()
        self._execute_command(cmd)

    def _command_string_callback(self, msg: String):
        """Handle String command (fallback)."""
        cmd = msg.data.upper()
        self._execute_command(cmd)

    def _execute_command(self, cmd: str):
        """Execute a command."""
        if cmd == 'LOAD':
            self.get_logger().info('Executing LOAD command')
            self._send_command('L')
        elif cmd == 'UNLOAD':
            self.get_logger().info('Executing UNLOAD command')
            self._send_command('U')
        elif cmd == 'STOP':
            self.get_logger().info('Executing STOP command')
            self._send_command('S')
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def _status_callback(self):
        """Publish current status."""
        # Update simulation if needed
        if self.simulate:
            self._update_simulation()

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
            self._send_command('S')  # Safety stop
            self._serial.close()
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
