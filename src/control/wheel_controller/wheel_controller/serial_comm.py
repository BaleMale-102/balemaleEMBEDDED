#!/usr/bin/env python3
"""
serial_comm.py - Arduino Serial Communication

Protocol:
  TX (to Arduino):   "M,FL,FR,RL,RR\n"  - Motor command
  RX (from Arduino): "I,ax,ay,az,gx,gy,gz,yaw\n" - IMU data
                     "E,enc1,enc2,enc3,enc4\n" - Encoder data
                     "S,status\n" - Status message
"""

import threading
import time
from typing import Optional, Callable
from dataclasses import dataclass

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


@dataclass
class IMUData:
    """IMU data from Arduino."""
    ax: float = 0.0  # Acceleration X (m/s^2)
    ay: float = 0.0  # Acceleration Y
    az: float = 0.0  # Acceleration Z
    gx: float = 0.0  # Gyro X (rad/s)
    gy: float = 0.0  # Gyro Y
    gz: float = 0.0  # Gyro Z
    yaw: float = 0.0  # Estimated yaw (rad)


@dataclass
class EncoderData:
    """Encoder data from Arduino."""
    ticks: list = None  # [FL, FR, RL, RR] encoder ticks

    def __post_init__(self):
        if self.ticks is None:
            self.ticks = [0, 0, 0, 0]


class ArduinoSerial:
    """Thread-safe Arduino serial communication."""

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baud: int = 115200,
        timeout: float = 0.1,
        simulation: bool = False
    ):
        """
        Initialize Arduino serial connection.

        Args:
            port: Serial port path
            baud: Baud rate
            timeout: Read timeout in seconds
            simulation: If True, simulate without actual hardware
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.simulation = simulation

        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

        # Callbacks
        self._imu_callback: Optional[Callable[[IMUData], None]] = None
        self._encoder_callback: Optional[Callable[[EncoderData], None]] = None
        self._status_callback: Optional[Callable[[str], None]] = None

        # Latest data
        self._last_imu = IMUData()
        self._last_encoder = EncoderData()

    def connect(self) -> bool:
        """Connect to Arduino."""
        if self.simulation:
            self._running = True
            return True

        if not HAS_SERIAL:
            return False

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout
            )
            time.sleep(2.0)  # Wait for Arduino reset
            self._serial.reset_input_buffer()
            self._running = True

            # Start read thread
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

            return True

        except Exception as e:
            print(f'Serial connect error: {e}')
            return False

    def disconnect(self):
        """Disconnect from Arduino."""
        self._running = False

        if self._read_thread is not None:
            self._read_thread.join(timeout=1.0)
            self._read_thread = None

        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def send_motor_command(self, fl: int, fr: int, rl: int, rr: int):
        """
        Send motor PWM command.

        Args:
            fl, fr, rl, rr: Motor PWM values (-3000 to 3000)
        """
        cmd = f'M,{fl},{fr},{rl},{rr}\n'

        if self.simulation:
            return

        if self._serial is None:
            return

        with self._lock:
            try:
                self._serial.write(cmd.encode('utf-8'))
            except Exception:
                pass

    def stop_motors(self):
        """Stop all motors."""
        self.send_motor_command(0, 0, 0, 0)

    def set_imu_callback(self, callback: Callable[[IMUData], None]):
        """Set IMU data callback."""
        self._imu_callback = callback

    def set_encoder_callback(self, callback: Callable[[EncoderData], None]):
        """Set encoder data callback."""
        self._encoder_callback = callback

    def set_status_callback(self, callback: Callable[[str], None]):
        """Set status callback."""
        self._status_callback = callback

    @property
    def last_imu(self) -> IMUData:
        """Get last IMU reading."""
        return self._last_imu

    @property
    def last_encoder(self) -> EncoderData:
        """Get last encoder reading."""
        return self._last_encoder

    @property
    def is_connected(self) -> bool:
        return self._running

    def _read_loop(self):
        """Background thread for reading serial data."""
        while self._running:
            if self._serial is None:
                time.sleep(0.1)
                continue

            try:
                line = self._serial.readline().decode('utf-8').strip()
                if line:
                    self._parse_line(line)
            except Exception:
                pass

    def _parse_line(self, line: str):
        """Parse incoming serial line."""
        parts = line.split(',')
        if len(parts) < 2:
            return

        msg_type = parts[0]

        try:
            if msg_type == 'I' and len(parts) >= 8:
                # IMU data: I,ax,ay,az,gx,gy,gz,yaw
                imu = IMUData(
                    ax=float(parts[1]),
                    ay=float(parts[2]),
                    az=float(parts[3]),
                    gx=float(parts[4]),
                    gy=float(parts[5]),
                    gz=float(parts[6]),
                    yaw=float(parts[7])
                )
                self._last_imu = imu
                if self._imu_callback:
                    self._imu_callback(imu)

            elif msg_type == 'E' and len(parts) >= 5:
                # Encoder data: E,fl,fr,rl,rr
                enc = EncoderData(
                    ticks=[int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])]
                )
                self._last_encoder = enc
                if self._encoder_callback:
                    self._encoder_callback(enc)

            elif msg_type == 'S':
                # Status: S,message
                status = ','.join(parts[1:])
                if self._status_callback:
                    self._status_callback(status)

        except (ValueError, IndexError):
            pass
