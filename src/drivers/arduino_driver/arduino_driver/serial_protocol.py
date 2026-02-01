#!/usr/bin/env python3
"""
serial_protocol.py - Arduino 시리얼 통신 프로토콜

프로토콜 정의:
- 명령 전송 (Jetson → Arduino):
  "M,FL,FR,RL,RR\n"  - 모터 PWM 명령 (각 값: 300~2000)
  "E,0\n" or "E,1\n" - 모터 비활성화/활성화

- 데이터 수신 (Arduino → Jetson):
  "I,ax,ay,az,gx,gy,gz,yaw\n" - IMU 데이터
  "S,status_code\n"           - 상태 코드
"""

import serial
import time
import threading
from dataclasses import dataclass
from typing import Optional, Callable
import logging


@dataclass
class IMUData:
    """IMU 데이터 구조체"""
    ax: float = 0.0    # 가속도 X (m/s²)
    ay: float = 0.0    # 가속도 Y (m/s²)
    az: float = 0.0    # 가속도 Z (m/s²)
    gx: float = 0.0    # 자이로 X (rad/s)
    gy: float = 0.0    # 자이로 Y (rad/s)
    gz: float = 0.0    # 자이로 Z (rad/s)
    yaw: float = 0.0   # Yaw 각도 (rad)
    timestamp: float = 0.0


@dataclass
class MotorCommand:
    """모터 명령 구조체"""
    front_left: int = 1150   # PWM 중립값
    front_right: int = 1150
    rear_left: int = 1150
    rear_right: int = 1150
    enabled: bool = False


class SerialProtocol:
    """Arduino 시리얼 통신 프로토콜 핸들러"""

    PWM_CENTER = 1150
    PWM_MIN = 300
    PWM_MAX = 2000

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        timeout: float = 0.1,
        simulate: bool = False,
        logger: Optional[logging.Logger] = None
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.simulate = simulate
        self.logger = logger or logging.getLogger(__name__)

        self.serial: Optional[serial.Serial] = None
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Callbacks
        self._imu_callback: Optional[Callable[[IMUData], None]] = None
        self._status_callback: Optional[Callable[[int], None]] = None

        # State
        self._last_motor_cmd = MotorCommand()
        self._last_imu = IMUData()
        self._connected = False

    def connect(self) -> bool:
        """시리얼 연결"""
        if self.simulate:
            self.logger.info('Simulation mode - no serial connection')
            self._connected = True
            return True

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2.0)  # Arduino 리셋 대기

            # 버퍼 클리어
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            self._connected = True
            self.logger.info(f'Connected to {self.port} @ {self.baudrate}')
            return True

        except serial.SerialException as e:
            self.logger.error(f'Serial connection failed: {e}')
            self._connected = False
            return False

    def disconnect(self):
        """시리얼 연결 해제"""
        self.stop_read_thread()

        if self.serial and self.serial.is_open:
            # 모터 정지 명령
            self.send_motor_command(MotorCommand(enabled=False))
            time.sleep(0.1)
            self.serial.close()

        self._connected = False
        self.logger.info('Disconnected')

    def start_read_thread(self):
        """수신 스레드 시작"""
        if self._running:
            return

        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        self.logger.info('Read thread started')

    def stop_read_thread(self):
        """수신 스레드 정지"""
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        self._read_thread = None

    def _read_loop(self):
        """시리얼 수신 루프"""
        while self._running:
            try:
                if self.simulate:
                    # 시뮬레이션: 가상 IMU 데이터 생성
                    time.sleep(0.02)  # 50Hz
                    imu = IMUData(
                        ax=0.0, ay=0.0, az=9.81,
                        gx=0.0, gy=0.0, gz=0.0,
                        yaw=0.0,
                        timestamp=time.time()
                    )
                    if self._imu_callback:
                        self._imu_callback(imu)
                    continue

                if not self.serial or not self.serial.is_open:
                    time.sleep(0.1)
                    continue

                line = self.serial.readline().decode('utf-8').strip()
                if not line:
                    continue

                self._parse_line(line)

            except serial.SerialException as e:
                self.logger.error(f'Serial read error: {e}')
                time.sleep(0.1)
            except Exception as e:
                self.logger.error(f'Read loop error: {e}')
                time.sleep(0.01)

    def _parse_line(self, line: str):
        """수신 라인 파싱"""
        parts = line.split(',')
        if len(parts) < 2:
            return

        msg_type = parts[0]

        if msg_type == 'I' and len(parts) >= 8:
            # IMU 데이터
            try:
                imu = IMUData(
                    ax=float(parts[1]),
                    ay=float(parts[2]),
                    az=float(parts[3]),
                    gx=float(parts[4]),
                    gy=float(parts[5]),
                    gz=float(parts[6]),
                    yaw=float(parts[7]),
                    timestamp=time.time()
                )
                self._last_imu = imu
                if self._imu_callback:
                    self._imu_callback(imu)
            except ValueError:
                pass

        elif msg_type == 'S' and len(parts) >= 2:
            # 상태 코드
            try:
                status = int(parts[1])
                if self._status_callback:
                    self._status_callback(status)
            except ValueError:
                pass

    def send_motor_command(self, cmd: MotorCommand) -> bool:
        """모터 명령 전송"""
        with self._lock:
            self._last_motor_cmd = cmd

            if not cmd.enabled:
                # 비활성화 명령
                message = "E,0\n"
            else:
                # PWM 값 클램핑
                fl = max(self.PWM_MIN, min(self.PWM_MAX, cmd.front_left))
                fr = max(self.PWM_MIN, min(self.PWM_MAX, cmd.front_right))
                rl = max(self.PWM_MIN, min(self.PWM_MAX, cmd.rear_left))
                rr = max(self.PWM_MIN, min(self.PWM_MAX, cmd.rear_right))

                message = f"M,{fl},{fr},{rl},{rr}\n"

            if self.simulate:
                return True

            if not self.serial or not self.serial.is_open:
                return False

            try:
                self.serial.write(message.encode('utf-8'))
                self.serial.flush()
                return True
            except serial.SerialException as e:
                self.logger.error(f'Serial write error: {e}')
                return False

    def set_imu_callback(self, callback: Callable[[IMUData], None]):
        """IMU 데이터 콜백 설정"""
        self._imu_callback = callback

    def set_status_callback(self, callback: Callable[[int], None]):
        """상태 코드 콜백 설정"""
        self._status_callback = callback

    @property
    def is_connected(self) -> bool:
        """연결 상태"""
        return self._connected

    @property
    def last_imu(self) -> IMUData:
        """마지막 IMU 데이터"""
        return self._last_imu
