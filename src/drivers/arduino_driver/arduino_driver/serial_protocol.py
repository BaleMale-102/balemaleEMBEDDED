#!/usr/bin/env python3
"""
serial_protocol.py - Arduino ROS2 Bridge 시리얼 프로토콜

프로토콜 (115200 baud):
  송신 (Jetson → Arduino):
    "V vx vy wz\n"    - 속도 명령 (m/s, rad/s)
    "D nx ny nw\n"    - 정규화 속도 (-1 ~ +1)
    "S\n"             - 긴급 정지
    "P max_pwm\n"     - 최대 PWM 설정
    "?\n"             - 상태 조회

  수신 (Arduino → Jetson):
    "OK\n"            - 명령 확인
    "READY\n"         - 부팅 완료
    "STOPPED\n"       - 정지됨
    "ERR\n"           - 에러
"""

import serial
import time
import threading
from dataclasses import dataclass
from typing import Optional, Callable
import logging


@dataclass
class VelocityCommand:
    """속도 명령"""
    vx: float = 0.0    # 전진 속도 (m/s)
    vy: float = 0.0    # 횡방향 속도 (m/s)
    wz: float = 0.0    # 회전 속도 (rad/s)


class SerialProtocol:
    """Arduino ROS2 Bridge 시리얼 통신"""

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
        self._response_callback: Optional[Callable[[str], None]] = None

        # State
        self._connected = False
        self._last_response = ""

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
            # 모터 정지
            self.send_stop()
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
                    time.sleep(0.1)
                    continue

                if not self.serial or not self.serial.is_open:
                    time.sleep(0.1)
                    continue

                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self._last_response = line
                        if self._response_callback:
                            self._response_callback(line)

            except serial.SerialException as e:
                self.logger.error(f'Serial read error: {e}')
                time.sleep(0.1)
            except Exception as e:
                self.logger.error(f'Read loop error: {e}')
                time.sleep(0.01)

    def _send(self, message: str) -> bool:
        """메시지 전송"""
        if self.simulate:
            return True

        if not self.serial or not self.serial.is_open:
            return False

        with self._lock:
            try:
                self.serial.write((message + '\n').encode('utf-8'))
                self.serial.flush()
                return True
            except serial.SerialException as e:
                self.logger.error(f'Serial write error: {e}')
                return False

    def send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """속도 명령 전송 (m/s, rad/s)"""
        message = f"V {vx:.4f} {vy:.4f} {wz:.4f}"
        return self._send(message)

    def send_velocity_normalized(self, nx: float, ny: float, nw: float) -> bool:
        """정규화 속도 명령 전송 (-1 ~ +1)"""
        message = f"D {nx:.4f} {ny:.4f} {nw:.4f}"
        return self._send(message)

    def send_stop(self) -> bool:
        """긴급 정지"""
        return self._send("S")

    def send_set_max_pwm(self, max_pwm: int) -> bool:
        """최대 PWM 설정"""
        message = f"P {max_pwm}"
        return self._send(message)

    def send_query_status(self) -> bool:
        """상태 조회"""
        return self._send("?")

    def set_response_callback(self, callback: Callable[[str], None]):
        """응답 콜백 설정"""
        self._response_callback = callback

    @property
    def is_connected(self) -> bool:
        """연결 상태"""
        return self._connected

    @property
    def last_response(self) -> str:
        """마지막 응답"""
        return self._last_response
