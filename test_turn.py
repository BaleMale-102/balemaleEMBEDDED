#!/usr/bin/env python3
"""
test_turn.py - 90도 턴 테스트 스크립트

사용법:
  1. sensors.launch.py 실행 (IMU, Arduino)
  2. motion_controller 실행
  3. python3 test_turn.py

키:
  L / ← : 좌회전 90도
  R / → : 우회전 90도
  Q     : 종료
"""

import sys
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Imu

try:
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False


def quaternion_to_yaw(q) -> float:
    """Quaternion에서 yaw 추출"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TurnTester(Node):
    def __init__(self):
        super().__init__('turn_tester')

        # State
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.is_turning = False
        self.target_deg = 0.0

        # Publishers
        self.pub_enable = self.create_publisher(Bool, '/control/enable_drive', 10)
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_turn_target = self.create_publisher(Int32, '/mission/turn_target_rad', 10)

        # Subscribers
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_profile_sensor_data
        )
        self.sub_turn_done = self.create_subscription(
            Bool, '/mission/turn_done', self.turn_done_callback, 10
        )

        # 시작 시 드라이브 활성화
        time.sleep(0.5)
        self.enable_drive(True)

        self.get_logger().info('TurnTester ready. Press L(left) or R(right) to test 90deg turn.')

    def imu_callback(self, msg: Imu):
        self.current_yaw = quaternion_to_yaw(msg.orientation)

    def turn_done_callback(self, msg: Bool):
        if msg.data and self.is_turning:
            self.is_turning = False

            # 실제 회전량 계산
            turned = self.current_yaw - self.start_yaw
            while turned > math.pi:
                turned -= 2 * math.pi
            while turned < -math.pi:
                turned += 2 * math.pi

            turned_deg = math.degrees(turned)
            error_deg = self.target_deg - turned_deg

            print(f"\n=== Turn Complete ===")
            print(f"  Target:  {self.target_deg:+.1f} deg")
            print(f"  Actual:  {turned_deg:+.1f} deg")
            print(f"  Error:   {error_deg:+.1f} deg")
            print(f"=====================\n")

    def enable_drive(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_enable.publish(msg)

    def start_turn(self, deg: float):
        """턴 시작 (deg: 양수=좌회전, 음수=우회전)"""
        if self.is_turning:
            print("Already turning...")
            return

        self.start_yaw = self.current_yaw
        self.target_deg = deg
        self.is_turning = True

        # 1. IDLE로 리셋
        state_msg = String()
        state_msg.data = 'IDLE'
        self.pub_state.publish(state_msg)
        time.sleep(0.1)

        # 2. 턴 각도 설정 (centiradians)
        rad = math.radians(deg)
        centirads = int(rad * 100)

        turn_msg = Int32()
        turn_msg.data = centirads
        self.pub_turn_target.publish(turn_msg)
        time.sleep(0.1)

        # 3. TURN 모드 시작
        state_msg.data = 'TURN'
        self.pub_state.publish(state_msg)

        print(f"\nStarting {deg:+.0f} deg turn...")
        print(f"  Start yaw: {math.degrees(self.start_yaw):.1f} deg")

    def stop(self):
        """정지"""
        self.is_turning = False
        state_msg = String()
        state_msg.data = 'IDLE'
        self.pub_state.publish(state_msg)
        self.enable_drive(False)


def get_key():
    """키보드 입력 읽기"""
    if not HAS_TERMIOS:
        return input("Command (l/r/q): ").lower()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Arrow keys
        if ch == '\x1b':
            ch2 = sys.stdin.read(2)
            if ch2 == '[D':  # Left arrow
                return 'l'
            elif ch2 == '[C':  # Right arrow
                return 'r'
        return ch.lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = TurnTester()

    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "="*40)
    print("       90 Degree Turn Tester")
    print("="*40)
    print("  L / ← : Turn LEFT  90 deg")
    print("  R / → : Turn RIGHT 90 deg")
    print("  S     : Stop")
    print("  Q     : Quit")
    print("="*40 + "\n")

    try:
        while rclpy.ok():
            key = get_key()

            if key == 'l':
                node.start_turn(90.0)   # 좌회전
            elif key == 'r':
                node.start_turn(-90.0)  # 우회전
            elif key == 's':
                node.stop()
                print("Stopped.")
            elif key == 'q' or key == '\x03':  # q or Ctrl+C
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\nBye!")


if __name__ == '__main__':
    main()
