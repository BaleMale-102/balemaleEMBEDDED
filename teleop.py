#!/usr/bin/env python3
"""
teleop.py - 키보드로 RC Car 수동 제어

사용법:
  python3 teleop.py          # Direct 모드 (기본) - Arduino 직접 연결
  python3 teleop.py --ros    # ROS 모드
  python3 teleop.py --port /dev/ttyUSB0  # 포트 지정

조작:
  W/↑ : 전진       S/↓ : 후진
  A/← : 좌측이동   D/→ : 우측이동
  Q   : 좌회전     E   : 우회전

  Space : 정지
  1 : Enable drive (ROS 모드만)
  2 : Disable drive (ROS 모드만)

  +/= : 속도 증가
  -   : 속도 감소

  Ctrl+C : 종료
"""

import sys
import termios
import tty
import select
import argparse
import time

# 방향키 코드
KEY_UP = 'A'
KEY_DOWN = 'B'
KEY_RIGHT = 'C'
KEY_LEFT = 'D'


class DirectArduinoController:
    """Arduino 직접 시리얼 통신"""

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        import serial
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)  # Arduino 리셋 대기
        print(f"[Direct] Arduino connected on {port}")

    def send_velocity(self, vx, vy, wz):
        """V 명령으로 속도 전송"""
        cmd = f"V {vx:.4f} {vy:.4f} {wz:.4f}\n"
        self.ser.write(cmd.encode())

    def stop(self):
        """정지 명령"""
        self.ser.write(b"S\n")

    def close(self):
        self.stop()
        self.ser.close()


class ROSController:
    """ROS 토픽 발행"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool

        rclpy.init()
        self.node = rclpy.create_node('teleop_node')
        self.pub_cmd = self.node.create_publisher(Twist, '/control/cmd_vel', 10)
        self.pub_enable = self.node.create_publisher(Bool, '/control/enable_drive', 10)
        self.Twist = Twist
        self.Bool = Bool
        self.rclpy = rclpy
        print("[ROS] Teleop node started")

    def send_velocity(self, vx, vy, wz):
        msg = self.Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub_cmd.publish(msg)
        self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0)

    def set_enable(self, enable):
        msg = self.Bool()
        msg.data = enable
        self.pub_enable.publish(msg)
        status = "ENABLED" if enable else "DISABLED"
        print(f"[ROS] Drive {status}")

    def close(self):
        self.stop()
        self.node.destroy_node()
        self.rclpy.shutdown()


class TeleopController:
    """통합 Teleop 컨트롤러"""

    def __init__(self, controller, is_ros=True):
        self.ctrl = controller
        self.is_ros = is_ros

        # 속도 설정
        self.linear_speed = 0.03   # m/s
        self.angular_speed = 0.1  # rad/s
        self.speed_step = 0.005

        # 현재 속도
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def set_enable(self, enable):
        if self.is_ros:
            self.ctrl.set_enable(enable)
        else:
            print("[Direct] Enable/Disable not applicable")

    def send_cmd(self):
        self.ctrl.send_velocity(self.vx, self.vy, self.wz)

    def stop(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.ctrl.stop()

    def process_key(self, key):
        """키 처리. 종료 시 False 반환"""

        # 전진/후진
        if key in ('w', 'W', KEY_UP):
            self.vx = self.linear_speed
            self.vy = 0.0
            self.wz = 0.0
        elif key in ('s', 'S', KEY_DOWN):
            self.vx = -self.linear_speed
            self.vy = 0.0
            self.wz = 0.0

        # 좌/우 이동 (스트레이프)
        elif key in ('a', 'A', KEY_LEFT):
            self.vx = 0.0
            self.vy = -self.linear_speed
            self.wz = 0.0
        elif key in ('d', 'D', KEY_RIGHT):
            self.vx = 0.0
            self.vy = self.linear_speed
            self.wz = 0.0

        # 회전
        elif key == 'q':
            self.vx = 0.0
            self.vy = 0.0
            self.wz = -self.angular_speed
        elif key == 'e':
            self.vx = 0.0
            self.vy = 0.0
            self.wz = self.angular_speed

        # 정지
        elif key == ' ':
            self.stop()
            return True

        # Enable/Disable (ROS only)
        elif key == '1':
            self.set_enable(True)
            return True
        elif key == '2':
            self.set_enable(False)
            self.stop()
            return True

        # 속도 조절
        elif key in ('+', '='):
            self.linear_speed = min(0.05, self.linear_speed + self.speed_step)
            self.angular_speed = min(0.2, self.angular_speed + 0.01)
            print(f"Speed: linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}")
            return True
        elif key == '-':
            self.linear_speed = max(0.005, self.linear_speed - self.speed_step)
            self.angular_speed = max(0.02, self.angular_speed - 0.01)
            print(f"Speed: linear={self.linear_speed:.3f}, angular={self.angular_speed:.3f}")
            return True

        # 종료
        elif key == '\x03':  # Ctrl+C
            self.stop()
            if self.is_ros:
                self.set_enable(False)
            return False

        else:
            return True

        self.send_cmd()
        return True

    def close(self):
        self.ctrl.close()


def get_key(timeout=0.1):
    """키보드 입력 읽기 (방향키 지원)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)

        if rlist:
            key = sys.stdin.read(1)

            # 방향키 처리 (ESC [ A/B/C/D)
            if key == '\x1b':
                extra = sys.stdin.read(2)
                if extra[0] == '[':
                    return extra[1]  # A, B, C, D
            return key
        return None

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_help(mode):
    print("\n" + "="*50)
    print(f"       RC Car Teleop Control [{mode}]")
    print("="*50)
    print("""
  W/↑ : Forward      S/↓ : Backward
  A/← : Strafe Left  D/→ : Strafe Right
  Q   : Rotate CCW   E   : Rotate CW

  Space : Stop
  1     : Enable drive (ROS only)
  2     : Disable drive (ROS only)

  +/=   : Speed up
  -     : Speed down

  Ctrl+C : Quit
""")
    print("="*50 + "\n")


def main():
    parser = argparse.ArgumentParser(description='RC Car Teleop Control')
    parser.add_argument('--ros', action='store_true',
                        help='Use ROS mode instead of direct Arduino')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Arduino serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Serial baudrate (default: 115200)')
    args = parser.parse_args()

    # 컨트롤러 선택 (기본: Direct 모드)
    if args.ros:
        try:
            ctrl = ROSController()
            teleop = TeleopController(ctrl, is_ros=True)
            print_help("ROS")
            print("Press 1 to enable drive first!")
        except Exception as e:
            print(f"Error: ROS init failed: {e}")
            print("Try direct mode: python3 teleop.py")
            sys.exit(1)
    else:
        try:
            ctrl = DirectArduinoController(args.port, args.baudrate)
            teleop = TeleopController(ctrl, is_ros=False)
            print_help("DIRECT")
        except Exception as e:
            print(f"Error: Failed to connect Arduino: {e}")
            print("Check port and try: python3 teleop.py --port /dev/ttyUSB0")
            sys.exit(1)

    try:
        while True:
            key = get_key(0.1)

            if key is not None:
                if not teleop.process_key(key):
                    break
            else:
                # 키를 떼면 정지
                if teleop.vx != 0 or teleop.vy != 0 or teleop.wz != 0:
                    teleop.stop()

    except KeyboardInterrupt:
        pass
    finally:
        teleop.close()
        print("\nTeleop stopped.")


if __name__ == '__main__':
    main()
