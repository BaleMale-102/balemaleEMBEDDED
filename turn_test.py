#!/usr/bin/env python3
"""
turn_test.py - 90도 턴 테스트 (IMU 기반)

사용법:
  python3 turn_test.py              # 왼쪽 90도
  python3 turn_test.py right        # 오른쪽 90도
  python3 turn_test.py left 45      # 왼쪽 45도
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
import math


class TurnTest(Node):
    def __init__(self, direction: str, angle_deg: float):
        super().__init__('turn_test')
        
        self.direction = direction
        self.angle_deg = angle_deg
        self.angle_rad = math.radians(angle_deg)
        
        # Publishers
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_enable = self.create_publisher(Bool, '/control/enable_drive', 10)
        self.pub_turn = self.create_publisher(Float32, '/mission/turn_target_rad', 10)
        
        # Subscriber - 턴 완료 감지
        self.sub_turn_done = self.create_subscription(Bool, '/mission/turn_done', self.cb_turn_done, 10)
        
        self.done = False
        
        self.get_logger().info(f"=== IMU 기반 턴 테스트 ===")
        self.get_logger().info(f"  direction: {direction}")
        self.get_logger().info(f"  angle: {angle_deg}deg")
        
        # 시작
        self.timer = self.create_timer(0.5, self.start_turn)
        self.started = False
    
    def start_turn(self):
        if self.started:
            return
        self.started = True
        self.timer.cancel()
        
        # Enable
        msg = Bool()
        msg.data = True
        self.pub_enable.publish(msg)
        
        time.sleep(0.2)
        
        # Turn target
        target = Float32()
        target.data = self.angle_rad if self.direction == 'left' else -self.angle_rad
        self.pub_turn.publish(target)
        
        # TURNING 모드
        state = String()
        state.data = 'TURNING'
        self.pub_state.publish(state)
        
        self.get_logger().info(f"턴 시작! ({self.direction} {self.angle_deg}도)")
        self.start_time = time.time()
    
    def cb_turn_done(self, msg):
        if msg.data and not self.done:
            self.done = True
            elapsed = time.time() - self.start_time
            
            # IDLE로 전환
            state = String()
            state.data = 'IDLE'
            self.pub_state.publish(state)
            
            self.get_logger().info(f"")
            self.get_logger().info(f"=== 턴 완료! ===")
            self.get_logger().info(f"  각도: {self.angle_deg}deg")
            self.get_logger().info(f"  시간: {elapsed:.2f}s")
            
            # 종료
            rclpy.shutdown()


def main():
    direction = 'left'
    angle_deg = 90.0
    
    if len(sys.argv) > 1:
        if sys.argv[1].lower() in ['right', 'r']:
            direction = 'right'
        elif sys.argv[1].lower() in ['left', 'l']:
            direction = 'left'
        else:
            try:
                angle_deg = float(sys.argv[1])
            except:
                pass
    
    if len(sys.argv) > 2:
        try:
            angle_deg = float(sys.argv[2])
        except:
            pass
    
    rclpy.init()
    node = TurnTest(direction, angle_deg)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            state = String()
            state.data = 'IDLE'
            node.pub_state.publish(state)
        except:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()