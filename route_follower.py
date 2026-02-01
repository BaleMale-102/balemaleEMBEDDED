#!/usr/bin/env python3
"""
route_follower.py

Route를 따라 마커 순회 테스트

시퀀스:
1. DRIVE: 목표 마커로 이동
2. marker_reached 수신 → 다음 마커로 턴 방향 계산
3. TURNING: 라인 찾을 때까지 회전
4. turn_done 수신 → 다음 마커로 DRIVE
5. 반복

사용법:
  python3 route_follower.py --ros-args -p route:="[1,5,9]"
"""

import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32, Int32


class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')
        
        # 파라미터
        self.declare_parameter('route', [1, 5, 9])  # 기본 route
        self.declare_parameter('auto_start', False)
        self.declare_parameter('wait_after_reach_sec', 1.5)  # 도달 후 대기 시간
        self.declare_parameter('initial_heading_deg', 90.0)  # 초기 방향 (도, 0=동, 90=북, 180=서, -90=남)
        
        route_param = self.get_parameter('route').value
        self.route: List[int] = list(route_param) if route_param else [1, 5, 9]
        self.auto_start = self.get_parameter('auto_start').value
        self.wait_after_reach = self.get_parameter('wait_after_reach_sec').value
        initial_deg = self.get_parameter('initial_heading_deg').value
        
        # 마커 맵 (좌표)
        self.marker_map = {
            0: (0.60, 0.18),
            1: (0.60, 0.54),
            2: (0.18, 0.54),
            3: (1.02, 0.54),
            4: (0.18, 1.11),
            5: (0.425, 1.11),
            6: (0.545, 1.11),
            7: (0.66, 1.11),
            8: (0.78, 1.11),
            9: (1.02, 1.11),
            10: (0.18, 1.87),
            11: (0.425, 1.87),
            12: (0.545, 1.87),
            13: (0.66, 1.87),
            14: (0.78, 1.87),
            15: (1.02, 1.87),
        }
        
        # 상태
        self.current_idx = 0
        self.state = "IDLE"  # IDLE, DRIVE, WAIT_BEFORE_TURN, TURNING, DONE
        self.current_heading = math.radians(initial_deg)  # 초기 heading
        self.enabled = False
        self.prev_marker_id = -1  # 이전 마커 (heading 계산용)
        self.wait_start_time = 0.0  # 대기 시작 시간
        
        self.get_logger().info(f"Initial heading: {initial_deg}°")
        
        # Publishers
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_target = self.create_publisher(Int32, '/mission/target_marker', 10)
        self.pub_turn = self.create_publisher(Float32, '/mission/turn_target_rad', 10)
        self.pub_enable = self.create_publisher(Bool, '/control/enable_drive', 10)
        
        # Subscribers
        self.sub_reached = self.create_subscription(Int32, '/mission/marker_reached', self.cb_marker_reached, 10)
        self.sub_turn_done = self.create_subscription(Bool, '/mission/turn_done', self.cb_turn_done, 10)
        self.sub_enable = self.create_subscription(Bool, '/route/enable', self.cb_enable, 10)
        
        # Timer (상태 출력용 + 대기 처리)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz로 변경
        
        self.get_logger().info(f"RouteFollower started")
        self.get_logger().info(f"  Route: {self.route}")
        self.get_logger().info(f"  Publish to /route/enable (Bool) to start")
        
        if self.auto_start:
            self.start_route()
    
    def cb_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.start_route()
        elif not msg.data and self.enabled:
            self.stop_route()
    
    def start_route(self):
        self.enabled = True
        self.current_idx = 0
        self.state = "DRIVE"
        
        # 드라이브 활성화
        enable_msg = Bool()
        enable_msg.data = True
        self.pub_enable.publish(enable_msg)
        
        # 첫 마커로 이동
        self._start_drive()
        
        self.get_logger().info(f"Route started! First target: {self.route[0]}")
    
    def stop_route(self):
        self.enabled = False
        self.state = "IDLE"
        
        # 정지
        state_msg = String()
        state_msg.data = "IDLE"
        self.pub_state.publish(state_msg)
        
        enable_msg = Bool()
        enable_msg.data = False
        self.pub_enable.publish(enable_msg)
        
        self.get_logger().info("Route stopped")
    
    def _start_drive(self):
        """현재 목표 마커로 DRIVE 시작"""
        if self.current_idx >= len(self.route):
            self.state = "DONE"
            self.get_logger().info("Route complete!")
            self.stop_route()
            return
        
        target = self.route[self.current_idx]
        
        # 목표 마커 발행
        target_msg = Int32()
        target_msg.data = target
        self.pub_target.publish(target_msg)
        
        # DRIVE 모드
        state_msg = String()
        state_msg.data = "DRIVE"
        self.pub_state.publish(state_msg)
        
        self.state = "DRIVE"
        self.get_logger().info(f"DRIVE to marker {target} ({self.current_idx + 1}/{len(self.route)})")
    
    def cb_marker_reached(self, msg: Int32):
        """마커 도달 콜백"""
        if not self.enabled:
            return
        
        if self.state != "DRIVE":
            return
        
        reached_id = msg.data
        target_id = self.route[self.current_idx]
        
        if reached_id != target_id:
            self.get_logger().warn(f"Reached {reached_id} but expected {target_id}")
            return
        
        self.get_logger().info(f"Reached marker {reached_id}!")
        
        # 현재 heading 업데이트 (이전 마커 → 현재 마커 방향)
        if self.prev_marker_id >= 0 and self.prev_marker_id in self.marker_map and reached_id in self.marker_map:
            prev_pos = self.marker_map[self.prev_marker_id]
            curr_pos = self.marker_map[reached_id]
            dx = curr_pos[0] - prev_pos[0]
            dy = curr_pos[1] - prev_pos[1]
            if abs(dx) > 0.01 or abs(dy) > 0.01:  # 의미있는 이동이면
                self.current_heading = math.atan2(dy, dx)
                self.get_logger().info(f"Updated heading: {math.degrees(self.current_heading):.0f}°")
        
        self.prev_marker_id = reached_id
        
        # 다음 마커 있나?
        self.current_idx += 1
        
        if self.current_idx >= len(self.route):
            self.state = "DONE"
            self.get_logger().info("=== Route complete! ===")
            self.stop_route()
            return
        
        # 대기 상태로 전환
        self.state = "WAIT_BEFORE_TURN"
        self.wait_start_time = time.time()
        self.get_logger().info(f"Waiting {self.wait_after_reach}s before turn...")
    
    def _process_turn(self):
        """대기 후 턴 처리"""
        next_id = self.route[self.current_idx]
        turn_rad = self._calc_turn_angle(self.prev_marker_id, next_id)
        
        self.get_logger().info(f"Turn calculation: {self.prev_marker_id} → {next_id}, angle={math.degrees(turn_rad):.0f}°")
        
        if abs(turn_rad) > 0.1:  # 약 6도 이상이면 턴
            self._start_turn(turn_rad)
        else:
            # 턴 불필요 → 바로 다음 마커로
            self.get_logger().info("Turn not needed, going straight")
            self._start_drive()
    
    def _calc_turn_angle(self, from_id: int, to_id: int) -> float:
        """두 마커 간 턴 각도 계산"""
        if from_id not in self.marker_map or to_id not in self.marker_map:
            self.get_logger().warn(f"Marker {from_id} or {to_id} not in map")
            return 0.0
        
        from_pos = self.marker_map[from_id]
        to_pos = self.marker_map[to_id]
        
        # 목표 방향
        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]
        target_heading = math.atan2(dy, dx)
        
        # 필요한 회전량 (현재 heading 기준)
        turn = target_heading - self.current_heading
        
        # -pi ~ pi 정규화
        while turn > math.pi:
            turn -= 2 * math.pi
        while turn < -math.pi:
            turn += 2 * math.pi
        
        self.get_logger().info(f"Heading: current={math.degrees(self.current_heading):.0f}° target={math.degrees(target_heading):.0f}° turn={math.degrees(turn):.0f}°")
        
        # heading은 턴 시작 시 업데이트 (턴 완료 후 이 방향을 바라봄)
        self.current_heading = target_heading
        
        return turn
    
    def _start_turn(self, turn_rad: float):
        """턴 시작"""
        self.state = "TURNING"
        
        # 턴 각도 발행
        turn_msg = Float32()
        turn_msg.data = float(turn_rad)
        self.pub_turn.publish(turn_msg)
        
        # TURNING 모드
        state_msg = String()
        state_msg.data = "TURNING"
        self.pub_state.publish(state_msg)
        
        direction = "LEFT" if turn_rad > 0 else "RIGHT"
        self.get_logger().info(f"TURNING {direction} {math.degrees(turn_rad):.0f}°")
    
    def cb_turn_done(self, msg: Bool):
        """턴 완료 콜백"""
        if not self.enabled:
            return
        
        if self.state != "TURNING":
            return
        
        if msg.data:
            self.get_logger().info("Turn complete!")
            # 다음 마커로 DRIVE
            self._start_drive()
    
    def timer_callback(self):
        """대기 상태 처리 + 상태 출력"""
        if not self.enabled:
            return
        
        # WAIT_BEFORE_TURN 상태 처리
        if self.state == "WAIT_BEFORE_TURN":
            elapsed = time.time() - self.wait_start_time
            if elapsed >= self.wait_after_reach:
                # 대기 완료 → 턴 시작
                self._process_turn()
            else:
                # 대기 중 (1초마다 로그)
                if int(elapsed * 10) % 10 == 0:
                    self.get_logger().info(f"Waiting... {elapsed:.1f}/{self.wait_after_reach}s")
            return
        
        # 상태 출력 (1초마다)
        if hasattr(self, '_last_print') and time.time() - self._last_print < 1.0:
            return
        self._last_print = time.time()
        
        target = self.route[self.current_idx] if self.current_idx < len(self.route) else -1
        self.get_logger().info(f"[{self.state}] idx={self.current_idx}/{len(self.route)} target={target}")


def main():
    rclpy.init()
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
