#!/usr/bin/env python3
"""
mission_manager_node.py

미션 FSM + 경로 플래너

상태:
  IDLE → DRIVE → STOP_AT_MARKER → ADVANCE_TO_CENTER → ALIGN_TO_MARKER 
       → STOP_BUMP → TURNING → DRIVE → ... → PARK → FINISH

토픽:
  Subscribe:
    - /server/task_cmd (TaskCmd) - 서버에서 미션 수신
    - /perception/marker_status (MarkerStatus) - 마커 인식
    - /mission/align_done (Bool) - 정렬 완료 신호
    - /imu/data (Imu) - IMU (턴 각도 추적)
  
  Publish:
    - /mission/state (String) - 현재 상태
    - /mission/turn_target_rad (Float32) - 턴 목표 각도
    - /control/enable_drive (Bool) - 주행 활성화
    - /server/task_status (TaskStatus) - 서버에 상태 보고
"""

import math
import time
import threading
from typing import Optional, List, Dict
from dataclasses import dataclass, field
from enum import Enum
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Imu

from rc_interfaces.msg import TaskCmd, TaskStatus, MarkerStatus


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class MissionState(Enum):
    IDLE = "IDLE"
    DRIVE = "DRIVE"
    STOP_AT_MARKER = "STOP_AT_MARKER"
    ADVANCE_TO_CENTER = "ADVANCE_TO_CENTER"
    ALIGN_TO_MARKER = "ALIGN_TO_MARKER"
    STOP_BUMP = "STOP_BUMP"
    TURNING = "TURNING"
    PARK = "PARK"
    FINISH = "FINISH"
    ERROR = "ERROR"


@dataclass
class MarkerMapEntry:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass 
class MissionContext:
    """미션 컨텍스트"""
    route_ids: List[int] = field(default_factory=list)
    route_idx: int = 0
    goal_id: int = -1
    
    current_marker_id: int = -1
    target_marker_id: int = -1
    
    turn_target_rad: float = 0.0
    turn_start_yaw: float = 0.0
    turn_accumulated: float = 0.0
    
    stop_bump_start_t: float = 0.0
    
    task_id: str = ""


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')
        
        # ===== Parameters =====
        self.declare_parameter('task_cmd_topic', '/server/task_cmd')
        self.declare_parameter('task_status_topic', '/server/task_status')
        self.declare_parameter('marker_status_topic', '/perception/marker_status')
        self.declare_parameter('align_done_topic', '/mission/align_done')
        self.declare_parameter('imu_topic', '/imu/data')
        
        self.declare_parameter('state_topic', '/mission/state')
        self.declare_parameter('turn_target_topic', '/mission/turn_target_rad')
        self.declare_parameter('enable_drive_topic', '/control/enable_drive')
        
        self.declare_parameter('marker_map_yaml', '')
        self.declare_parameter('map_unit_scale', 0.01)  # cm -> m
        
        # 마커 도착 판정
        self.declare_parameter('marker_arrive_dist', 0.20)  # 20cm 이내면 도착
        self.declare_parameter('marker_min_quality', 0.3)
        
        # Stop bump (관성 안정화)
        self.declare_parameter('stop_bump_duration_sec', 0.12)
        
        # Turn 판정 threshold (rad)
        self.declare_parameter('turn_straight_thresh', 0.3)    # ±17°
        self.declare_parameter('turn_uturn_thresh', 2.8)       # ±160°
        
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('state_publish_rate_hz', 5.0)
        
        # Load params
        self.marker_arrive_dist = self.get_parameter('marker_arrive_dist').value
        self.marker_min_quality = self.get_parameter('marker_min_quality').value
        self.stop_bump_duration = self.get_parameter('stop_bump_duration_sec').value
        self.turn_straight_thresh = self.get_parameter('turn_straight_thresh').value
        self.turn_uturn_thresh = self.get_parameter('turn_uturn_thresh').value
        
        # State
        self._lock = threading.Lock()
        self._state = MissionState.IDLE
        self._ctx = MissionContext()
        
        # Marker map
        self.marker_map: Dict[int, MarkerMapEntry] = {}
        self._load_marker_map()
        
        # Latest sensor data
        self._marker: Optional[MarkerStatus] = None
        self._imu_yaw: float = 0.0
        self._align_done: bool = False
        
        # Publishers
        self.pub_state = self.create_publisher(
            String,
            self.get_parameter('state_topic').value,
            10
        )
        self.pub_turn = self.create_publisher(
            Float32,
            self.get_parameter('turn_target_topic').value,
            10
        )
        self.pub_enable = self.create_publisher(
            Bool,
            self.get_parameter('enable_drive_topic').value,
            10
        )
        self.pub_task_status = self.create_publisher(
            TaskStatus,
            self.get_parameter('task_status_topic').value,
            10
        )
        
        # Subscribers
        self.sub_task = self.create_subscription(
            TaskCmd,
            self.get_parameter('task_cmd_topic').value,
            self.cb_task_cmd,
            10
        )
        self.sub_marker = self.create_subscription(
            MarkerStatus,
            self.get_parameter('marker_status_topic').value,
            self.cb_marker,
            10
        )
        self.sub_align = self.create_subscription(
            Bool,
            self.get_parameter('align_done_topic').value,
            self.cb_align_done,
            10
        )
        self.sub_imu = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.cb_imu,
            qos_profile_sensor_data
        )
        
        # Timers
        update_period = 1.0 / max(1.0, self.get_parameter('update_rate_hz').value)
        state_period = 1.0 / max(1.0, self.get_parameter('state_publish_rate_hz').value)
        
        self.timer_update = self.create_timer(update_period, self.update_fsm)
        self.timer_state = self.create_timer(state_period, self.publish_state)
        
        self.get_logger().info(
            f"mission_manager_node started\n"
            f"  marker_map entries: {len(self.marker_map)}"
        )
    
    def _load_marker_map(self):
        """marker_map.yaml 로드"""
        path = self.get_parameter('marker_map_yaml').value
        if not path:
            self.get_logger().warn("marker_map_yaml not specified")
            return
        
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            
            scale = self.get_parameter('map_unit_scale').value
            markers = data.get('markers', [])
            
            for m in markers:
                mid = m.get('id')
                if mid is None:
                    continue
                
                x = m.get('x', 0) * scale
                y = m.get('y', 0) * scale
                yaw = m.get('yaw', 0)
                
                self.marker_map[mid] = MarkerMapEntry(x=x, y=y, yaw=yaw)
            
            self.get_logger().info(f"Loaded marker_map: {len(self.marker_map)} entries")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load marker_map: {e}")
    
    # ===== Callbacks =====
    
    def cb_task_cmd(self, msg: TaskCmd):
        """서버에서 미션 수신"""
        with self._lock:
            if self._state != MissionState.IDLE:
                self.get_logger().warn("Task received but not IDLE, ignoring")
                return
            
            self._ctx = MissionContext()
            self._ctx.route_ids = list(msg.route_ids)
            self._ctx.goal_id = msg.goal_id
            self._ctx.task_id = msg.task_id
            self._ctx.route_idx = 0
            
            if len(self._ctx.route_ids) > 0:
                self._ctx.target_marker_id = self._ctx.route_ids[0]
                self._state = MissionState.DRIVE
                self._publish_enable(True)
                self.get_logger().info(
                    f"Task started: route={self._ctx.route_ids}, goal={self._ctx.goal_id}"
                )
            else:
                self.get_logger().warn("Empty route received")
    
    def cb_marker(self, msg: MarkerStatus):
        with self._lock:
            self._marker = msg
            if msg.valid:
                self._ctx.current_marker_id = msg.id
    
    def cb_align_done(self, msg: Bool):
        with self._lock:
            self._align_done = msg.data
    
    def cb_imu(self, msg: Imu):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self._imu_yaw = math.atan2(siny, cosy)
    
    # ===== FSM Update =====
    
    def update_fsm(self):
        with self._lock:
            state = self._state
            
            if state == MissionState.IDLE:
                pass
            elif state == MissionState.DRIVE:
                self._update_drive()
            elif state == MissionState.STOP_AT_MARKER:
                self._update_stop_at_marker()
            elif state == MissionState.ADVANCE_TO_CENTER:
                self._update_advance()
            elif state == MissionState.ALIGN_TO_MARKER:
                self._update_align()
            elif state == MissionState.STOP_BUMP:
                self._update_stop_bump()
            elif state == MissionState.TURNING:
                self._update_turning()
            elif state == MissionState.PARK:
                self._update_park()
            elif state == MissionState.FINISH:
                pass
            elif state == MissionState.ERROR:
                pass
    
    def _update_drive(self):
        """DRIVE: 차선 추종, 마커 도착 감지"""
        if self._marker is None or not self._marker.valid:
            return
        
        # 목표 마커 도착 체크
        if self._marker.id == self._ctx.target_marker_id:
            if self._marker.rel_z <= self.marker_arrive_dist:
                if self._marker.quality >= self.marker_min_quality:
                    self.get_logger().info(f"Arrived at marker {self._marker.id}")
                    self._state = MissionState.STOP_AT_MARKER
                    self._publish_enable(False)
    
    def _update_stop_at_marker(self):
        """STOP_AT_MARKER: 잠시 정지 후 ADVANCE"""
        # 바로 ADVANCE로 전환 (또는 짧은 대기)
        self._state = MissionState.ADVANCE_TO_CENTER
        self._publish_enable(True)
    
    def _update_advance(self):
        """ADVANCE_TO_CENTER: 마커 중앙으로 전진"""
        # control_stack이 처리, align_done 대기
        # 또는 거리 기반으로 전환
        if self._marker and self._marker.valid:
            if self._marker.rel_z <= 0.15:  # 15cm
                self._state = MissionState.ALIGN_TO_MARKER
    
    def _update_align(self):
        """ALIGN_TO_MARKER: 정밀 정렬"""
        if self._align_done:
            self._align_done = False
            self.get_logger().info("Align done")
            self._state = MissionState.STOP_BUMP
            self._ctx.stop_bump_start_t = time.time()
            self._publish_enable(False)
    
    def _update_stop_bump(self):
        """STOP_BUMP: 관성 안정화"""
        elapsed = time.time() - self._ctx.stop_bump_start_t
        if elapsed >= self.stop_bump_duration:
            # 다음 행동 결정
            self._decide_next_action()
    
    def _decide_next_action(self):
        """다음 행동 결정: TURNING or PARK or FINISH"""
        self._ctx.route_idx += 1
        
        if self._ctx.route_idx >= len(self._ctx.route_ids):
            # 경로 끝
            if self._ctx.goal_id >= 16:  # 주차칸 마커
                self._state = MissionState.PARK
                self._publish_enable(True)
                self.get_logger().info("Starting PARK")
            else:
                self._state = MissionState.FINISH
                self._publish_task_status("FINISHED")
                self.get_logger().info("Mission FINISH")
            return
        
        # 다음 마커로 턴
        next_id = self._ctx.route_ids[self._ctx.route_idx]
        turn_angle = self._compute_turn_angle(self._ctx.current_marker_id, next_id)
        
        self._ctx.target_marker_id = next_id
        self._ctx.turn_target_rad = turn_angle
        self._ctx.turn_start_yaw = self._imu_yaw
        self._ctx.turn_accumulated = 0.0
        
        if abs(turn_angle) < self.turn_straight_thresh:
            # 직진
            self._state = MissionState.DRIVE
            self._publish_enable(True)
            self.get_logger().info(f"Go straight to marker {next_id}")
        else:
            # 턴
            self._state = MissionState.TURNING
            self._publish_turn(turn_angle)
            self._publish_enable(True)
            self.get_logger().info(f"Turning {math.degrees(turn_angle):.1f}° to marker {next_id}")
    
    def _compute_turn_angle(self, from_id: int, to_id: int) -> float:
        """두 마커 사이 턴 각도 계산"""
        from_m = self.marker_map.get(from_id)
        to_m = self.marker_map.get(to_id)
        
        if from_m is None or to_m is None:
            return 0.0
        
        # 목표 방향
        dx = to_m.x - from_m.x
        dy = to_m.y - from_m.y
        target_yaw = math.atan2(dy, dx)
        
        # 현재 로봇 방향 (마커 yaw 기준)
        current_yaw = from_m.yaw
        
        # 턴 각도
        turn = wrap_angle(target_yaw - current_yaw)
        
        return turn
    
    def _update_turning(self):
        """TURNING: IMU 기반 턴 완료 감지"""
        # 현재 yaw와 시작 yaw 차이
        delta = wrap_angle(self._imu_yaw - self._ctx.turn_start_yaw)
        
        # 목표 도달 체크
        remaining = abs(self._ctx.turn_target_rad) - abs(delta)
        
        if remaining <= 0.1:  # ~6°
            self._state = MissionState.DRIVE
            self.get_logger().info("Turn complete")
    
    def _update_park(self):
        """PARK: control_stack이 처리"""
        # 완료 조건은 control_stack에서 별도 토픽으로 알림
        pass
    
    # ===== Publishers =====
    
    def publish_state(self):
        with self._lock:
            msg = String()
            msg.data = self._state.value
        self.pub_state.publish(msg)
    
    def _publish_enable(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_enable.publish(msg)
    
    def _publish_turn(self, angle: float):
        msg = Float32()
        msg.data = float(angle)
        self.pub_turn.publish(msg)
    
    def _publish_task_status(self, status: str):
        msg = TaskStatus()
        msg.task_id = self._ctx.task_id
        msg.status = status
        msg.current_marker_id = self._ctx.current_marker_id
        self.pub_task_status.publish(msg)


def main():
    rclpy.init()
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
