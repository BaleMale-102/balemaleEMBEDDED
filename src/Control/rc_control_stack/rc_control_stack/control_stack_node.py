#!/usr/bin/env python3
"""
control_stack_node.py (v4 - Marker Memory + Target Filtering)

작은 맵(120x205cm) 최적화
- 목표 마커 ID 필터링
- 마커 방향 기억 (시야 이탈 시 dead reckoning)
- EKF 위치 기반 목표 도달 체크

주행 우선순위:
  1. 목표 마커 보임 → 마커 추종 + 방향 저장
  2. 목표 마커 안 보임 + 최근에 봤음 → 기억된 방향으로 계속
  3. 라인 있음 → 라인 추종
  4. 아무것도 없음 → IMU 직진
"""

import math
import time
import threading
from typing import Optional
from dataclasses import dataclass, field
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Bool, Float32, Int32
from geometry_msgs.msg import PoseStamped

from rc_interfaces.msg import (
    DriveCmd, DrivingState,
    LaneStatus, MarkerStatus, ParkingLineStatus
)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class ControlMode(Enum):
    IDLE = "IDLE"
    LANE_FOLLOW = "LANE_FOLLOW"
    MARKER_FOLLOW = "MARKER_FOLLOW"
    TURNING = "TURNING"
    ADVANCE_TO_CENTER = "ADVANCE_TO_CENTER"
    ALIGN_TO_MARKER = "ALIGN_TO_MARKER"
    PARK = "PARK"


class ParkPhase(Enum):
    IDLE = "IDLE"
    LINE_ALIGN = "LINE_ALIGN"
    SLOT_MARKER_ALIGN = "SLOT_MARKER_ALIGN"
    DONE = "DONE"


@dataclass
class ControllerState:
    mode: ControlMode = ControlMode.IDLE
    park_phase: ParkPhase = ParkPhase.IDLE
    
    # 에러
    err_x: float = 0.0
    err_y: float = 0.0
    err_yaw: float = 0.0
    
    # 목표 마커 (mission에서 설정)
    target_marker_id: int = -1
    
    # 관측 마커
    observed_marker_id: int = -1
    marker_quality: float = 0.0
    marker_rel_x: float = 0.0
    marker_rel_z: float = 0.0
    marker_rel_yaw: float = 0.0
    
    # Lane
    lane_offset: float = 0.0
    lane_angle: float = 0.0
    lane_quality: float = 0.0
    
    # 턴
    turn_target_rad: float = 0.0
    
    # 타임스탬프
    last_lane_t: float = 0.0
    last_marker_t: float = 0.0
    last_target_marker_t: float = 0.0  # 목표 마커 본 시간
    
    # ===== 마커 기억 (시야 이탈 대비) =====
    memory_vx: float = 0.0       # 기억된 전진 속도
    memory_wz: float = 0.0       # 기억된 각속도 보정
    memory_marker_id: int = -1   # 기억된 마커 ID
    
    # EKF 위치 (localization에서 받음)
    ekf_x: float = 0.0
    ekf_y: float = 0.0
    ekf_yaw: float = 0.0
    last_ekf_t: float = 0.0


class ControlStackNode(Node):
    def __init__(self):
        super().__init__('control_stack_node')
        
        # ===== Parameters =====
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('target_marker_topic', '/mission/target_marker')  # NEW
        self.declare_parameter('turn_target_topic', '/mission/turn_target_rad')
        self.declare_parameter('enable_drive_topic', '/control/enable_drive')
        self.declare_parameter('lane_topic', '/perception/lane')
        self.declare_parameter('marker_status_topic', '/perception/marker_status')
        self.declare_parameter('parking_line_topic', '/perception/parking_line')
        self.declare_parameter('slot_marker_topic', '/perception/slot_marker_pose')
        self.declare_parameter('ekf_pose_topic', '/localization/pose')  # NEW
        
        self.declare_parameter('drive_cmd_topic', '/control/drive_cmd')
        self.declare_parameter('driving_state_topic', '/driving/state')
        self.declare_parameter('align_done_topic', '/mission/align_done')
        self.declare_parameter('marker_reached_topic', '/mission/marker_reached')  # NEW
        
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('state_publish_rate_hz', 10.0)
        
        # Timeout
        self.declare_parameter('lane_timeout_sec', 1.0)
        self.declare_parameter('marker_timeout_sec', 2.0)
        self.declare_parameter('marker_memory_timeout_sec', 2.5)  # 기억 유지 시간
        self.declare_parameter('ekf_timeout_sec', 0.5)
        
        # ===== 속도 (작은 맵용) =====
        self.declare_parameter('lane_vx', 0.03)
        self.declare_parameter('lane_k_psi', 0.5)
        self.declare_parameter('lane_k_y', 0.8)
        self.declare_parameter('lane_v0', 0.05)
        
        self.declare_parameter('marker_vx', 0.025)
        self.declare_parameter('marker_kp_x', 0.3)
        self.declare_parameter('marker_kp_yaw', 0.4)
        
        self.declare_parameter('memory_vx', 0.02)   # 기억 모드 속도 (더 느림)
        
        self.declare_parameter('blind_vx', 0.015)
        
        # Turning
        self.declare_parameter('turn_wz', 0.3)
        self.declare_parameter('turn_tolerance_rad', 0.08)
        
        # Advance
        self.declare_parameter('advance_vx', 0.025)
        self.declare_parameter('advance_target_z', 0.10)
        self.declare_parameter('advance_tolerance_z', 0.02)
        
        # Align
        self.declare_parameter('align_kp_x', 0.5)
        self.declare_parameter('align_kp_y', 0.4)
        self.declare_parameter('align_kp_yaw', 0.3)
        self.declare_parameter('align_tolerance_xy', 0.015)
        self.declare_parameter('align_tolerance_yaw', 0.06)
        self.declare_parameter('align_max_v', 0.04)
        
        # 목표 도달 판정 (EKF 기반)
        self.declare_parameter('goal_reach_distance', 0.15)  # 15cm 이내면 도달
        
        # Park
        self.declare_parameter('park_line_vy', 0.025)
        self.declare_parameter('park_line_kp_offset', 0.3)
        self.declare_parameter('park_line_kp_angle', 0.2)
        self.declare_parameter('park_line_tolerance_offset', 0.03)
        self.declare_parameter('park_line_tolerance_angle', 0.08)
        
        self.declare_parameter('park_slot_vx', 0.02)
        self.declare_parameter('park_slot_target_x', 0.08)
        self.declare_parameter('park_slot_tolerance', 0.015)
        
        # Safety
        self.declare_parameter('max_vx', 0.05)
        self.declare_parameter('max_vy', 0.05)
        self.declare_parameter('max_wz', 0.5)
        
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('log_throttle_sec', 1.0)
        
        self._load_params()
        self._load_marker_map()
        
        # State
        self._lock = threading.Lock()
        self._state = ControllerState()
        self._enable_drive = False
        self._mission_state = "IDLE"
        self._last_log_t = 0.0
        
        # Perception
        self._lane: Optional[LaneStatus] = None
        self._marker: Optional[MarkerStatus] = None
        self._parking_line: Optional[ParkingLineStatus] = None
        self._slot_marker: Optional[PoseStamped] = None
        
        # Publishers
        self.pub_drive = self.create_publisher(DriveCmd, self.get_parameter('drive_cmd_topic').value, 10)
        self.pub_driving_state = self.create_publisher(DrivingState, self.get_parameter('driving_state_topic').value, 10)
        self.pub_align_done = self.create_publisher(Bool, self.get_parameter('align_done_topic').value, 10)
        self.pub_marker_reached = self.create_publisher(Int32, self.get_parameter('marker_reached_topic').value, 10)
        
        # Subscribers
        self.sub_mission = self.create_subscription(String, self.get_parameter('mission_state_topic').value, self.cb_mission_state, 10)
        self.sub_target = self.create_subscription(Int32, self.get_parameter('target_marker_topic').value, self.cb_target_marker, 10)
        self.sub_turn = self.create_subscription(Float32, self.get_parameter('turn_target_topic').value, self.cb_turn_target, 10)
        self.sub_enable = self.create_subscription(Bool, self.get_parameter('enable_drive_topic').value, self.cb_enable, 10)
        self.sub_lane = self.create_subscription(LaneStatus, self.get_parameter('lane_topic').value, self.cb_lane, qos_profile_sensor_data)
        self.sub_marker = self.create_subscription(MarkerStatus, self.get_parameter('marker_status_topic').value, self.cb_marker, 10)
        self.sub_parking_line = self.create_subscription(ParkingLineStatus, self.get_parameter('parking_line_topic').value, self.cb_parking_line, qos_profile_sensor_data)
        self.sub_slot = self.create_subscription(PoseStamped, self.get_parameter('slot_marker_topic').value, self.cb_slot_marker, 10)
        self.sub_ekf = self.create_subscription(PoseStamped, self.get_parameter('ekf_pose_topic').value, self.cb_ekf_pose, 10)
        
        # Timers
        ctrl_period = 1.0 / max(1.0, self.get_parameter('control_rate_hz').value)
        state_period = 1.0 / max(1.0, self.get_parameter('state_publish_rate_hz').value)
        
        self.timer_control = self.create_timer(ctrl_period, self.control_loop)
        self.timer_state = self.create_timer(state_period, self.publish_driving_state)
        
        self.get_logger().info(f"control_stack_node v4 (marker memory) started")
    
    def _load_params(self):
        self.lane_vx = self.get_parameter('lane_vx').value
        self.lane_k_psi = self.get_parameter('lane_k_psi').value
        self.lane_k_y = self.get_parameter('lane_k_y').value
        self.lane_v0 = self.get_parameter('lane_v0').value
        
        self.marker_vx = self.get_parameter('marker_vx').value
        self.marker_kp_x = self.get_parameter('marker_kp_x').value
        self.marker_kp_yaw = self.get_parameter('marker_kp_yaw').value
        
        self.memory_vx = self.get_parameter('memory_vx').value
        self.blind_vx = self.get_parameter('blind_vx').value
        
        self.turn_wz = self.get_parameter('turn_wz').value
        self.turn_tol = self.get_parameter('turn_tolerance_rad').value
        
        self.advance_vx = self.get_parameter('advance_vx').value
        self.advance_z = self.get_parameter('advance_target_z').value
        self.advance_tol = self.get_parameter('advance_tolerance_z').value
        
        self.align_kp_x = self.get_parameter('align_kp_x').value
        self.align_kp_y = self.get_parameter('align_kp_y').value
        self.align_kp_yaw = self.get_parameter('align_kp_yaw').value
        self.align_tol_xy = self.get_parameter('align_tolerance_xy').value
        self.align_tol_yaw = self.get_parameter('align_tolerance_yaw').value
        self.align_max_v = self.get_parameter('align_max_v').value
        
        self.goal_reach_dist = self.get_parameter('goal_reach_distance').value
        
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        
        self.lane_timeout = self.get_parameter('lane_timeout_sec').value
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value
        self.memory_timeout = self.get_parameter('marker_memory_timeout_sec').value
        self.ekf_timeout = self.get_parameter('ekf_timeout_sec').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.base_frame = self.get_parameter('base_frame').value
    
    def _load_marker_map(self):
        """마커 위치 맵 (marker_map.yaml 기반, cm→m 변환됨)"""
        self.marker_positions = {
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
            # 주차칸 마커
            16: (0.425, 0.825),
            17: (0.545, 0.825),
            18: (0.66, 0.825),
            19: (0.78, 0.825),
            20: (0.425, 1.3633),
            21: (0.545, 1.3633),
            22: (0.66, 1.3633),
            23: (0.78, 1.3633),
            24: (0.425, 1.6166),
            25: (0.545, 1.6166),
            26: (0.66, 1.6166),
            27: (0.78, 1.6166),
        }
    
    def _log_throttle(self, msg: str):
        now = time.time()
        if now - self._last_log_t >= self.log_throttle:
            self._last_log_t = now
            self.get_logger().info(msg)
    
    # ===== Callbacks =====
    
    def cb_mission_state(self, msg: String):
        state = msg.data.upper()
        with self._lock:
            prev = self._mission_state
            self._mission_state = state
            
            if state == "IDLE":
                self._state.mode = ControlMode.IDLE
            elif state == "DRIVE":
                self._state.mode = ControlMode.LANE_FOLLOW
            elif state == "MARKER_FOLLOW":
                self._state.mode = ControlMode.MARKER_FOLLOW
            elif state == "TURNING":
                self._state.mode = ControlMode.TURNING
            elif state == "ADVANCE_TO_CENTER":
                self._state.mode = ControlMode.ADVANCE_TO_CENTER
            elif state == "ALIGN_TO_MARKER":
                self._state.mode = ControlMode.ALIGN_TO_MARKER
            elif state == "PARK":
                self._state.mode = ControlMode.PARK
                if self._state.park_phase == ParkPhase.IDLE:
                    self._state.park_phase = ParkPhase.LINE_ALIGN
            else:
                self._state.mode = ControlMode.IDLE
            
            if state != prev:
                self.get_logger().info(f"Mode: {prev} → {state}")
    
    def cb_target_marker(self, msg: Int32):
        """목표 마커 ID 설정"""
        with self._lock:
            prev = self._state.target_marker_id
            self._state.target_marker_id = msg.data
            if msg.data != prev:
                self.get_logger().info(f"Target marker: {prev} → {msg.data}")
                # 새 목표 설정 시 기억 초기화
                self._state.memory_marker_id = -1
                self._state.memory_wz = 0.0
    
    def cb_turn_target(self, msg: Float32):
        with self._lock:
            self._state.turn_target_rad = float(msg.data)
    
    def cb_enable(self, msg: Bool):
        with self._lock:
            self._enable_drive = msg.data
    
    def cb_lane(self, msg: LaneStatus):
        with self._lock:
            self._lane = msg
            self._state.last_lane_t = time.time()
            if msg.valid:
                self._state.lane_offset = msg.offset_norm
                self._state.lane_angle = msg.angle
                self._state.lane_quality = msg.quality
    
    def cb_marker(self, msg: MarkerStatus):
        with self._lock:
            self._marker = msg
            self._state.last_marker_t = time.time()
            
            if msg.valid:
                self._state.observed_marker_id = msg.id
                self._state.marker_quality = msg.quality
                self._state.marker_rel_x = msg.rel_x
                self._state.marker_rel_z = msg.rel_z
                self._state.marker_rel_yaw = msg.rel_yaw
                
                # 목표 마커를 봤으면 시간 기록
                if msg.id == self._state.target_marker_id:
                    self._state.last_target_marker_t = time.time()
    
    def cb_parking_line(self, msg: ParkingLineStatus):
        with self._lock:
            self._parking_line = msg
    
    def cb_slot_marker(self, msg: PoseStamped):
        with self._lock:
            self._slot_marker = msg
    
    def cb_ekf_pose(self, msg: PoseStamped):
        """EKF 위치 수신"""
        with self._lock:
            self._state.ekf_x = msg.pose.position.x
            self._state.ekf_y = msg.pose.position.y
            # quaternion to yaw
            q = msg.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._state.ekf_yaw = math.atan2(siny, cosy)
            self._state.last_ekf_t = time.time()
    
    # ===== Control Loop =====
    
    def control_loop(self):
        with self._lock:
            if not self._enable_drive:
                self._publish_stop("disabled")
                return
            
            # 목표 도달 체크 (EKF 기반)
            if self._check_goal_reached():
                return
            
            mode = self._state.mode
            
            if mode == ControlMode.IDLE:
                self._publish_stop("idle")
            elif mode == ControlMode.LANE_FOLLOW:
                self._control_drive_hybrid()
            elif mode == ControlMode.MARKER_FOLLOW:
                self._control_drive_hybrid()
            elif mode == ControlMode.TURNING:
                self._control_turning()
            elif mode == ControlMode.ADVANCE_TO_CENTER:
                self._control_advance()
            elif mode == ControlMode.ALIGN_TO_MARKER:
                self._control_align()
            elif mode == ControlMode.PARK:
                self._control_park()
            else:
                self._publish_stop("unknown")
    
    def _check_goal_reached(self) -> bool:
        """EKF 위치로 목표 마커 도달 체크"""
        now = time.time()
        
        # EKF 유효 체크
        if (now - self._state.last_ekf_t) > self.ekf_timeout:
            return False
        
        target_id = self._state.target_marker_id
        if target_id < 0 or target_id not in self.marker_positions:
            return False
        
        # 목표 위치
        goal_x, goal_y = self.marker_positions[target_id]
        
        # 현재 위치
        curr_x = self._state.ekf_x
        curr_y = self._state.ekf_y
        
        # 거리 계산
        dist = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        
        if dist < self.goal_reach_dist:
            self._log_throttle(f"Goal reached: marker {target_id}, dist={dist:.3f}m")
            self._publish_stop("goal_reached")
            
            # marker_reached 발행
            msg = Int32()
            msg.data = target_id
            self.pub_marker_reached.publish(msg)
            return True
        
        return False
    
    def _publish_stop(self, source: str):
        cmd = DriveCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.enable = False
        cmd.vx = cmd.vy = cmd.wz = 0.0
        cmd.source = f"ctrl:{source}"
        self.pub_drive.publish(cmd)
    
    def _publish_cmd(self, vx: float, vy: float, wz: float, source: str):
        cmd = DriveCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.enable = True
        cmd.vx = float(clamp(vx, -self.max_vx, self.max_vx))
        cmd.vy = float(clamp(vy, -self.max_vy, self.max_vy))
        cmd.wz = float(clamp(wz, -self.max_wz, self.max_wz))
        cmd.source = f"ctrl:{source}"
        self.pub_drive.publish(cmd)
    
    # ===== 하이브리드 주행 (마커 우선 + 기억) =====
    
    def _control_drive_hybrid(self):
        """
        주행 우선순위:
        1. 목표 마커 보임 → 마커 추종 + 방향 기억
        2. 목표 마커 안 보임 + 최근에 봤음 → 기억된 방향으로 계속
        3. 라인 있음 → 라인 추종
        4. 아무것도 없음 → IMU 직진
        """
        now = time.time()
        
        target_id = self._state.target_marker_id
        
        # 센서 유효성
        lane_valid = (self._lane is not None and 
                      self._lane.valid and 
                      (now - self._state.last_lane_t) < self.lane_timeout)
        
        marker_valid = (self._marker is not None and 
                        self._marker.valid and 
                        (now - self._state.last_marker_t) < self.marker_timeout)
        
        # 목표 마커 보이는지
        target_visible = (marker_valid and 
                          self._state.observed_marker_id == target_id)
        
        # 목표 마커 최근에 봤는지 (기억 모드)
        memory_valid = ((now - self._state.last_target_marker_t) < self.memory_timeout and
                        self._state.memory_marker_id == target_id)
        
        # ===== 1. 목표 마커 보임 =====
        if target_visible:
            rel_x = self._state.marker_rel_x
            rel_yaw = self._state.marker_rel_yaw
            
            vx = self.marker_vx
            wz = -self.marker_kp_x * rel_x - self.marker_kp_yaw * rel_yaw
            
            # 기억 저장
            self._state.memory_vx = vx
            self._state.memory_wz = wz
            self._state.memory_marker_id = target_id
            
            self._log_throttle(f"Target {target_id}: x={rel_x:.2f} yaw={rel_yaw:.2f}")
            self._publish_cmd(vx, 0.0, wz, f"marker_{target_id}")
            return
        
        # ===== 2. 기억 모드 (목표 마커 안 보이지만 최근에 봄) =====
        if memory_valid:
            vx = self.memory_vx
            # 시간 지남에 따라 보정량 감쇠 (직진에 가깝게)
            elapsed = now - self._state.last_target_marker_t
            decay = max(0.3, 1.0 - elapsed / self.memory_timeout)
            wz = self._state.memory_wz * decay
            
            self._log_throttle(f"Memory: target={target_id}, elapsed={elapsed:.1f}s, decay={decay:.2f}")
            self._publish_cmd(vx, 0.0, wz, "memory")
            return
        
        # ===== 3. 라인 추종 =====
        if lane_valid:
            offset = self._state.lane_offset
            angle = self._state.lane_angle
            
            vx = self.lane_vx
            psi_term = self.lane_k_psi * angle
            y_term = math.atan2(self.lane_k_y * offset, vx + self.lane_v0)
            wz = psi_term + y_term
            
            self._publish_cmd(vx, 0.0, wz, "lane")
            return
        
        # ===== 4. 블라인드 직진 =====
        self._log_throttle("Blind forward (no marker, no lane)")
        self._publish_cmd(self.blind_vx, 0.0, 0.0, "blind")
    
    def _control_turning(self):
        target = self._state.turn_target_rad
        
        if abs(target) < self.turn_tol:
            self._publish_stop("turn_done")
            return
        
        wz = self.turn_wz if target > 0 else -self.turn_wz
        self._publish_cmd(0.0, 0.0, wz, "turn")
    
    def _control_advance(self):
        """마커 접근"""
        now = time.time()
        target_id = self._state.target_marker_id
        
        marker_valid = (self._marker is not None and 
                        self._marker.valid and 
                        (now - self._state.last_marker_t) < self.marker_timeout)
        
        target_visible = marker_valid and self._state.observed_marker_id == target_id
        
        if target_visible:
            dist = self._marker.rel_z
            
            if dist <= self.advance_z + self.advance_tol:
                self._publish_stop("adv_done")
                return
            
            rel_x = self._marker.rel_x
            wz = -self.marker_kp_x * rel_x
            
            self._publish_cmd(self.advance_vx, 0.0, wz, "advance")
        else:
            # 마커 안 보이면 느리게 직진
            self._publish_cmd(self.advance_vx * 0.5, 0.0, 0.0, "adv_blind")
    
    def _control_align(self):
        """마커 정밀 정렬"""
        now = time.time()
        target_id = self._state.target_marker_id
        
        marker_valid = (self._marker is not None and 
                        self._marker.valid and 
                        (now - self._state.last_marker_t) < self.marker_timeout)
        
        target_visible = marker_valid and self._state.observed_marker_id == target_id
        
        if not target_visible:
            self._log_throttle("Align: target not visible")
            self._publish_stop("align_no_target")
            return
        
        ex = self._marker.rel_x
        ey = self._marker.rel_z - self.advance_z
        eyaw = self._marker.rel_yaw
        
        vx = -self.align_kp_y * ey
        vy = -self.align_kp_x * ex
        wz = -self.align_kp_yaw * eyaw
        
        vx = clamp(vx, -self.align_max_v, self.align_max_v)
        vy = clamp(vy, -self.align_max_v, self.align_max_v)
        
        self._state.err_x = ex
        self._state.err_y = ey
        self._state.err_yaw = eyaw
        
        if (abs(ex) < self.align_tol_xy and 
            abs(ey) < self.align_tol_xy and 
            abs(eyaw) < self.align_tol_yaw):
            self._publish_stop("align_done")
            self._publish_align_done(True)
            self.get_logger().info(f"Align done: ex={ex:.3f} ey={ey:.3f} eyaw={eyaw:.3f}")
            return
        
        self._publish_cmd(vx, vy, wz, "align")
    
    def _control_park(self):
        phase = self._state.park_phase
        
        if phase == ParkPhase.LINE_ALIGN:
            self._park_line_align()
        elif phase == ParkPhase.SLOT_MARKER_ALIGN:
            self._park_slot_align()
        elif phase == ParkPhase.DONE:
            self._publish_stop("park_done")
        else:
            self._publish_stop("park_idle")
    
    def _park_line_align(self):
        if self._parking_line is None or not self._parking_line.valid:
            self._publish_stop("no_park_line")
            return
        
        offset = self._parking_line.offset_norm
        angle = self._parking_line.angle
        
        kp_off = self.get_parameter('park_line_kp_offset').value
        kp_ang = self.get_parameter('park_line_kp_angle').value
        tol_off = self.get_parameter('park_line_tolerance_offset').value
        tol_ang = self.get_parameter('park_line_tolerance_angle').value
        base_vy = self.get_parameter('park_line_vy').value
        
        vy = base_vy - kp_off * offset
        wz = -kp_ang * angle
        
        if abs(offset) < tol_off and abs(angle) < tol_ang:
            self._state.park_phase = ParkPhase.SLOT_MARKER_ALIGN
            self.get_logger().info("Park: LINE → SLOT")
            return
        
        self._publish_cmd(0.0, vy, wz, "park_line")
    
    def _park_slot_align(self):
        if self._slot_marker is None:
            self._publish_stop("no_slot")
            return
        
        slot_x = self._slot_marker.pose.position.x
        target_x = self.get_parameter('park_slot_target_x').value
        tol = self.get_parameter('park_slot_tolerance').value
        base_vx = self.get_parameter('park_slot_vx').value
        
        err = slot_x - target_x
        
        if abs(err) < tol:
            self._state.park_phase = ParkPhase.DONE
            self.get_logger().info("Park: SLOT → DONE")
            return
        
        vx = base_vx if err > 0 else -base_vx
        self._publish_cmd(vx, 0.0, 0.0, "park_slot")
    
    def _publish_align_done(self, done: bool):
        msg = Bool()
        msg.data = done
        self.pub_align_done.publish(msg)
    
    def publish_driving_state(self):
        with self._lock:
            msg = DrivingState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            
            if self._state.mode == ControlMode.PARK:
                msg.state = f"PARK_{self._state.park_phase.value}"
            else:
                msg.state = self._state.mode.value
            
            msg.target_marker_id = self._state.target_marker_id
            msg.observed_marker_id = self._state.observed_marker_id
            msg.ex = float(self._state.err_x)
            msg.ey = float(self._state.err_y)
            msg.eyaw = float(self._state.err_yaw)
            msg.confidence = float(self._state.marker_quality)
            
            # 기억 모드 상태
            now = time.time()
            memory_age = now - self._state.last_target_marker_t
            msg.detail = f"target={self._state.target_marker_id} obs={self._state.observed_marker_id} mem_age={memory_age:.1f}s"
        
        self.pub_driving_state.publish(msg)


def main():
    rclpy.init()
    node = ControlStackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()