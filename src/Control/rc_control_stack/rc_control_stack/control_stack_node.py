#!/usr/bin/env python3
"""
control_stack_node.py (v2 - with DrivingState)

자율주행 메인 컨트롤러
모드별 제어 + DrivingState 발행

모드:
  - IDLE: 대기 (출력 없음)
  - LANE_FOLLOW: 차선 추종 (Stanley-lite)
  - TURNING: 제자리 회전
  - ADVANCE_TO_CENTER: 마커 중앙으로 전진
  - ALIGN_TO_MARKER: 마커 기반 정밀 정렬
  - PARK: 주차 FSM

토픽:
  Subscribe:
    - /mission/state (String)
    - /mission/turn_target_rad (Float32)
    - /control/enable_drive (Bool)
    - /perception/lane (LaneStatus)
    - /perception/marker_status (MarkerStatus)
    - /perception/parking_line (ParkingLineStatus) [PARK용]
    - /perception/slot_marker_pose (PoseStamped) [PARK용]
  
  Publish:
    - /control/drive_cmd (DriveCmd)
    - /driving/state (DrivingState) ← NEW
    - /mission/align_done (Bool)
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

from std_msgs.msg import String, Bool, Float32
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
    """내부 상태 관리"""
    mode: ControlMode = ControlMode.IDLE
    park_phase: ParkPhase = ParkPhase.IDLE
    
    # 에러 상태
    err_x: float = 0.0
    err_y: float = 0.0
    err_yaw: float = 0.0
    
    # 마커 정보
    target_marker_id: int = -1
    observed_marker_id: int = -1
    marker_quality: float = 0.0
    
    # Lane 정보
    lane_offset: float = 0.0
    lane_angle: float = 0.0
    lane_quality: float = 0.0
    
    # 턴 타겟
    turn_target_rad: float = 0.0
    turn_accumulated_rad: float = 0.0
    
    # 타임스탬프
    last_lane_t: float = 0.0
    last_marker_t: float = 0.0


class ControlStackNode(Node):
    def __init__(self):
        super().__init__('control_stack_node')
        
        # ===== Parameters =====
        # Topics
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('turn_target_topic', '/mission/turn_target_rad')
        self.declare_parameter('enable_drive_topic', '/control/enable_drive')
        self.declare_parameter('lane_topic', '/perception/lane')
        self.declare_parameter('marker_status_topic', '/perception/marker_status')
        self.declare_parameter('parking_line_topic', '/perception/parking_line')
        self.declare_parameter('slot_marker_topic', '/perception/slot_marker_pose')
        
        self.declare_parameter('drive_cmd_topic', '/control/drive_cmd')
        self.declare_parameter('driving_state_topic', '/driving/state')
        self.declare_parameter('align_done_topic', '/mission/align_done')
        
        # Control rates
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('state_publish_rate_hz', 10.0)
        
        # Timeouts
        self.declare_parameter('lane_timeout_sec', 0.5)
        self.declare_parameter('marker_timeout_sec', 1.0)
        
        # Lane follow (Stanley-lite)
        self.declare_parameter('lane_vx', 0.15)
        self.declare_parameter('lane_k_psi', 0.8)      # heading gain
        self.declare_parameter('lane_k_y', 1.2)        # crosstrack gain
        self.declare_parameter('lane_v0', 0.1)         # softening
        
        # Turning
        self.declare_parameter('turn_wz', 0.5)
        self.declare_parameter('turn_tolerance_rad', 0.1)
        
        # Advance to center
        self.declare_parameter('advance_vx', 0.10)
        self.declare_parameter('advance_target_z', 0.15)  # 마커까지 목표 거리
        self.declare_parameter('advance_tolerance_z', 0.03)
        
        # Align to marker
        self.declare_parameter('align_kp_x', 0.8)
        self.declare_parameter('align_kp_y', 0.6)
        self.declare_parameter('align_kp_yaw', 0.5)
        self.declare_parameter('align_tolerance_xy', 0.02)
        self.declare_parameter('align_tolerance_yaw', 0.08)
        self.declare_parameter('align_max_v', 0.12)
        
        # Park - Line align
        self.declare_parameter('park_line_vy', 0.08)
        self.declare_parameter('park_line_kp_offset', 0.5)
        self.declare_parameter('park_line_kp_angle', 0.3)
        self.declare_parameter('park_line_tolerance_offset', 0.05)
        self.declare_parameter('park_line_tolerance_angle', 0.1)
        
        # Park - Slot marker
        self.declare_parameter('park_slot_vx', 0.06)
        self.declare_parameter('park_slot_target_x', 0.10)
        self.declare_parameter('park_slot_tolerance', 0.02)
        
        # Safety
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_wz', 1.0)
        
        # Frame
        self.declare_parameter('base_frame', 'base_link')
        
        # Logging
        self.declare_parameter('log_throttle_sec', 1.0)
        
        # Load params
        self._load_params()
        
        # State
        self._lock = threading.Lock()
        self._state = ControllerState()
        self._enable_drive = False
        self._mission_state = "IDLE"
        self._last_log_t = 0.0
        
        # Perception data
        self._lane: Optional[LaneStatus] = None
        self._marker: Optional[MarkerStatus] = None
        self._parking_line: Optional[ParkingLineStatus] = None
        self._slot_marker: Optional[PoseStamped] = None
        
        # Publishers
        self.pub_drive = self.create_publisher(
            DriveCmd, 
            self.get_parameter('drive_cmd_topic').value, 
            10
        )
        self.pub_driving_state = self.create_publisher(
            DrivingState, 
            self.get_parameter('driving_state_topic').value, 
            10
        )
        self.pub_align_done = self.create_publisher(
            Bool, 
            self.get_parameter('align_done_topic').value, 
            10
        )
        
        # Subscribers
        self.sub_mission = self.create_subscription(
            String,
            self.get_parameter('mission_state_topic').value,
            self.cb_mission_state,
            10
        )
        self.sub_turn = self.create_subscription(
            Float32,
            self.get_parameter('turn_target_topic').value,
            self.cb_turn_target,
            10
        )
        self.sub_enable = self.create_subscription(
            Bool,
            self.get_parameter('enable_drive_topic').value,
            self.cb_enable,
            10
        )
        self.sub_lane = self.create_subscription(
            LaneStatus,
            self.get_parameter('lane_topic').value,
            self.cb_lane,
            qos_profile_sensor_data
        )
        self.sub_marker = self.create_subscription(
            MarkerStatus,
            self.get_parameter('marker_status_topic').value,
            self.cb_marker,
            10
        )
        self.sub_parking_line = self.create_subscription(
            ParkingLineStatus,
            self.get_parameter('parking_line_topic').value,
            self.cb_parking_line,
            qos_profile_sensor_data
        )
        self.sub_slot = self.create_subscription(
            PoseStamped,
            self.get_parameter('slot_marker_topic').value,
            self.cb_slot_marker,
            10
        )
        
        # Timers
        ctrl_period = 1.0 / max(1.0, self.get_parameter('control_rate_hz').value)
        state_period = 1.0 / max(1.0, self.get_parameter('state_publish_rate_hz').value)
        
        self.timer_control = self.create_timer(ctrl_period, self.control_loop)
        self.timer_state = self.create_timer(state_period, self.publish_driving_state)
        
        self.get_logger().info(
            f"control_stack_node started\n"
            f"  drive_cmd: {self.get_parameter('drive_cmd_topic').value}\n"
            f"  driving_state: {self.get_parameter('driving_state_topic').value}\n"
            f"  control_rate: {self.get_parameter('control_rate_hz').value}Hz"
        )
    
    def _load_params(self):
        """파라미터 로드"""
        self.lane_vx = self.get_parameter('lane_vx').value
        self.lane_k_psi = self.get_parameter('lane_k_psi').value
        self.lane_k_y = self.get_parameter('lane_k_y').value
        self.lane_v0 = self.get_parameter('lane_v0').value
        
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
        
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        
        self.lane_timeout = self.get_parameter('lane_timeout_sec').value
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value
        self.log_throttle = self.get_parameter('log_throttle_sec').value
        self.base_frame = self.get_parameter('base_frame').value
    
    def _log_throttle(self, msg: str):
        now = time.time()
        if now - self._last_log_t >= self.log_throttle:
            self._last_log_t = now
            self.get_logger().info(msg)
    
    # ===== Callbacks =====
    
    def cb_mission_state(self, msg: String):
        state = msg.data.upper()
        with self._lock:
            prev_state = self._mission_state
            self._mission_state = state
            
            # 모드 매핑
            if state == "IDLE":
                self._state.mode = ControlMode.IDLE
            elif state == "DRIVE":
                self._state.mode = ControlMode.LANE_FOLLOW
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
                # STOP_AT_MARKER, STOP_BUMP 등 → IDLE 처리
                self._state.mode = ControlMode.IDLE
            
            if state != prev_state:
                self.get_logger().info(f"Mission state: {prev_state} → {state}, mode: {self._state.mode.value}")
    
    def cb_turn_target(self, msg: Float32):
        with self._lock:
            self._state.turn_target_rad = float(msg.data)
            self._state.turn_accumulated_rad = 0.0  # reset
    
    def cb_enable(self, msg: Bool):
        with self._lock:
            self._enable_drive = msg.data
    
    def cb_lane(self, msg: LaneStatus):
        with self._lock:
            self._lane = msg
            self._state.last_lane_t = time.time()
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
                self._state.err_x = msg.rel_x
                self._state.err_y = msg.rel_y
                self._state.err_yaw = msg.rel_yaw
    
    def cb_parking_line(self, msg: ParkingLineStatus):
        with self._lock:
            self._parking_line = msg
    
    def cb_slot_marker(self, msg: PoseStamped):
        with self._lock:
            self._slot_marker = msg
    
    # ===== Control Loop =====
    
    def control_loop(self):
        with self._lock:
            if not self._enable_drive:
                self._publish_stop("disabled")
                return
            
            mode = self._state.mode
            
            if mode == ControlMode.IDLE:
                self._publish_stop("idle")
            elif mode == ControlMode.LANE_FOLLOW:
                self._control_lane_follow()
            elif mode == ControlMode.TURNING:
                self._control_turning()
            elif mode == ControlMode.ADVANCE_TO_CENTER:
                self._control_advance()
            elif mode == ControlMode.ALIGN_TO_MARKER:
                self._control_align()
            elif mode == ControlMode.PARK:
                self._control_park()
            else:
                self._publish_stop("unknown_mode")
    
    def _publish_stop(self, source: str):
        cmd = DriveCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.enable = False
        cmd.vx = 0.0
        cmd.vy = 0.0
        cmd.wz = 0.0
        cmd.source = f"control_stack:{source}"
        self.pub_drive.publish(cmd)
    
    def _publish_cmd(self, vx: float, vy: float, wz: float, source: str):
        cmd = DriveCmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.enable = True
        cmd.vx = float(clamp(vx, -self.max_vx, self.max_vx))
        cmd.vy = float(clamp(vy, -self.max_vy, self.max_vy))
        cmd.wz = float(clamp(wz, -self.max_wz, self.max_wz))
        cmd.source = f"control_stack:{source}"
        self.pub_drive.publish(cmd)
    
    # ===== Mode Controllers =====
    
    def _control_lane_follow(self):
        """Stanley-lite 차선 추종"""
        now = time.time()
        
        # Lane timeout 체크
        if now - self._state.last_lane_t > self.lane_timeout:
            self._log_throttle("Lane timeout, stopping")
            self._publish_stop("lane_timeout")
            return
        
        if self._lane is None or not self._lane.in_lane:
            self._log_throttle("No lane detected")
            self._publish_stop("no_lane")
            return
        
        # Stanley-lite 조향
        offset = self._state.lane_offset  # -1 ~ +1
        angle = self._state.lane_angle    # rad
        
        # vx: 일정 속도
        vx = self.lane_vx
        
        # wz: heading + crosstrack
        psi_term = self.lane_k_psi * angle
        y_term = math.atan2(self.lane_k_y * offset, vx + self.lane_v0)
        wz = psi_term + y_term
        
        self._publish_cmd(vx, 0.0, wz, "lane_follow")
    
    def _control_turning(self):
        """제자리 회전"""
        target = self._state.turn_target_rad
        
        if abs(target) < self.turn_tol:
            # 회전 완료
            self._publish_stop("turn_done")
            return
        
        # 회전 방향
        wz = self.turn_wz if target > 0 else -self.turn_wz
        
        self._publish_cmd(0.0, 0.0, wz, "turning")
    
    def _control_advance(self):
        """마커 중앙으로 전진"""
        now = time.time()
        
        if now - self._state.last_marker_t > self.marker_timeout:
            self._log_throttle("Marker timeout in advance")
            self._publish_stop("marker_timeout")
            return
        
        if self._marker is None or not self._marker.valid:
            self._publish_stop("no_marker")
            return
        
        # rel_z: 마커까지 거리
        dist = self._marker.rel_z
        
        if dist <= self.advance_z + self.advance_tol:
            # 목표 도달
            self._publish_stop("advance_done")
            return
        
        # 전진
        self._publish_cmd(self.advance_vx, 0.0, 0.0, "advance")
    
    def _control_align(self):
        """마커 기반 정밀 정렬"""
        now = time.time()
        
        if now - self._state.last_marker_t > self.marker_timeout:
            self._log_throttle("Marker timeout in align")
            self._publish_stop("marker_timeout")
            return
        
        if self._marker is None or not self._marker.valid:
            self._publish_stop("no_marker")
            return
        
        # 에러
        ex = self._marker.rel_x  # 좌우
        ey = self._marker.rel_z - self.advance_z  # 전후 (목표 거리 기준)
        eyaw = self._marker.rel_yaw
        
        # P 제어
        vx = -self.align_kp_x * ey  # 전후
        vy = -self.align_kp_y * ex  # 좌우
        wz = -self.align_kp_yaw * eyaw
        
        # 속도 제한
        vx = clamp(vx, -self.align_max_v, self.align_max_v)
        vy = clamp(vy, -self.align_max_v, self.align_max_v)
        
        # 수렴 체크
        if (abs(ex) < self.align_tol_xy and 
            abs(ey) < self.align_tol_xy and 
            abs(eyaw) < self.align_tol_yaw):
            # 정렬 완료
            self._publish_stop("align_done")
            self._publish_align_done(True)
            return
        
        # 상태 저장
        self._state.err_x = ex
        self._state.err_y = ey
        self._state.err_yaw = eyaw
        
        self._publish_cmd(vx, vy, wz, "align")
    
    def _control_park(self):
        """주차 FSM"""
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
        """주차선 정렬"""
        if self._parking_line is None or not self._parking_line.valid:
            self._publish_stop("no_parking_line")
            return
        
        offset = self._parking_line.offset_norm
        angle = self._parking_line.angle
        
        kp_off = self.get_parameter('park_line_kp_offset').value
        kp_ang = self.get_parameter('park_line_kp_angle').value
        tol_off = self.get_parameter('park_line_tolerance_offset').value
        tol_ang = self.get_parameter('park_line_tolerance_angle').value
        base_vy = self.get_parameter('park_line_vy').value
        
        # P 제어
        vy = base_vy - kp_off * offset
        wz = -kp_ang * angle
        
        # 수렴 체크
        if abs(offset) < tol_off and abs(angle) < tol_ang:
            self._state.park_phase = ParkPhase.SLOT_MARKER_ALIGN
            self.get_logger().info("Park: LINE_ALIGN → SLOT_MARKER_ALIGN")
            return
        
        self._publish_cmd(0.0, vy, wz, "park_line")
    
    def _park_slot_align(self):
        """슬롯 마커 정렬"""
        if self._slot_marker is None:
            self._publish_stop("no_slot_marker")
            return
        
        # slot_marker는 camera_side 기준
        # x: forward, y: left, z: up
        slot_x = self._slot_marker.pose.position.x
        
        target_x = self.get_parameter('park_slot_target_x').value
        tol = self.get_parameter('park_slot_tolerance').value
        base_vx = self.get_parameter('park_slot_vx').value
        
        err = slot_x - target_x
        
        if abs(err) < tol:
            self._state.park_phase = ParkPhase.DONE
            self.get_logger().info("Park: SLOT_MARKER_ALIGN → DONE")
            return
        
        # 전진/후진
        vx = base_vx if err > 0 else -base_vx
        self._publish_cmd(vx, 0.0, 0.0, "park_slot")
    
    def _publish_align_done(self, done: bool):
        msg = Bool()
        msg.data = done
        self.pub_align_done.publish(msg)
    
    # ===== DrivingState 발행 =====
    
    def publish_driving_state(self):
        """DrivingState 메시지 발행"""
        with self._lock:
            msg = DrivingState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            
            # 모드 문자열
            if self._state.mode == ControlMode.PARK:
                msg.state = f"PARK_{self._state.park_phase.value}"
            else:
                msg.state = self._state.mode.value
            
            # 마커 정보
            msg.target_marker_id = self._state.target_marker_id
            msg.observed_marker_id = self._state.observed_marker_id
            
            # 에러
            msg.ex = float(self._state.err_x)
            msg.ey = float(self._state.err_y)
            msg.eyaw = float(self._state.err_yaw)
            
            # 신뢰도 (마커 quality 기반)
            msg.confidence = float(self._state.marker_quality)
            
            # 상세 정보
            msg.detail = f"lane_q={self._state.lane_quality:.2f} marker_q={self._state.marker_quality:.2f}"
        
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
