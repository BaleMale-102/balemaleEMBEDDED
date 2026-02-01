#!/usr/bin/env python3
"""
control_stack_node.py (v5 - 횡이동 보정 + 추가 전진)

테스트 완료된 로직:
- 회전(wz) 대신 횡이동(vy)으로 마커 보정
- 마커 사라지면 추가 전진 후 정지
- 90도 회전은 마커 위에서만

확정 파라미터:
- vx: 0.003
- vy_gain: -0.03
- reach_distance: 0.30
- extra_forward_sec: 1.0
"""

import math
import time
import threading
from typing import Optional
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Bool, Float32, Int32

from rc_interfaces.msg import (
    DriveCmd, DrivingState,
    LaneStatus, MarkerStatus, ParkingLineStatus
)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class ControlMode(Enum):
    IDLE = "IDLE"
    DRIVE = "DRIVE"              # 마커 기반 주행
    TURNING = "TURNING"          # 제자리 90도 회전
    PARK = "PARK"


class ParkPhase(Enum):
    IDLE = "IDLE"
    APPROACH = "APPROACH"          # 주차칸 옆으로 전진 (슬롯 마커 찾을 때까지)
    ALIGN_WAIT = "ALIGN_WAIT"      # 정지하고 카메라 대기
    ALIGN_CHECK = "ALIGN_CHECK"    # 오차 확인
    ALIGN_MOVE = "ALIGN_MOVE"      # 펄스 이동
    READY = "READY"                # 로봇팔에게 신호
    WAIT_DROP = "WAIT_DROP"        # 로봇팔 완료 대기
    DONE = "DONE"


@dataclass
class ControllerState:
    mode: ControlMode = ControlMode.IDLE
    park_phase: ParkPhase = ParkPhase.IDLE
    
    # 목표 마커
    target_marker_id: int = -1
    
    # 관측 마커
    observed_marker_id: int = -1
    marker_quality: float = 0.0
    marker_rel_x: float = 0.0
    marker_rel_z: float = 0.0
    marker_rel_yaw: float = 0.0
    
    # 마커 기억 (추가 전진용)
    last_marker_z: float = 999.0
    extra_forward_start: float = 0.0  # 추가 전진 시작 시간
    
    # Lane
    lane_offset: float = 0.0
    lane_angle: float = 0.0
    lane_quality: float = 0.0
    
    # 턴
    turn_target_rad: float = 0.0
    turn_start_yaw: float = 0.0
    
    # IMU yaw
    imu_yaw: float = 0.0
    
    # 타임스탬프
    last_lane_t: float = 0.0
    last_marker_t: float = 0.0
    last_target_marker_t: float = 0.0
    
    # ===== PARK 관련 =====
    target_slot_id: int = -1
    
    # 슬롯 마커 (side_cam)
    slot_marker_valid: bool = False
    slot_marker_id: int = -1
    slot_rel_x: float = 0.0
    slot_rel_z: float = 0.0
    slot_rel_yaw: float = 0.0
    last_slot_t: float = 0.0
    
    # 주차 라인 (side_cam)
    parking_line_valid: bool = False
    parking_line_angle: float = 0.0
    parking_line_offset: float = 0.0
    last_parking_line_t: float = 0.0
    
    # 펄스 제어
    pulse_start_t: float = 0.0
    pulse_cmd: str = ""  # "vx", "vy", "wz"


class ControlStackNode(Node):
    def __init__(self):
        super().__init__('control_stack_node')
        
        # ===== Parameters =====
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('target_marker_topic', '/mission/target_marker')
        self.declare_parameter('turn_target_topic', '/mission/turn_target_rad')
        self.declare_parameter('enable_drive_topic', '/control/enable_drive')
        self.declare_parameter('lane_topic', '/perception/lane')
        self.declare_parameter('marker_status_topic', '/perception/marker_status')
        self.declare_parameter('parking_line_topic', '/perception/parking_line')
        self.declare_parameter('slot_marker_topic', '/perception/slot_marker')
        self.declare_parameter('imu_topic', '/imu/data')
        
        self.declare_parameter('drive_cmd_topic', '/control/drive_cmd')
        self.declare_parameter('driving_state_topic', '/driving/state')
        self.declare_parameter('marker_reached_topic', '/mission/marker_reached')
        self.declare_parameter('turn_done_topic', '/mission/turn_done')
        
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('state_publish_rate_hz', 10.0)
        
        # Timeout
        self.declare_parameter('marker_timeout_sec', 0.5)
        self.declare_parameter('lane_timeout_sec', 1.0)
        
        # ===== 주행 속도 (테스트 확정) =====
        self.declare_parameter('drive_vx', 0.003)
        self.declare_parameter('drive_vy_gain', -0.03)
        self.declare_parameter('drive_max_vy', 0.005)
        self.declare_parameter('drive_wz_gain', -0.3)   # 방향 보정 (rel_x → wz)
        self.declare_parameter('drive_max_wz', 0.1)     # 최대 회전 속도
        self.declare_parameter('reach_distance', 0.30)
        self.declare_parameter('extra_forward_sec', 1.0)
        self.declare_parameter('blind_vx', 0.0015)
        self.declare_parameter('camera_to_center', 0.11)  # 카메라→차량중심 거리(m)
        
        # Turning (제자리 회전) - 고정 시간 기반
        self.declare_parameter('turn_wz', 0.15)           # 회전 속도
        self.declare_parameter('turn_wz_invert', False)   # wz 방향 반전
        self.declare_parameter('turn_time_90', 1.5)       # 90도 회전에 필요한 시간(초) - 테스트로 조정
        self.declare_parameter('turn_stop_ratio', 0.75)   # 오버슈팅 보정 (0.75 = 75%에서 멈춤)
        
        # Park
        self.declare_parameter('park_vy', 0.003)
        self.declare_parameter('park_vx', 0.002)
        
        # Park 펄스 제어
        self.declare_parameter('target_slot_topic', '/mission/target_slot')
        self.declare_parameter('park_ready_topic', '/parking/ready_to_drop')
        self.declare_parameter('drop_complete_topic', '/parking/drop_complete')
        
        self.declare_parameter('park_pulse_move_sec', 0.2)   # 펄스 이동 시간
        self.declare_parameter('park_pulse_wait_sec', 0.5)   # 대기 시간
        self.declare_parameter('park_angle_tol', 0.1)        # 각도 허용 오차 (rad, ~5.7도)
        self.declare_parameter('park_x_tol', 0.02)           # 좌우 허용 오차 (m)
        self.declare_parameter('park_z_target', 0.15)        # 목표 거리 (m)
        self.declare_parameter('park_z_tol', 0.02)           # 거리 허용 오차 (m)
        self.declare_parameter('park_wz', 0.1)               # 각도 보정 회전 속도
        self.declare_parameter('slot_timeout_sec', 0.5)
        
        # Safety
        self.declare_parameter('max_vx', 0.01)
        self.declare_parameter('max_vy', 0.01)
        self.declare_parameter('max_wz', 0.3)
        
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('log_throttle_sec', 0.5)
        
        self._load_params()
        
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
        
        # Publishers
        self.pub_drive = self.create_publisher(DriveCmd, self.get_parameter('drive_cmd_topic').value, 10)
        self.pub_driving_state = self.create_publisher(DrivingState, self.get_parameter('driving_state_topic').value, 10)
        self.pub_marker_reached = self.create_publisher(Int32, self.get_parameter('marker_reached_topic').value, 10)
        self.pub_turn_done = self.create_publisher(Bool, self.get_parameter('turn_done_topic').value, 10)
        self.pub_park_ready = self.create_publisher(Bool, self.get_parameter('park_ready_topic').value, 10)
        
        # Subscribers
        self.sub_mission = self.create_subscription(String, self.get_parameter('mission_state_topic').value, self.cb_mission_state, 10)
        self.sub_target = self.create_subscription(Int32, self.get_parameter('target_marker_topic').value, self.cb_target_marker, 10)
        self.sub_turn = self.create_subscription(Float32, self.get_parameter('turn_target_topic').value, self.cb_turn_target, 10)
        self.sub_enable = self.create_subscription(Bool, self.get_parameter('enable_drive_topic').value, self.cb_enable, 10)
        self.sub_lane = self.create_subscription(LaneStatus, self.get_parameter('lane_topic').value, self.cb_lane, qos_profile_sensor_data)
        self.sub_marker = self.create_subscription(MarkerStatus, self.get_parameter('marker_status_topic').value, self.cb_marker, 10)
        self.sub_parking_line = self.create_subscription(ParkingLineStatus, self.get_parameter('parking_line_topic').value, self.cb_parking_line, qos_profile_sensor_data)
        self.sub_slot_marker = self.create_subscription(MarkerStatus, self.get_parameter('slot_marker_topic').value, self.cb_slot_marker, 10)
        self.sub_target_slot = self.create_subscription(Int32, self.get_parameter('target_slot_topic').value, self.cb_target_slot, 10)
        self.sub_drop_complete = self.create_subscription(Bool, self.get_parameter('drop_complete_topic').value, self.cb_drop_complete, 10)
        
        # IMU subscriber
        from sensor_msgs.msg import Imu
        self.sub_imu = self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.cb_imu, qos_profile_sensor_data)
        
        # Timers
        ctrl_period = 1.0 / max(1.0, self.get_parameter('control_rate_hz').value)
        state_period = 1.0 / max(1.0, self.get_parameter('state_publish_rate_hz').value)
        
        self.timer_control = self.create_timer(ctrl_period, self.control_loop)
        self.timer_state = self.create_timer(state_period, self.publish_driving_state)
        
        self.get_logger().info(f"control_stack_node v5 (lateral correction) started")
        self.get_logger().info(f"  vx={self.drive_vx}, vy_gain={self.drive_vy_gain}, extra_sec={self.extra_sec}")
    
    def _load_params(self):
        self.drive_vx = self.get_parameter('drive_vx').value
        self.drive_vy_gain = self.get_parameter('drive_vy_gain').value
        self.drive_max_vy = self.get_parameter('drive_max_vy').value
        self.drive_wz_gain = self.get_parameter('drive_wz_gain').value
        self.drive_max_wz = self.get_parameter('drive_max_wz').value
        self.reach_dist = self.get_parameter('reach_distance').value
        self.extra_sec = self.get_parameter('extra_forward_sec').value
        self.blind_vx = self.get_parameter('blind_vx').value
        self.cam_offset = self.get_parameter('camera_to_center').value
        
        self.turn_wz = self.get_parameter('turn_wz').value
        self.turn_wz_invert = self.get_parameter('turn_wz_invert').value
        self.turn_time_90 = self.get_parameter('turn_time_90').value
        self.turn_stop_ratio = self.get_parameter('turn_stop_ratio').value
        
        self.park_vy = self.get_parameter('park_vy').value
        self.park_vx = self.get_parameter('park_vx').value
        
        self.park_pulse_move = self.get_parameter('park_pulse_move_sec').value
        self.park_pulse_wait = self.get_parameter('park_pulse_wait_sec').value
        self.park_angle_tol = self.get_parameter('park_angle_tol').value
        self.park_x_tol = self.get_parameter('park_x_tol').value
        self.park_z_target = self.get_parameter('park_z_target').value
        self.park_z_tol = self.get_parameter('park_z_tol').value
        self.park_wz = self.get_parameter('park_wz').value
        self.slot_timeout = self.get_parameter('slot_timeout_sec').value
        
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        
        self.marker_timeout = self.get_parameter('marker_timeout_sec').value
        self.lane_timeout = self.get_parameter('lane_timeout_sec').value
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
            prev = self._mission_state
            self._mission_state = state
            
            if state in ["IDLE", "STOP"]:
                self._state.mode = ControlMode.IDLE
            elif state in ["DRIVE", "MARKER_FOLLOW", "LANE_FOLLOW"]:
                self._state.mode = ControlMode.DRIVE
                # 상태 초기화
                self._state.last_marker_z = 999.0
                self._state.extra_forward_start = 0.0
            elif state == "TURNING":
                self._state.mode = ControlMode.TURNING
                self._state.turn_start_yaw = self._state.imu_yaw
                self._state.turn_start_time = 0.0
                self._state.turn_completed = False
            elif state == "PARK":
                self._state.mode = ControlMode.PARK
                if self._state.park_phase == ParkPhase.IDLE or self._state.park_phase == ParkPhase.DONE:
                    self._state.park_phase = ParkPhase.APPROACH
                    self.get_logger().info("Park started: APPROACH")
            else:
                self._state.mode = ControlMode.IDLE
            
            if state != prev:
                self.get_logger().info(f"Mode: {prev} → {state}")
    
    def cb_target_marker(self, msg: Int32):
        with self._lock:
            prev = self._state.target_marker_id
            self._state.target_marker_id = msg.data
            if msg.data != prev:
                self.get_logger().info(f"Target marker: {prev} → {msg.data}")
                # 새 목표 → 상태 초기화
                self._state.last_marker_z = 999.0
                self._state.extra_forward_start = 0.0
    
    def cb_turn_target(self, msg: Float32):
        with self._lock:
            self._state.turn_target_rad = float(msg.data)
            self._state.turn_start_yaw = self._state.imu_yaw
    
    def cb_enable(self, msg: Bool):
        with self._lock:
            prev = self._enable_drive
            self._enable_drive = msg.data
            if msg.data and not prev:
                # 활성화될 때 상태 초기화
                self._state.last_marker_z = 999.0
                self._state.extra_forward_start = 0.0
    
    def cb_lane(self, msg: LaneStatus):
        with self._lock:
            self._lane = msg
            self._state.last_lane_t = time.time()
            if msg.in_lane:
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
                
                # 목표 마커면 시간 & 거리 기록
                if msg.id == self._state.target_marker_id:
                    self._state.last_target_marker_t = time.time()
                    self._state.last_marker_z = msg.rel_z
    
    def cb_parking_line(self, msg: ParkingLineStatus):
        with self._lock:
            self._parking_line = msg
            self._state.last_parking_line_t = time.time()
            self._state.parking_line_valid = msg.valid
            if msg.valid:
                self._state.parking_line_angle = msg.angle
                self._state.parking_line_offset = msg.offset_norm
    
    def cb_slot_marker(self, msg: MarkerStatus):
        with self._lock:
            self._state.last_slot_t = time.time()
            self._state.slot_marker_valid = msg.valid
            if msg.valid:
                self._state.slot_marker_id = msg.id
                self._state.slot_rel_x = msg.rel_x
                self._state.slot_rel_z = msg.rel_z
                self._state.slot_rel_yaw = msg.rel_yaw
    
    def cb_target_slot(self, msg: Int32):
        with self._lock:
            prev = self._state.target_slot_id
            self._state.target_slot_id = msg.data
            if prev != msg.data:
                self.get_logger().info(f"Target slot: {prev} → {msg.data}")
    
    def cb_drop_complete(self, msg: Bool):
        """로봇팔에서 드롭 완료 신호"""
        with self._lock:
            if msg.data and self._state.park_phase == ParkPhase.WAIT_DROP:
                self._state.park_phase = ParkPhase.DONE
                self.get_logger().info("Drop complete! PARK DONE")
    
    def cb_imu(self, msg):
        with self._lock:
            q = msg.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._state.imu_yaw = math.atan2(siny, cosy)
    
    # ===== Control Loop =====
    
    def control_loop(self):
        with self._lock:
            if not self._enable_drive:
                self._publish_stop("disabled")
                return
            
            mode = self._state.mode
            
            if mode == ControlMode.IDLE:
                self._publish_stop("idle")
            elif mode == ControlMode.DRIVE:
                self._control_drive()
            elif mode == ControlMode.TURNING:
                self._control_turning()
            elif mode == ControlMode.PARK:
                self._control_park()
            else:
                self._publish_stop("unknown")
    
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
    
    # ===== DRIVE 모드 (마커 기반 횡이동 보정) =====
    
    def _control_drive(self):
        """
        테스트 확정 로직:
        1. 목표 마커 보임 → 전진 + 횡이동(vy) 보정
        2. 마커 사라짐 & 가까웠음 → 추가 전진 후 정지
        3. 마커 멀리서 사라짐 → 느리게 직진
        """
        now = time.time()
        target_id = self._state.target_marker_id
        
        # 목표 마커 유효성
        target_visible = (
            self._marker is not None and
            self._marker.valid and
            self._marker.id == target_id and
            (now - self._state.last_target_marker_t) < self.marker_timeout
        )
        
        # ===== 1. 목표 마커 보임 =====
        if target_visible:
            rel_x = self._state.marker_rel_x
            rel_z = self._state.marker_rel_z
            
            # nan 체크
            if math.isnan(rel_x) or math.isnan(rel_z):
                self._log_throttle(f"Marker {target_id}: invalid pose (nan)")
                self._publish_cmd(self.blind_vx, 0.0, 0.0, "blind_nan")
                return
            
            # 추가 전진 타이머 리셋
            self._state.extra_forward_start = 0.0
            self._state.last_marker_z = rel_z
            
            # 횡이동 보정 (rel_x 기반)
            vy = self.drive_vy_gain * rel_x
            vy = clamp(vy, -self.drive_max_vy, self.drive_max_vy)
            
            # 방향 보정 (rel_x 기반 - 마커 쪽으로 회전)
            wz = self.drive_wz_gain * rel_x
            wz = clamp(wz, -self.drive_max_wz, self.drive_max_wz)
            
            self._log_throttle(f"Marker {target_id}: z={rel_z:.2f} x={rel_x:.3f} -> vy={vy:.4f} wz={wz:.3f}")
            self._publish_cmd(self.drive_vx, vy, wz, f"marker_{target_id}")
            return
        
        # ===== 2. 마커 사라짐 - 거리 기반 추가 전진 =====
        if self._state.last_marker_z < self.reach_dist + 0.1:  # 약간 여유
            if self._state.extra_forward_start == 0.0:
                # 차량 중심 기준 남은 거리 계산
                dist_to_go = self._state.last_marker_z + self.cam_offset
                # 전진 시간 계산 (reach_dist:extra_sec 비율 사용)
                self._state.extra_forward_time = dist_to_go / self.reach_dist * self.extra_sec
                self._state.extra_forward_start = now
                self.get_logger().info(f"Marker lost at z={self._state.last_marker_z:.2f}m + cam={self.cam_offset:.2f}m, forward {self._state.extra_forward_time:.2f}s")
            
            elapsed = now - self._state.extra_forward_start
            forward_time = getattr(self._state, 'extra_forward_time', self.extra_sec)
            
            if elapsed < forward_time:
                # 추가 전진 중
                self._log_throttle(f"Extra forward: {elapsed:.1f}/{forward_time:.1f}s")
                self._publish_cmd(self.drive_vx, 0.0, 0.0, "extra_forward")
                return
            else:
                # 추가 전진 완료 → 도달!
                self.get_logger().info(f"REACHED marker {target_id} after extra forward!")
                self._publish_stop("reached")
                self._publish_marker_reached(target_id)
                
                # 상태 리셋
                self._state.last_marker_z = 999.0
                self._state.extra_forward_start = 0.0
                self._state.extra_forward_time = 0.0
                return
        
        # ===== 3. 마커 멀리서 사라짐 → 느리게 직진 =====
        self._log_throttle(f"No marker - slow forward (last_z={self._state.last_marker_z:.2f})")
        self._publish_cmd(self.blind_vx, 0.0, 0.0, "blind")
    
    def _publish_marker_reached(self, marker_id: int):
        msg = Int32()
        msg.data = marker_id
        self.pub_marker_reached.publish(msg)
    
    # ===== TURNING 모드 (IMU yaw 기반) =====
    
    def _control_turning(self):
        """
        IMU yaw 기반 회전:
        - 시작 yaw 저장
        - 목표 각도만큼 회전
        - yaw 변화량이 목표에 도달하면 정지
        """
        now = time.time()
        target_rad = self._state.turn_target_rad
        
        # 이미 완료됨
        if getattr(self._state, 'turn_completed', False):
            self._publish_stop("turn_already_done")
            return
        
        # 턴 시작 초기화
        turn_start = getattr(self._state, 'turn_start_time', 0.0)
        if turn_start == 0.0:
            # 파라미터 다시 읽기
            self.turn_wz = self.get_parameter('turn_wz').value
            self.turn_wz_invert = self.get_parameter('turn_wz_invert').value
            self.turn_stop_ratio = self.get_parameter('turn_stop_ratio').value
            
            self._state.turn_start_time = now
            self._state.turn_start_yaw = self._state.imu_yaw
            self._state.turn_completed = False
            self.get_logger().info(f"Turn start: target={math.degrees(target_rad):.1f}deg, start_yaw={math.degrees(self._state.turn_start_yaw):.1f}deg, stop_ratio={self.turn_stop_ratio}")
        
        # 현재 yaw 변화량 계산
        yaw_diff = self._state.imu_yaw - self._state.turn_start_yaw
        
        # -π ~ π 범위로 정규화
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # 오버슈팅 보정: 목표의 X%에서 멈추기 (관성 고려)
        stop_threshold = abs(target_rad) * self.turn_stop_ratio
        
        # 목표 도달 확인
        if abs(yaw_diff) >= stop_threshold:
            self._publish_stop("turn_done")
            self._publish_turn_done()
            elapsed = now - self._state.turn_start_time
            self.get_logger().info(f"Turn complete! yaw_diff={math.degrees(yaw_diff):.1f}deg ({elapsed:.2f}s)")
            self._state.turn_start_time = 0.0
            self._state.turn_completed = True
            # IDLE 전환 안 함 - route_follower가 다음 모드 설정
            return
        
        # 타임아웃 (10초)
        elapsed = now - self._state.turn_start_time
        if elapsed > 10.0:
            self._publish_stop("turn_timeout")
            self.get_logger().warn(f"Turn timeout! yaw_diff={math.degrees(yaw_diff):.1f}deg")
            self._state.turn_start_time = 0.0
            self._state.turn_completed = True
            # IDLE 전환 안 함 - route_follower가 처리
            return
        
        # 회전
        wz = self.turn_wz if target_rad > 0 else -self.turn_wz
        if self.turn_wz_invert:
            wz = -wz
        
        self._log_throttle(f"Turning: yaw_diff={math.degrees(yaw_diff):.1f}/{math.degrees(target_rad):.1f}deg wz={wz:.2f}")
        self._publish_cmd(0.0, 0.0, wz, "turn")
    
    def _publish_turn_done(self):
        msg = Bool()
        msg.data = True
        self.pub_turn_done.publish(msg)
    
    # ===== PARK 모드 (펄스 제어) =====
    
    def _control_park(self):
        """
        주차 시퀀스 (펄스 제어):
        1. APPROACH   - 전진하며 목표 슬롯 마커 찾기
        2. ALIGN_WAIT - 정지하고 카메라 대기 (0.5초)
        3. ALIGN_CHECK- 오차 확인 → 다음 동작 결정
        4. ALIGN_MOVE - 펄스 이동 (0.2초)
        5. READY      - 로봇팔에게 준비됨 신호
        6. WAIT_DROP  - 로봇팔 완료 대기
        7. DONE
        """
        phase = self._state.park_phase
        
        if phase == ParkPhase.IDLE:
            self._publish_stop("park_idle")
        elif phase == ParkPhase.APPROACH:
            self._park_approach()
        elif phase == ParkPhase.ALIGN_WAIT:
            self._park_align_wait()
        elif phase == ParkPhase.ALIGN_CHECK:
            self._park_align_check()
        elif phase == ParkPhase.ALIGN_MOVE:
            self._park_align_move()
        elif phase == ParkPhase.READY:
            self._park_ready()
        elif phase == ParkPhase.WAIT_DROP:
            self._publish_stop("wait_drop")
        elif phase == ParkPhase.DONE:
            self._publish_stop("park_done")
    
    def _park_approach(self):
        """목표 슬롯 마커 찾을 때까지 전진"""
        now = time.time()
        target_id = self._state.target_slot_id
        
        # 슬롯 마커 유효성
        slot_valid = (
            self._state.slot_marker_valid and
            (now - self._state.last_slot_t) < self.slot_timeout
        )
        
        # 목표 슬롯 찾음?
        if slot_valid and self._state.slot_marker_id == target_id:
            self.get_logger().info(f"Found slot {target_id}, starting align")
            self._state.park_phase = ParkPhase.ALIGN_WAIT
            self._state.pulse_start_t = now
            self._publish_stop("found_slot")
            return
        
        # 아직 못 찾음 → 느리게 전진
        self._log_throttle(f"Approach: looking for slot {target_id}")
        self._publish_cmd(self.park_vx, 0.0, 0.0, "approach")
    
    def _park_align_wait(self):
        """정지하고 카메라 정보 대기"""
        now = time.time()
        elapsed = now - self._state.pulse_start_t
        
        self._publish_stop("align_wait")
        
        if elapsed >= self.park_pulse_wait:
            self._state.park_phase = ParkPhase.ALIGN_CHECK
            self.get_logger().debug("ALIGN_WAIT → ALIGN_CHECK")
    
    def _park_align_check(self):
        """오차 확인 → 다음 동작 결정"""
        now = time.time()
        target_id = self._state.target_slot_id
        
        # 센서 유효성
        slot_valid = (
            self._state.slot_marker_valid and
            self._state.slot_marker_id == target_id and
            (now - self._state.last_slot_t) < self.slot_timeout
        )
        
        line_valid = (
            self._state.parking_line_valid and
            (now - self._state.last_parking_line_t) < self.slot_timeout
        )
        
        if not slot_valid:
            self._log_throttle(f"Align: slot {target_id} not visible")
            # 슬롯 안 보이면 다시 approach
            self._state.park_phase = ParkPhase.APPROACH
            return
        
        # 오차 계산
        angle_err = self._state.parking_line_angle if line_valid else 0.0
        x_err = self._state.slot_rel_x
        z_err = self._state.slot_rel_z - self.park_z_target
        
        self._log_throttle(f"Align check: angle={math.degrees(angle_err):.1f}° x={x_err:.3f}m z_err={z_err:.3f}m")
        
        # 우선순위: angle → x → z
        # 1. 각도 보정 (wz)
        if line_valid and abs(angle_err) > self.park_angle_tol:
            self._state.pulse_cmd = "wz"
            self._state.pulse_start_t = now
            self._state.park_phase = ParkPhase.ALIGN_MOVE
            direction = "CCW" if angle_err > 0 else "CW"
            self.get_logger().info(f"Pulse: wz ({direction}) angle_err={math.degrees(angle_err):.1f}°")
            return
        
        # 2. 좌우 보정 (vy)
        if abs(x_err) > self.park_x_tol:
            self._state.pulse_cmd = "vy"
            self._state.pulse_start_t = now
            self._state.park_phase = ParkPhase.ALIGN_MOVE
            direction = "left" if x_err > 0 else "right"
            self.get_logger().info(f"Pulse: vy ({direction}) x_err={x_err:.3f}m")
            return
        
        # 3. 전후 보정 (vx)
        if abs(z_err) > self.park_z_tol:
            self._state.pulse_cmd = "vx"
            self._state.pulse_start_t = now
            self._state.park_phase = ParkPhase.ALIGN_MOVE
            direction = "forward" if z_err > 0 else "backward"
            self.get_logger().info(f"Pulse: vx ({direction}) z_err={z_err:.3f}m")
            return
        
        # 모두 OK → READY
        self.get_logger().info("Align complete! → READY")
        self._state.park_phase = ParkPhase.READY
    
    def _park_align_move(self):
        """펄스 이동 (짧은 시간)"""
        now = time.time()
        elapsed = now - self._state.pulse_start_t
        
        if elapsed >= self.park_pulse_move:
            # 이동 완료 → 대기로
            self._state.pulse_start_t = now
            self._state.park_phase = ParkPhase.ALIGN_WAIT
            self._publish_stop("pulse_done")
            return
        
        # 펄스 이동
        cmd = self._state.pulse_cmd
        
        if cmd == "wz":
            angle_err = self._state.parking_line_angle
            wz = self.park_wz if angle_err > 0 else -self.park_wz
            self._publish_cmd(0.0, 0.0, wz, "pulse_wz")
        
        elif cmd == "vy":
            x_err = self._state.slot_rel_x
            vy = -self.park_vy if x_err > 0 else self.park_vy
            self._publish_cmd(0.0, vy, 0.0, "pulse_vy")
        
        elif cmd == "vx":
            z_err = self._state.slot_rel_z - self.park_z_target
            vx = self.park_vx if z_err > 0 else -self.park_vx
            self._publish_cmd(vx, 0.0, 0.0, "pulse_vx")
    
    def _park_ready(self):
        """로봇팔에게 준비됨 신호"""
        self._publish_stop("park_ready")
        
        # Ready 신호 발행
        msg = Bool()
        msg.data = True
        self.pub_park_ready.publish(msg)
        
        self.get_logger().info("Published ready_to_drop signal")
        self._state.park_phase = ParkPhase.WAIT_DROP
    
    # ===== Driving State =====
    
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
            msg.ex = float(self._state.marker_rel_x)
            msg.ey = float(self._state.marker_rel_z)
            msg.eyaw = float(self._state.marker_rel_yaw)
            msg.confidence = float(self._state.marker_quality)
            
            msg.detail = f"last_z={self._state.last_marker_z:.2f} extra={self._state.extra_forward_start > 0}"
        
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