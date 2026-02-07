#!/usr/bin/env python3
"""
controller_node.py - Motion Controller Node

Controls robot motion based on perception inputs:
- Lane following mode: Follow detected lanes
- Marker approach mode: Approach target marker
- Turn mode: In-place rotation
- Parking modes: Side camera-based parking maneuvers

Subscribes:
    /perception/lane_status: LaneStatus
    /perception/tracked_marker: TrackedMarker
    /perception/side_markers: MarkerArray (for parking)
    /perception/slot_rect: SlotLineStatus (for parking)
    /mission/state: String (mission state)
    /mission/target_marker: Int32
    /parking/target_slot: Int32

Publishes:
    /control/cmd_vel: Twist (velocity commands)
    /driving/state: DrivingState (diagnostics)
    /parking/align_marker_done: Bool
    /parking/align_rect_done: Bool
    /parking/final_done: Bool
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Imu

from .pid_controller import PIDController


def quaternion_to_yaw(q) -> float:
    """Quaternion에서 yaw 추출"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MotionControllerNode(Node):
    """Motion controller with multiple control modes including parking."""

    # Control modes
    MODE_IDLE = 'IDLE'
    MODE_DRIVE = 'DRIVE'
    MODE_STOP_AT_MARKER = 'STOP_AT_MARKER'
    MODE_ADVANCE_TO_CENTER = 'ADVANCE_TO_CENTER'
    MODE_TURN = 'TURN'
    MODE_ALIGN = 'ALIGN'
    MODE_STOP = 'STOP'
    # Parking modes
    MODE_PARK_DETECT = 'PARK_DETECT'
    MODE_PARK_RECOVERY = 'PARK_RECOVERY'
    MODE_PARK_ALIGN_MARKER = 'PARK_ALIGN_MARKER'
    MODE_PARK_ALIGN_RECT = 'PARK_ALIGN_RECT'    # 안씀
    MODE_PARK_FINAL = 'PARK_FINAL'
    # Approach/retreat modes (hardcoded movement)
    MODE_APPROACH_VEHICLE = 'APPROACH_VEHICLE'
    MODE_RETREAT_FROM_VEHICLE = 'RETREAT_FROM_VEHICLE'
    MODE_RETREAT_FROM_SLOT = 'RETREAT_FROM_SLOT'

    def __init__(self):
        super().__init__('motion_controller_node')

        # Parameters - general
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_vx', 0.008)
        self.declare_parameter('max_vy', 0.008)
        self.declare_parameter('max_wz', 0.05)

        # Parameters - fallback
        self.declare_parameter('reverse_vx', 0.015)

        # Parameters - lane following
        self.declare_parameter('lane_vx', 0.004)
        self.declare_parameter('lane_vy_kp', 0.03)
        self.declare_parameter('lane_vy_ki', 0.0)
        self.declare_parameter('lane_vy_kd', 0.005)
        self.declare_parameter('lane_wz_kp', 0.3)
        self.declare_parameter('lane_wz_ki', 0.0)
        self.declare_parameter('lane_wz_kd', 0.05)

        # Parameters - marker approach
        self.declare_parameter('marker_vx_kp', 0.015)
        self.declare_parameter('marker_vy_kp', 0.005)
        self.declare_parameter('marker_wz_kp', 0.025)
        self.declare_parameter('marker_reach_distance', 0.25)
        self.declare_parameter('marker_min_distance', 0.08)
        self.declare_parameter('marker_lost_reach_distance', 0.35)
        self.declare_parameter('marker_align_threshold', 0.15)
        self.declare_parameter('advance_vx', 0.02)
        self.declare_parameter('advance_time', 0.5)
        self.declare_parameter('align_move_duration', 0.15)
        self.declare_parameter('align_settle_duration', 0.3)

        # Parameters - turn
        self.declare_parameter('turn_wz', 0.04)
        self.declare_parameter('turn_wz_min', 0.015)
        self.declare_parameter('turn_tolerance', 0.10)
        self.declare_parameter('turn_kp', 1.5)
        self.declare_parameter('turn_kd', 0.3)
        self.declare_parameter('turn_decel_zone', 0.5)
        self.declare_parameter('turn_scale', 1.0)

        # Parameters - parking
        self.declare_parameter('park_creep_vx', 0.003)
        self.declare_parameter('park_align_kp', 0.02)
        self.declare_parameter('park_rect_kp', 0.015)
        self.declare_parameter('park_final_kp', 0.02)
        self.declare_parameter('park_max_vx', 0.008)
        self.declare_parameter('park_max_vy', 0.01)
        self.declare_parameter('park_min_speed', 0.003)
        self.declare_parameter('park_marker_threshold', 0.05)  # ~3deg
        self.declare_parameter('park_center_threshold', 0.01)  # 1cm
        self.declare_parameter('park_side_camera_offset_x', 0.0)  # camera offset
        self.declare_parameter('park_rect_threshold', 0.015)   # 1.5cm
        self.declare_parameter('park_distance_threshold', 0.02)  # 2cm
        self.declare_parameter('park_target_distance', 0.15)   # 15cm
        self.declare_parameter('park_align_forward', 0.03)   # 정렬 후 전진 거리 (3cm)

        # Parameters - approach/retreat
        self.declare_parameter('approach_speed', 0.01)
        self.declare_parameter('retreat_from_slot_speed', 0.02)

        # Parameters - stall detection and power boost
        self.declare_parameter('stall_check_interval', 0.5)
        self.declare_parameter('stall_boost_increment', 0.002)
        self.declare_parameter('stall_max_boost', 0.015)
        self.declare_parameter('stall_threshold_wz', 0.005)  # rad
        self.declare_parameter('stall_threshold_vy', 0.01)   # rad (marker angle change)
        self.declare_parameter('stall_pulse_duration', 0.15)

        # Parameters - drive search creep (마커 안 보일 때 전진 탐색)
        self.declare_parameter('drive_search_creep_vx', 0.004)
        self.declare_parameter('drive_search_creep_delay', 1.0)

        # Parameters - post-turn marker search
        self.declare_parameter('post_turn_search_timeout', 3.0)

        # Load parameters
        control_rate = self.get_parameter('control_rate').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value

        self.reverse_vx = self.get_parameter('reverse_vx').value
        self.lane_vx = self.get_parameter('lane_vx').value
        self.marker_reach_dist = self.get_parameter('marker_reach_distance').value
        self.marker_min_dist = self.get_parameter('marker_min_distance').value
        self.marker_lost_reach_dist = self.get_parameter('marker_lost_reach_distance').value
        self.marker_align_threshold = self.get_parameter('marker_align_threshold').value
        self.advance_vx = self.get_parameter('advance_vx').value
        self.advance_time = self.get_parameter('advance_time').value
        self.align_move_duration = self.get_parameter('align_move_duration').value
        self.align_settle_duration = self.get_parameter('align_settle_duration').value
        self.turn_wz = self.get_parameter('turn_wz').value
        self.turn_wz_min = self.get_parameter('turn_wz_min').value
        self.turn_tolerance = self.get_parameter('turn_tolerance').value
        self.turn_kp = self.get_parameter('turn_kp').value
        self.turn_kd = self.get_parameter('turn_kd').value
        self.turn_decel_zone = self.get_parameter('turn_decel_zone').value
        self.turn_scale = self.get_parameter('turn_scale').value

        # Parking parameters
        self.park_creep_vx = self.get_parameter('park_creep_vx').value
        self.park_align_kp = self.get_parameter('park_align_kp').value
        self.park_rect_kp = self.get_parameter('park_rect_kp').value
        self.park_final_kp = self.get_parameter('park_final_kp').value
        self.park_max_vx = self.get_parameter('park_max_vx').value
        self.park_max_vy = self.get_parameter('park_max_vy').value
        self.park_min_speed = self.get_parameter('park_min_speed').value
        self.park_marker_threshold = self.get_parameter('park_marker_threshold').value
        self.park_center_threshold = self.get_parameter('park_center_threshold').value
        self.park_side_camera_offset_x = self.get_parameter('park_side_camera_offset_x').value
        self.park_rect_threshold = self.get_parameter('park_rect_threshold').value
        self.park_distance_threshold = self.get_parameter('park_distance_threshold').value
        self.park_target_distance = self.get_parameter('park_target_distance').value
        self.park_align_forward = self.get_parameter('park_align_forward').value

        # Approach/retreat speed
        self.approach_speed = self.get_parameter('approach_speed').value
        self.retreat_from_slot_speed = self.get_parameter('retreat_from_slot_speed').value

        # Drive search creep (긴 구간에서 마커 안 보일 때 전진 탐색)
        self._drive_search_creep_vx = self.get_parameter('drive_search_creep_vx').value
        self._drive_search_creep_delay = self.get_parameter('drive_search_creep_delay').value
        self._drive_search_creep_pairs = {(4, 10), (9, 15), (10, 4), (15, 9)}
        self._drive_search_start_time = 0.0

        # Stall detection parameters
        self._stall_check_interval = self.get_parameter('stall_check_interval').value
        self._stall_boost_increment = self.get_parameter('stall_boost_increment').value
        self._stall_max_boost = self.get_parameter('stall_max_boost').value
        self._stall_threshold_wz = self.get_parameter('stall_threshold_wz').value
        self._stall_threshold_vy = self.get_parameter('stall_threshold_vy').value
        self._stall_pulse_duration = self.get_parameter('stall_pulse_duration').value
        self._post_turn_search_timeout = self.get_parameter('post_turn_search_timeout').value

        # PID controllers for lane following
        self.lane_vy_pid = PIDController(
            kp=self.get_parameter('lane_vy_kp').value,
            ki=self.get_parameter('lane_vy_ki').value,
            kd=self.get_parameter('lane_vy_kd').value,
            output_limits=(-self.max_vy, self.max_vy)
        )
        self.lane_wz_pid = PIDController(
            kp=self.get_parameter('lane_wz_kp').value,
            ki=self.get_parameter('lane_wz_ki').value,
            kd=self.get_parameter('lane_wz_kd').value,
            output_limits=(-self.max_wz, self.max_wz)
        )

        # Marker approach gains
        self.marker_vx_kp = self.get_parameter('marker_vx_kp').value
        self.marker_vy_kp = self.get_parameter('marker_vy_kp').value
        self.marker_wz_kp = self.get_parameter('marker_wz_kp').value

        # State
        self._mode = self.MODE_IDLE
        self._enable = False
        self._target_marker_id = -1
        self._target_slot_id = -1
        self._turn_target_rad = 0.0
        self._turn_start_yaw = 0.0
        self._current_yaw = 0.0
        self._prev_turn_error = 0.0
        self._turn_done_sent = False
        self._turn_last_target = 0.0

        # Recovery state
        self._recovery_direction = ''
        self._recovery_start_time = 0.0

        # Perception data
        self._tracked_marker = None
        self._side_marker = None
        self._slot_rect = None
        self._imu_data = None

        # Marker tracking state
        self._last_marker_distance = float('inf')
        self._last_marker_id = -1
        self._drive_from_marker_id = -1  # DRIVE 진입 시 출발 마커 (creep pair용)

        # ADVANCE_TO_CENTER state
        self._advance_target_distance = 0.0
        self._advance_start_time = 0.0
        self._advance_done_sent = False

        # Parking done flags
        self._park_align_marker_done_sent = False
        self._park_align_rect_done_sent = False
        self._park_final_done_sent = False

        # ALIGN_TO_MARKER two-phase state (wz → vy)
        self._align_phase = 'WZ'  # 'WZ' first, then 'VY'
        self._align_wz_done = False
        # Move-pause-observe pattern for alignment
        self._align_action = 'MOVE'  # 'MOVE' or 'SETTLE'
        self._align_action_start_time = 0.0

        # Turn marker tracking (for post-turn search)
        self._turn_direction = 0  # +1 = CCW, -1 = CW
        self._marker_seen_during_turn = False
        self._marker_visible_at_turn_end = False
        self._post_turn_search_mode = False
        self._post_turn_search_direction = 0
        self._post_turn_search_start_time = 0.0
        self._post_turn_search_timeout = 3.0  # seconds

        # Stall detection and power boost
        self._stall_detection_enabled = True
        self._stall_check_interval = 0.5  # Check every 0.5s
        self._last_stall_check_time = 0.0
        self._prev_yaw_for_stall = 0.0
        self._prev_marker_angle_for_stall = 0.0
        self._stall_vy_boost = 0.0
        self._stall_wz_boost = 0.0
        self._stall_boost_increment = 0.002  # Small increment per detection
        self._stall_max_boost = 0.015  # Maximum boost
        self._stall_threshold_wz = 0.005  # rad - minimum rotation to not be considered stalled
        self._stall_threshold_vy = 0.01  # marker angle change threshold
        self._consecutive_stalls = 0
        self._stall_pulse_mode = False
        self._stall_pulse_start_time = 0.0
        self._stall_pulse_duration = 0.15  # 150ms pulse

        # Parking stall detection and power boost
        self._park_stall_boost = 0.0
        self._park_consecutive_stalls = 0
        self._park_stall_pulse_mode = False
        self._park_stall_pulse_start_time = 0.0
        self._park_last_stall_check_time = 0.0
        self._park_prev_stall_value = 0.0

        # Log output interval
        self._last_log_time = 0.0
        self._log_interval = 1.0

        # Import custom interfaces
        try:
            from robot_interfaces.msg import (
                TrackedMarker, DrivingState,
                MarkerArray, SlotLineStatus
            )
            self._has_interface = True
            self._TrackedMarker = TrackedMarker
            self._DrivingState = DrivingState
            self._MarkerArray = MarkerArray
            self._SlotLineStatus = SlotLineStatus
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 10)

        if self._has_interface:
            self.pub_driving_state = self.create_publisher(
                self._DrivingState, '/driving/state', 10
            )

        # Mission event publishers
        self.pub_marker_reached = self.create_publisher(
            Int32, '/mission/marker_reached', 10
        )
        self.pub_turn_done = self.create_publisher(
            Bool, '/mission/turn_done', 10
        )

        # Parking event publishers
        self.pub_park_align_marker_done = self.create_publisher(
            Bool, '/parking/align_marker_done', 10
        )
        self.pub_park_align_rect_done = self.create_publisher(
            Bool, '/parking/align_rect_done', 10
        )
        self.pub_park_final_done = self.create_publisher(
            Bool, '/parking/final_done', 10
        )

        # Subscribers
        self.sub_mission_state = self.create_subscription(
            String, '/mission/state', self._mission_state_callback, 10
        )
        self.sub_target_marker = self.create_subscription(
            Int32, '/mission/target_marker', self._target_marker_callback, 10
        )
        self.sub_target_slot = self.create_subscription(
            Int32, '/parking/target_slot', self._target_slot_callback, 10
        )
        self.sub_enable = self.create_subscription(
            Bool, '/control/enable_drive', self._enable_callback, 10
        )
        self.sub_turn_target = self.create_subscription(
            Int32, '/mission/turn_target_rad', self._turn_target_callback, 10
        )

        if self._has_interface:
            self.sub_marker = self.create_subscription(
                self._TrackedMarker, '/perception/tracked_marker',
                self._marker_callback, qos_profile_sensor_data
            )
            # Side markers for parking
            self.sub_side_markers = self.create_subscription(
                self._MarkerArray, '/perception/side_markers',
                self._side_markers_callback, qos_profile_sensor_data
            )
            # Slot rectangle for parking
            self.sub_slot_rect = self.create_subscription(
                self._SlotLineStatus, '/perception/slot_rect',
                self._slot_rect_callback, qos_profile_sensor_data
            )

        # IMU subscription
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data',
            self._imu_callback, qos_profile_sensor_data
        )

        # Control timer
        self.timer = self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info('MotionControllerNode started (with parking)')

    def _mission_state_callback(self, msg: String):
        """Handle mission state changes."""
        state = msg.data.upper()
        prev_mode = self._mode

        if state == 'IDLE':
            self._mode = self.MODE_IDLE
        elif state in ('DRIVE', 'LANE_FOLLOW', 'MARKER_APPROACH', 'APPROACH'):
            if prev_mode != self.MODE_DRIVE:
                self._drive_from_marker_id = self._last_marker_id
                self._drive_search_start_time = 0.0
                self.get_logger().info(
                    f'DRIVE started: from_marker={self._drive_from_marker_id}, target={self._target_marker_id}'
                )
            self._mode = self.MODE_DRIVE
        elif state == 'STOP_AT_MARKER':
            self._mode = self.MODE_STOP_AT_MARKER
            if prev_mode != self.MODE_STOP_AT_MARKER:
                if self._tracked_marker is not None and self._tracked_marker.is_detected:
                    self._advance_target_distance = self._tracked_marker.distance
                else:
                    self._advance_target_distance = self._last_marker_distance
                self.get_logger().info(f'STOP_AT_MARKER: distance={self._advance_target_distance:.3f}m')
        elif state == 'ADVANCE_TO_CENTER':
            self._mode = self.MODE_ADVANCE_TO_CENTER
            if prev_mode != self.MODE_ADVANCE_TO_CENTER:
                self._advance_start_time = self.get_clock().now().nanoseconds / 1e9
                self._advance_done_sent = False
                self.get_logger().info(f'ADVANCE_TO_CENTER: target={self._advance_target_distance:.3f}m')
        elif state in ('TURN', 'TURNING'):
            self._mode = self.MODE_TURN
            if prev_mode != self.MODE_TURN:
                self._turn_start_yaw = self._current_yaw
                self._prev_turn_error = 0.0
                self._turn_done_sent = False
                self._turn_direction = 1 if self._turn_target_rad > 0 else -1
                self._marker_seen_during_turn = False
                self._marker_visible_at_turn_end = False
                self._post_turn_search_mode = False
                self._turn_last_target = self._turn_target_rad  # 변경 감지용
                self._turn_log_count = 0  # 초기 반복 로깅용
                # Reset stall detection for turn
                self._stall_wz_boost = 0.0
                self._consecutive_stalls = 0
                self._stall_pulse_mode = False
                self._last_stall_check_time = 0.0
                self._prev_yaw_for_stall = self._current_yaw
                # 레이스 컨디션 진단: 턴 시작 시 target_marker_id 확인
                tracker_id = self._tracked_marker.id if self._tracked_marker else -1
                tracker_conf = self._tracked_marker.prediction_confidence if self._tracked_marker else 0.0
                tracker_det = self._tracked_marker.is_detected if self._tracked_marker else False
                self.get_logger().info(
                    f'TURN started: start_yaw={math.degrees(self._turn_start_yaw):.1f}deg, '
                    f'target={math.degrees(self._turn_target_rad):.1f}deg, dir={self._turn_direction} | '
                    f'target_marker_id={self._target_marker_id}, '
                    f'tracker=[id={tracker_id}, det={tracker_det}, conf={tracker_conf:.2f}]'
                )
        elif state == 'ALIGN_TO_MARKER':
            self._mode = self.MODE_ALIGN
            if prev_mode != self.MODE_ALIGN:
                # Reset two-phase alignment state (WZ first, then VY)
                self._align_phase = 'WZ'
                self._align_wz_done = False
                # Reset stall detection
                self._stall_vy_boost = 0.0
                self._stall_wz_boost = 0.0
                self._consecutive_stalls = 0
                self._stall_pulse_mode = False
                self._align_action = 'MOVE'
                self._align_action_start_time = 0.0
                self.get_logger().info('ALIGN_TO_MARKER started (WZ → VY phases)')
        elif state == 'APPROACH_VEHICLE':
            self._mode = self.MODE_APPROACH_VEHICLE
        elif state == 'RETREAT_FROM_VEHICLE':
            self._mode = self.MODE_RETREAT_FROM_VEHICLE
        elif state == 'RETREAT_FROM_SLOT':
            self._mode = self.MODE_RETREAT_FROM_SLOT
            if prev_mode != self.MODE_RETREAT_FROM_SLOT:
                self.get_logger().info('RETREAT_FROM_SLOT: fixed +vy lateral movement')
        elif state in ('STOP', 'FINISH', 'STOP_BUMP'):
            self._mode = self.MODE_STOP
        # Parking states
        elif state == 'PARK_DETECT':
            self._mode = self.MODE_PARK_DETECT
        elif state == 'PARK_RECOVERY':
            self._mode = self.MODE_PARK_RECOVERY
            if prev_mode != self.MODE_PARK_RECOVERY:
                self._recovery_start_time = self.get_clock().now().nanoseconds / 1e9
        elif state == 'PARK_ALIGN_MARKER':
            self._mode = self.MODE_PARK_ALIGN_MARKER
            if prev_mode != self.MODE_PARK_ALIGN_MARKER:
                self._park_align_marker_done_sent = False
                self._park_align_phase = 'ANGLE'  # Phase 1: angle, Phase 2: center
                self._park_action = 'MOVE'
                self._park_action_start_time = 0.0
                # Reset parking stall detection
                self._park_stall_boost = 0.0
                self._park_consecutive_stalls = 0
                self._park_stall_pulse_mode = False
                self._park_last_stall_check_time = 0.0
                self._park_prev_stall_value = 0.0
                self.get_logger().info('PARK_ALIGN_MARKER: Phase 1 (ANGLE) → Phase 2 (CENTER)')
        elif state == 'PARK_ALIGN_RECT':
            self._mode = self.MODE_PARK_ALIGN_RECT
            if prev_mode != self.MODE_PARK_ALIGN_RECT:
                self._park_align_rect_done_sent = False
        elif state == 'PARK_FINAL':
            self._mode = self.MODE_PARK_FINAL
            if prev_mode != self.MODE_PARK_FINAL:
                self._park_final_done_sent = False
                self._park_final_phase = 'DISTANCE'  # DISTANCE → FORWARD
                self._park_action = 'MOVE'
                self._park_action_start_time = 0.0
                self._park_final_last_distance = -1.0
                self._park_final_lost_start = 0.0
                # Reset parking stall detection
                self._park_stall_boost = 0.0
                self._park_consecutive_stalls = 0
                self._park_stall_pulse_mode = False
                self._park_last_stall_check_time = 0.0
                self._park_prev_stall_value = 0.0
        elif state == 'PARK':
            # Legacy PARK state - treat as PARK_DETECT
            self._mode = self.MODE_PARK_DETECT
        else:
            self._mode = self.MODE_IDLE

        self.get_logger().debug(f'Mode changed: {self._mode}')

    def _target_marker_callback(self, msg: Int32):
        """Handle target marker changes."""
        self._target_marker_id = msg.data

    def _target_slot_callback(self, msg: Int32):
        """Handle target slot changes."""
        self._target_slot_id = msg.data

    def _enable_callback(self, msg: Bool):
        """Handle enable/disable commands."""
        self._enable = msg.data
        if not self._enable:
            self._stop()

    def _turn_target_callback(self, msg):
        """Handle turn target angle."""
        raw_rad = msg.data / 100.0
        new_target = raw_rad * self.turn_scale
        # TURN 중 target이 크게 바뀌면 재초기화 (레이스 컨디션 보정)
        if (self._mode == self.MODE_TURN and not self._turn_done_sent and
                abs(new_target - getattr(self, '_turn_last_target', new_target)) > 0.1):
            self._turn_start_yaw = self._current_yaw
            self._turn_direction = 1 if new_target > 0 else -1
            self._prev_turn_error = 0.0
            self._turn_last_target = new_target
            self.get_logger().info(
                f'TURN reinit: target changed to {math.degrees(new_target):.1f}deg, '
                f'start_yaw={math.degrees(self._turn_start_yaw):.1f}deg, dir={self._turn_direction}'
            )
        self._turn_target_rad = new_target

    def _marker_callback(self, msg):
        """Handle tracked marker updates."""
        self._tracked_marker = msg

        # Track if target marker is seen during turn
        if self._mode == self.MODE_TURN and not self._turn_done_sent:
            if (msg.id == self._target_marker_id and
                (msg.is_detected or msg.prediction_confidence > 0.3)):
                self._marker_seen_during_turn = True

    def _side_markers_callback(self, msg):
        """Handle side camera marker updates for parking."""
        if not msg.markers:
            self._side_marker = None
            return

        # Find the target slot marker or nearest parking marker
        best_marker = None
        for marker in msg.markers:
            if marker.id < 16 or marker.id > 27:
                continue  # Not a parking marker
            if marker.id == self._target_slot_id:
                best_marker = marker
                break
            if best_marker is None or marker.distance < best_marker.distance:
                best_marker = marker

        self._side_marker = best_marker

    def _slot_rect_callback(self, msg):
        """Handle slot rectangle detection updates."""
        self._slot_rect = msg

    def _imu_callback(self, msg: Imu):
        """Handle IMU data updates."""
        self._imu_data = msg
        self._current_yaw = quaternion_to_yaw(msg.orientation)

    def _control_loop(self):
        """Main control loop."""
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0
        detail = ''

        if not self._enable:
            self._publish_cmd(cmd)
            self._publish_state('DISABLED', error_x, error_y, error_yaw, '')
            return

        if self._mode == self.MODE_IDLE:
            detail = 'Waiting for mission'

        elif self._mode == self.MODE_DRIVE:
            cmd, error_x, error_y, error_yaw, detail = self._drive_with_marker_only()

        elif self._mode == self.MODE_STOP_AT_MARKER:
            detail = f'Stopped at marker, dist={self._advance_target_distance:.3f}m'

        elif self._mode == self.MODE_ADVANCE_TO_CENTER:
            cmd, error_x, error_y, error_yaw, detail = self._advance_to_center()

        elif self._mode == self.MODE_TURN:
            cmd, error_x, error_y, error_yaw, detail = self._turn_control()

        elif self._mode == self.MODE_ALIGN:
            cmd, error_x, error_y, error_yaw, detail = self._align_to_marker()

        elif self._mode == self.MODE_STOP:
            detail = 'Stopped'

        # Parking modes
        elif self._mode == self.MODE_PARK_DETECT:
            cmd, error_x, error_y, error_yaw, detail = self._park_detect_control()

        elif self._mode == self.MODE_PARK_RECOVERY:
            cmd, error_x, error_y, error_yaw, detail = self._park_recovery_control()

        elif self._mode == self.MODE_PARK_ALIGN_MARKER:
            cmd, error_x, error_y, error_yaw, detail = self._park_align_marker_control()

        elif self._mode == self.MODE_PARK_ALIGN_RECT:
            cmd, error_x, error_y, error_yaw, detail = self._park_align_rect_control()

        elif self._mode == self.MODE_PARK_FINAL:
            cmd, error_x, error_y, error_yaw, detail = self._park_final_control()

        # Approach/retreat (hardcoded forward/backward movement) - 주석처리: 접근/후퇴 이동 비활성화
        # elif self._mode == self.MODE_APPROACH_VEHICLE:
        #     cmd.linear.x = self.approach_speed
        #     detail = 'Approaching vehicle'
        # elif self._mode == self.MODE_RETREAT_FROM_VEHICLE:
        #     cmd.linear.x = -self.approach_speed
        #     detail = 'Retreating from vehicle'

        # Retreat from parking slot: 슬롯→도로 복귀 (side cam 왼쪽, +vy=오른쪽)
        elif self._mode == self.MODE_RETREAT_FROM_SLOT:
            cmd.linear.y = self.retreat_from_slot_speed
            detail = f'Retreat from slot: +vy={cmd.linear.y:.3f}'

        # Apply velocity limits
        cmd.linear.x = self._clamp(cmd.linear.x, -self.max_vx, self.max_vx)
        cmd.linear.y = self._clamp(cmd.linear.y, -self.max_vy, self.max_vy)
        cmd.angular.z = self._clamp(cmd.angular.z, -self.max_wz, self.max_wz)

        self._publish_cmd(cmd)
        self._publish_state(self._mode, error_x, error_y, error_yaw, detail)

        # Periodic log output
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log_time >= self._log_interval:
            self._last_log_time = now
            self.get_logger().info(
                f'[{self._mode}] target={self._target_marker_id}, slot={self._target_slot_id}, '
                f'vx={cmd.linear.x:.4f}, vy={cmd.linear.y:.4f}, wz={cmd.angular.z:.4f} | {detail}'
            )

    def _drive_with_marker_only(self):
        """Marker-only driving with stall detection for alignment."""
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0
        now = self.get_clock().now().nanoseconds / 1e9

        marker_ok = (self._tracked_marker is not None and
                     self._tracked_marker.id == self._target_marker_id and
                     self._tracked_marker.id < 16 and
                     (self._tracked_marker.is_detected or
                      self._tracked_marker.prediction_confidence > 0.3))

        if marker_ok:
            marker = self._tracked_marker
            distance = marker.distance
            angle = marker.angle

            self._last_marker_distance = distance
            self._last_marker_id = marker.id
            self._drive_search_start_time = 0.0  # Reset search creep timer

            error_x = distance
            error_yaw = angle

            if distance < self.marker_reach_dist:
                self._publish_marker_reached(marker.id)
                return cmd, error_x, error_y, error_yaw, f'Reached marker {marker.id}'

            pred = 'pred' if not marker.is_detected else 'det'

            # Always move forward (vx) + correct laterally (vy) + gentle steer (wz)
            # Slow down slightly when off-center, but keep moving
            angle_factor = max(0.5, 1.0 - abs(angle) / (math.pi / 3))
            vx = self.marker_vx_kp * (distance - self.marker_min_dist) * angle_factor
            vx = max(0.0, vx)

            # Lateral correction
            vy = -self.marker_vy_kp * angle

            # Gentle steering: 30% of full wz gain for tracking
            wz = self.marker_wz_kp * angle * 0.3
            cmd.angular.z = -wz  # 모터 부호 반전

            cmd.linear.x = vx
            cmd.linear.y = vy
            detail = f'[DRIVE] d={distance:.2f}m, a={math.degrees(angle):.1f}deg, wz={-wz:.3f} [{pred}]'

            return cmd, error_x, error_y, error_yaw, detail

        if (self._last_marker_id == self._target_marker_id and
            self._last_marker_distance < self.marker_lost_reach_dist):
            self._publish_marker_reached(self._last_marker_id)
            self.get_logger().info(
                f'Marker {self._last_marker_id} lost at {self._last_marker_distance:.2f}m - counted as reached'
            )
            return cmd, error_x, error_y, error_yaw, f'Marker lost but reached'

        # 긴 구간(4→10, 9→15 등)에서 마커 안 보이면 전진하며 탐색
        transition = (self._drive_from_marker_id, self._target_marker_id)
        if transition in self._drive_search_creep_pairs:
            if self._drive_search_start_time == 0.0:
                self._drive_search_start_time = now
            search_elapsed = now - self._drive_search_start_time
            if search_elapsed > self._drive_search_creep_delay:
                cmd.linear.x = self._drive_search_creep_vx
                detail = f'No marker {self._target_marker_id}, creeping forward ({search_elapsed:.1f}s)'
                return cmd, error_x, error_y, error_yaw, detail

        detail = f'No marker {self._target_marker_id}, waiting...'
        return cmd, error_x, error_y, error_yaw, detail

    def _advance_to_center(self):
        """Advance to marker center (fixed time)."""
        cmd = Twist()
        error_x = self._advance_target_distance
        error_y = 0.0
        error_yaw = 0.0

        advance_vx = self.advance_vx
        required_time = self.advance_time

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._advance_start_time

        if elapsed >= required_time:
            if not self._advance_done_sent:
                self._publish_align_done()
                self._advance_done_sent = True
            detail = f'Advance complete, elapsed={elapsed:.2f}s'
            return cmd, 0.0, error_y, error_yaw, detail

        cmd.linear.x = advance_vx
        remaining = required_time - elapsed
        detail = f'Advancing: {elapsed:.2f}/{required_time:.2f}s, remaining={remaining:.2f}s'

        return cmd, error_x, error_y, error_yaw, detail

    def _publish_align_done(self):
        """Notify alignment/advance complete."""
        msg = Bool()
        msg.data = True
        if not hasattr(self, 'pub_align_done'):
            self.pub_align_done = self.create_publisher(Bool, '/mission/align_done', 10)
        self.pub_align_done.publish(msg)
        self.get_logger().info('Alignment complete')

    def _align_to_marker(self):
        """
        Turn 후 마커 정렬 (2단계: WZ → VY → 주행).

        Phase 1 (WZ): 회전으로 마커 방향 먼저 맞춤
        Phase 2 (VY): 좌우 이동으로 마커 중심 미세 정렬

        추가 기능:
        - 마커 못 찾으면 turn 방향으로 회전하며 탐색
        - 스톨 감지 시 파워 부스트
        """
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0
        now = self.get_clock().now().nanoseconds / 1e9

        marker_ok = (self._tracked_marker is not None and
                     self._tracked_marker.id == self._target_marker_id and
                     (self._tracked_marker.is_detected or
                      self._tracked_marker.prediction_confidence > 0.3))

        # ========================================
        # Post-turn marker search mode
        # ========================================
        if not marker_ok:
            # Enter search mode if marker not found
            if not self._post_turn_search_mode:
                self._post_turn_search_mode = True
                self._post_turn_search_start_time = now

                # Determine search direction
                if self._marker_seen_during_turn and not self._marker_visible_at_turn_end:
                    # Marker was seen but now lost - turn opposite direction
                    self._post_turn_search_direction = -self._turn_direction
                    self.get_logger().info(f'Marker lost after turn - searching opposite (dir={self._post_turn_search_direction})')
                else:
                    # Marker never seen or still not visible - continue in turn direction
                    self._post_turn_search_direction = self._turn_direction
                    self.get_logger().info(f'Marker not found - searching in turn direction (dir={self._post_turn_search_direction})')

            # Track elapsed time (no timeout - keep searching)
            search_elapsed = now - self._post_turn_search_start_time

            # Progressive rotation speed - starts slow, increases over time
            # Base: turn_wz_min, ramps up to turn_wz over 5 seconds
            base_wz = self.turn_wz_min
            max_wz = self.turn_wz
            ramp_time = 5.0  # seconds to reach max speed
            ramp_factor = min(1.0, search_elapsed / ramp_time)
            wz = (base_wz + (max_wz - base_wz) * ramp_factor) * self._post_turn_search_direction
            cmd.angular.z = -wz  # Negate for motor direction

            detail = f'[SEARCH] dir={self._post_turn_search_direction}, wz={abs(wz):.3f}, elapsed={search_elapsed:.1f}s'
            return cmd, error_x, error_y, error_yaw, detail

        # ========================================
        # Marker found - exit search mode
        # ========================================
        if self._post_turn_search_mode:
            self._post_turn_search_mode = False
            self.get_logger().info('Marker found - exiting search mode')

        marker = self._tracked_marker
        angle = marker.angle
        error_yaw = angle
        pred = 'pred' if not marker.is_detected else 'det'

        # ========================================
        # Stall detection
        # ========================================
        if now - self._last_stall_check_time >= self._stall_check_interval:
            self._check_and_handle_stall(angle)
            self._last_stall_check_time = now

        # ========================================
        # Alignment thresholds (strict)
        # ========================================
        wz_threshold = self.marker_align_threshold * 0.5  # ~3deg for WZ phase
        vy_threshold = self.marker_align_threshold * 0.3  # ~2deg for final VY
        switch_threshold = self.marker_align_threshold * 0.6  # threshold to switch phases

        # ========================================
        # Phase 1: WZ alignment (회전으로 방향 맞춤)
        # ========================================
        if self._align_phase == 'WZ':
            if abs(angle) < switch_threshold:
                # WZ phase done, move to VY
                self._align_phase = 'VY'
                # Reset stall detection and settle timer for VY phase
                self._stall_wz_boost = 0.0
                self._consecutive_stalls = 0
                self._align_action = 'MOVE'
                self._align_action_start_time = 0.0
                self.get_logger().info(f'WZ align done (angle={math.degrees(angle):.1f}deg), starting VY phase')
            else:
                # Move-pause-observe pattern
                if self._align_action_start_time == 0.0:
                    self._align_action = 'MOVE'
                    self._align_action_start_time = now

                action_elapsed = now - self._align_action_start_time

                if self._align_action == 'SETTLE':
                    # Pause: wait for camera to observe result
                    if action_elapsed >= self.align_settle_duration:
                        self._align_action = 'MOVE'
                        self._align_action_start_time = now
                    detail = f'[ALIGN-WZ] SETTLE {action_elapsed:.2f}/{self.align_settle_duration:.1f}s, a={math.degrees(angle):.1f}deg [{pred}]'
                    return cmd, error_x, error_y, error_yaw, detail

                # MOVE phase
                if action_elapsed >= self.align_move_duration:
                    # Switch to settle
                    self._align_action = 'SETTLE'
                    self._align_action_start_time = now
                    detail = f'[ALIGN-WZ] → SETTLE, a={math.degrees(angle):.1f}deg [{pred}]'
                    return cmd, error_x, error_y, error_yaw, detail

                # Apply WZ control with stall boost
                wz = self.marker_wz_kp * angle * 2.0

                # Apply stall boost
                if self._stall_pulse_mode:
                    pulse_elapsed = now - self._stall_pulse_start_time
                    if pulse_elapsed < self._stall_pulse_duration:
                        boost = self._stall_wz_boost * 3.0
                        wz = wz + (boost if wz > 0 else -boost)
                    else:
                        self._stall_pulse_mode = False
                else:
                    wz = wz + (self._stall_wz_boost if wz > 0 else -self._stall_wz_boost)

                # Minimum speed guarantee (적재장치 무게 보상)
                min_wz = 0.04 + self._stall_wz_boost
                if abs(wz) < min_wz and abs(angle) > 0.03:
                    wz = min_wz if angle > 0 else -min_wz

                cmd.angular.z = -wz  # Negate for motor direction

                detail = f'[ALIGN-WZ] MOVE a={math.degrees(angle):.1f}deg, wz={-wz:.3f}, boost={self._stall_wz_boost:.3f} [{pred}]'
                return cmd, error_x, error_y, error_yaw, detail

        # ========================================
        # Phase 2: VY alignment (좌우 이동 미세 조정)
        # ========================================
        if self._align_phase == 'VY':
            # Final alignment check
            if abs(angle) < vy_threshold:
                # Check if stable for a short time
                if not hasattr(self, '_align_stable_start'):
                    self._align_stable_start = now
                elif now - self._align_stable_start > 0.3:  # Stable for 0.3s
                    self._publish_align_done()
                    delattr(self, '_align_stable_start')
                    return cmd, error_x, error_y, error_yaw, f'Aligned to marker {marker.id}'
            else:
                # Reset stable timer if not aligned
                if hasattr(self, '_align_stable_start'):
                    delattr(self, '_align_stable_start')

            # If angle drifted too much, go back to WZ
            if abs(angle) > self.marker_align_threshold:
                self._align_phase = 'WZ'
                self._stall_vy_boost = 0.0
                self._consecutive_stalls = 0
                self._align_action = 'MOVE'
                self._align_action_start_time = 0.0
                self.get_logger().info(f'Angle drifted ({math.degrees(angle):.1f}deg), back to WZ phase')
                return self._align_to_marker()  # Recursive call for WZ

            # Move-pause-observe pattern for VY
            if self._align_action_start_time == 0.0:
                self._align_action = 'MOVE'
                self._align_action_start_time = now

            action_elapsed = now - self._align_action_start_time

            if self._align_action == 'SETTLE':
                # Pause: wait for camera to observe result
                if action_elapsed >= self.align_settle_duration:
                    self._align_action = 'MOVE'
                    self._align_action_start_time = now
                detail = f'[ALIGN-VY] SETTLE {action_elapsed:.2f}/{self.align_settle_duration:.1f}s, a={math.degrees(angle):.1f}deg [{pred}]'
                return cmd, error_x, error_y, error_yaw, detail

            # MOVE phase
            if action_elapsed >= self.align_move_duration:
                # Switch to settle
                self._align_action = 'SETTLE'
                self._align_action_start_time = now
                detail = f'[ALIGN-VY] → SETTLE, a={math.degrees(angle):.1f}deg [{pred}]'
                return cmd, error_x, error_y, error_yaw, detail

            # VY control for fine adjustment
            vy = -self.marker_vy_kp * angle * 3.0

            # Apply stall boost
            if self._stall_pulse_mode:
                pulse_elapsed = now - self._stall_pulse_start_time
                if pulse_elapsed < self._stall_pulse_duration:
                    boost = self._stall_vy_boost * 3.0
                    vy = vy + (boost if vy > 0 else -boost)
                else:
                    self._stall_pulse_mode = False
            else:
                vy = vy + (self._stall_vy_boost if vy > 0 else -self._stall_vy_boost)

            # Minimum speed guarantee (적재장치 무게 보상)
            min_speed = 0.015 + self._stall_vy_boost
            if abs(vy) < min_speed and abs(angle) > 0.02:
                vy = min_speed if angle < 0 else -min_speed

            cmd.linear.y = vy

            detail = f'[ALIGN-VY] MOVE a={math.degrees(angle):.1f}deg, vy={vy:.3f}, boost={self._stall_vy_boost:.3f} [{pred}]'
            return cmd, error_x, error_y, error_yaw, detail

        detail = f'No marker {self._target_marker_id}, waiting...'
        return cmd, error_x, error_y, error_yaw, detail

    def _check_and_handle_stall(self, current_angle: float):
        """
        스톨 감지 및 파워 부스트 처리.

        - IMU yaw 변화량으로 회전 스톨 감지 (WZ phase)
        - 마커 각도 변화량으로 이동 스톨 감지 (VY phase)
        - 연속 스톨 시 점진적 파워 증가 또는 펄스 모드
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # Calculate changes
        yaw_change = abs(self._current_yaw - self._prev_yaw_for_stall)
        angle_change = abs(current_angle - self._prev_marker_angle_for_stall)

        # Normalize yaw change
        if yaw_change > math.pi:
            yaw_change = 2 * math.pi - yaw_change

        # Detect stall based on current phase (WZ first, then VY)
        is_stalled = False

        if self._align_phase == 'WZ':
            # WZ phase: check rotation (yaw change)
            if yaw_change < self._stall_threshold_wz:
                is_stalled = True
                self.get_logger().debug(f'WZ stall detected: yaw_change={math.degrees(yaw_change):.2f}deg')
        elif self._align_phase == 'VY':
            # VY phase: check marker angle change (indicates lateral movement)
            if angle_change < self._stall_threshold_vy:
                is_stalled = True
                self.get_logger().debug(f'VY stall detected: angle_change={math.degrees(angle_change):.2f}deg')

        if is_stalled:
            self._consecutive_stalls += 1

            # Gradual boost increase based on current phase
            if self._align_phase == 'WZ':
                self._stall_wz_boost = min(
                    self._stall_wz_boost + self._stall_boost_increment,
                    self._stall_max_boost
                )
            elif self._align_phase == 'VY':
                self._stall_vy_boost = min(
                    self._stall_vy_boost + self._stall_boost_increment,
                    self._stall_max_boost
                )

            # Enter pulse mode after several consecutive stalls
            if self._consecutive_stalls >= 3 and not self._stall_pulse_mode:
                self._stall_pulse_mode = True
                self._stall_pulse_start_time = now
                self.get_logger().info(
                    f'Entering pulse mode: stalls={self._consecutive_stalls}, '
                    f'vy_boost={self._stall_vy_boost:.3f}, wz_boost={self._stall_wz_boost:.3f}'
                )
        else:
            # Reset consecutive stalls (but keep boost for now)
            self._consecutive_stalls = max(0, self._consecutive_stalls - 1)

            # Gradually reduce boost when moving normally
            if self._consecutive_stalls == 0:
                self._stall_vy_boost = max(0, self._stall_vy_boost - self._stall_boost_increment * 0.5)
                self._stall_wz_boost = max(0, self._stall_wz_boost - self._stall_boost_increment * 0.5)

        # Update previous values
        self._prev_yaw_for_stall = self._current_yaw
        self._prev_marker_angle_for_stall = current_angle

    def _turn_control(self):
        """
        In-place turn control - marker-only completion.

        일정 속도로 turn_direction 방향으로 회전하면서
        타겟 마커가 감지되면 즉시 종료 → ALIGN 모드에서 정밀 정렬.
        IMU 기반 완료 판단 없음 (모터 가속 오버슈트 방지).
        """
        cmd = Twist()

        if self._turn_done_sent:
            return cmd, 0.0, 0.0, 0.0, 'Turn done, waiting for state change'

        now = self.get_clock().now().nanoseconds / 1e9

        # 초기 반복 진단 카운터
        self._turn_log_count = getattr(self, '_turn_log_count', 0) + 1

        # 타겟 마커가 감지되면 즉시 턴 종료
        marker_ok = (self._tracked_marker is not None and
                     self._tracked_marker.id == self._target_marker_id and
                     (self._tracked_marker.is_detected or
                      self._tracked_marker.prediction_confidence > 0.5))

        # 처음 5회 반복에서 marker_ok 상태 로깅 (레이스 컨디션 진단)
        if self._turn_log_count <= 5:
            tracker_id = self._tracked_marker.id if self._tracked_marker else -1
            tracker_det = self._tracked_marker.is_detected if self._tracked_marker else False
            tracker_conf = self._tracked_marker.prediction_confidence if self._tracked_marker else 0.0
            self.get_logger().info(
                f'[TURN_ITER_{self._turn_log_count}] marker_ok={marker_ok}, '
                f'target_id={self._target_marker_id}, '
                f'tracker=[id={tracker_id}, det={tracker_det}, conf={tracker_conf:.2f}], '
                f'yaw={math.degrees(self._current_yaw):.1f}'
            )

        # ========================================
        # Stall detection for TURN
        # ========================================
        if now - self._last_stall_check_time >= self._stall_check_interval:
            yaw_change = abs(self._current_yaw - self._prev_yaw_for_stall)
            if yaw_change > math.pi:
                yaw_change = 2 * math.pi - yaw_change

            if yaw_change < self._stall_threshold_wz:
                self._consecutive_stalls += 1
                self._stall_wz_boost = min(
                    self._stall_wz_boost + self._stall_boost_increment,
                    self._stall_max_boost
                )
                if self._consecutive_stalls >= 3 and not self._stall_pulse_mode:
                    self._stall_pulse_mode = True
                    self._stall_pulse_start_time = now
                    self.get_logger().info(
                        f'TURN stall pulse: stalls={self._consecutive_stalls}, '
                        f'boost={self._stall_wz_boost:.3f}'
                    )
                elif self._consecutive_stalls == 1:
                    self.get_logger().info(
                        f'TURN stall detected: yaw_change={math.degrees(yaw_change):.2f}deg, '
                        f'boost={self._stall_wz_boost:.3f}'
                    )
            else:
                self._consecutive_stalls = max(0, self._consecutive_stalls - 1)
                if self._consecutive_stalls == 0:
                    self._stall_wz_boost = max(0, self._stall_wz_boost - self._stall_boost_increment * 0.5)

            self._prev_yaw_for_stall = self._current_yaw
            self._last_stall_check_time = now

        if marker_ok:
            marker_angle = self._tracked_marker.angle

            # 마커 발견 후: angle이 충분히 작으면 턴 완료
            if abs(marker_angle) < self.marker_align_threshold:
                self._marker_visible_at_turn_end = True
                self._publish_turn_done()
                self._turn_done_sent = True
                self.get_logger().info(
                    f'Turn complete: marker {self._target_marker_id} centered! '
                    f'angle={math.degrees(marker_angle):.1f}deg'
                )
                return cmd, 0.0, 0.0, 0.0, f'Marker {self._target_marker_id} centered, turn done'

            # 마커 발견했지만 아직 중앙이 아님 → 마커 각도 기반으로 천천히 회전
            wz = self.marker_wz_kp * marker_angle
            # 최소 속도 보장
            if abs(wz) < self.turn_wz_min:
                wz = self.turn_wz_min if marker_angle > 0 else -self.turn_wz_min
            # 최대 속도 제한
            wz = max(-self.turn_wz, min(self.turn_wz, wz))

            # Apply stall boost
            if self._stall_pulse_mode:
                pulse_elapsed = now - self._stall_pulse_start_time
                if pulse_elapsed < self._stall_pulse_duration:
                    boost = self._stall_wz_boost * 3.0
                    wz = wz + (boost if wz > 0 else -boost)
                else:
                    self._stall_pulse_mode = False
            else:
                wz = wz + (self._stall_wz_boost if wz > 0 else -self._stall_wz_boost)

            cmd.angular.z = -wz  # 모터 부호 반전

            detail = (f'Turn: marker found, aligning '
                      f'a={math.degrees(marker_angle):.1f}deg, wz={-wz:.3f}, boost={self._stall_wz_boost:.3f}')
            return cmd, 0.0, 0.0, 0.0, detail

        # ========================================
        # IMU fallback: 마커 한 번도 안 보인 경우 각도 기반 턴 완료
        # (9→15, 4→10 등 먼 거리에서 마커가 카메라 범위 밖)
        # ========================================
        if not self._marker_seen_during_turn:
            diff = self._current_yaw - self._turn_start_yaw
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            total_rotation = abs(diff)
            target_rotation = abs(self._turn_target_rad)
            if target_rotation > 0.1 and total_rotation >= target_rotation * 0.9:
                self._publish_turn_done()
                self._turn_done_sent = True
                self.get_logger().info(
                    f'Turn complete (IMU fallback): rotated {math.degrees(total_rotation):.1f}deg, '
                    f'target {math.degrees(self._turn_target_rad):.1f}deg (marker never seen)'
                )
                return cmd, 0.0, 0.0, 0.0, 'Turn done (IMU, marker not found)'

        # 마커 미발견 → 일정 속도로 계속 회전
        # _turn_direction: +1 = CCW (좌회전), -1 = CW (우회전)
        wz = self.turn_wz * self._turn_direction

        # Apply stall boost
        if self._stall_pulse_mode:
            pulse_elapsed = now - self._stall_pulse_start_time
            if pulse_elapsed < self._stall_pulse_duration:
                boost = self._stall_wz_boost * 3.0
                wz = wz + (boost if wz > 0 else -boost)
            else:
                self._stall_pulse_mode = False
        else:
            wz = wz + (self._stall_wz_boost if wz > 0 else -self._stall_wz_boost)

        cmd.angular.z = -wz  # 모터 부호 반전

        detail = (f'Turn: rotating dir={self._turn_direction}, '
                  f'wz={-wz:.3f}, boost={self._stall_wz_boost:.3f}, yaw={math.degrees(self._current_yaw):.1f}')

        return cmd, 0.0, 0.0, 0.0, detail

    # ==========================================
    # Parking Control Methods
    # ==========================================

    def _check_park_stall(self, current_value: float, threshold: float):
        """
        주차 상태 스톨 감지 및 파워 부스트.
        current_value: 현재 추적 값 (angle, offset_x, distance 등)
        threshold: 변화량 임계값 (이하이면 스톨)
        """
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._park_last_stall_check_time < self._stall_check_interval:
            return

        change = abs(current_value - self._park_prev_stall_value)

        if change < threshold:
            self._park_consecutive_stalls += 1
            self._park_stall_boost = min(
                self._park_stall_boost + self._stall_boost_increment,
                self._stall_max_boost
            )
            if self._park_consecutive_stalls >= 3 and not self._park_stall_pulse_mode:
                self._park_stall_pulse_mode = True
                self._park_stall_pulse_start_time = now
                self.get_logger().info(
                    f'PARK stall pulse: stalls={self._park_consecutive_stalls}, '
                    f'boost={self._park_stall_boost:.3f}'
                )
            elif self._park_consecutive_stalls == 1:
                self.get_logger().info(
                    f'PARK stall detected: change={change:.4f}, '
                    f'boost={self._park_stall_boost:.3f}'
                )
        else:
            self._park_consecutive_stalls = max(0, self._park_consecutive_stalls - 1)
            if self._park_consecutive_stalls == 0:
                self._park_stall_boost = max(0, self._park_stall_boost - self._stall_boost_increment * 0.5)

        self._park_prev_stall_value = current_value
        self._park_last_stall_check_time = now

    def _apply_park_stall_boost(self, speed: float) -> float:
        """스톨 부스트를 속도에 적용 (방향 유지)."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self._park_stall_pulse_mode:
            pulse_elapsed = now - self._park_stall_pulse_start_time
            if pulse_elapsed < self._stall_pulse_duration:
                boost = self._park_stall_boost * 3.0
                return speed + (boost if speed > 0 else -boost)
            else:
                self._park_stall_pulse_mode = False
        if self._park_stall_boost > 0:
            return speed + (self._park_stall_boost if speed > 0 else -self._park_stall_boost)
        return speed

    def _park_detect_control(self):
        """
        PARK_DETECT: Creep forward to find slot marker with side camera.
        Side camera looks right, so marker.angle indicates front/back offset.
        """
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0

        if self._side_marker is None:
            # No marker - creep forward slowly
            cmd.linear.x = self.park_creep_vx
            detail = f'Searching for slot {self._target_slot_id}, creeping forward'
        else:
            # Marker found - manager_node will handle state transition
            marker = self._side_marker
            error_x = marker.distance
            error_yaw = marker.angle
            detail = f'Found marker {marker.id}, d={marker.distance:.2f}m, a={math.degrees(marker.angle):.1f}deg'

        return cmd, error_x, error_y, error_yaw, detail

    def _park_recovery_control(self):
        """
        PARK_RECOVERY: Move forward/backward to correct zone.
        Direction determined by mission_manager based on detected vs target zone.
        """
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0

        # Get recovery direction from mission state
        # Note: The actual direction comes from mission_manager publishing state
        # For now, just move forward slowly - manager handles timing
        cmd.linear.x = self.park_creep_vx

        detail = f'Recovery move in progress'

        return cmd, error_x, error_y, error_yaw, detail

    def _park_align_marker_control(self):
        """
        PARK_ALIGN_MARKER: 2단계 정렬 (Side camera 기준).

        Phase 1 (ANGLE): marker.angle 기반 대략적 정렬
        - angle > 0: 마커가 카메라 왼쪽 = 로봇이 앞에 → 후진
        - angle < 0: 마커가 카메라 오른쪽 = 로봇이 뒤에 → 전진
        - park_marker_threshold 이하이면 Phase 2로 전환

        Phase 2 (CENTER): pose.position.x (tvec[0]) 기반 미세 중앙 정렬
        - offset_x > 0: 마커가 카메라 오른쪽 = 로봇 앞 → 후진
        - offset_x < 0: 마커가 카메라 왼쪽 = 로봇 뒤 → 전진
        - park_center_threshold 이하이면 완료
        """
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0

        if self._park_align_marker_done_sent:
            return cmd, error_x, error_y, error_yaw, 'Marker align done, waiting'

        if self._side_marker is None:
            detail = 'No side marker, waiting...'
            return cmd, error_x, error_y, error_yaw, detail

        marker = self._side_marker
        now = self.get_clock().now().nanoseconds / 1e9
        angle = marker.angle
        offset_x = marker.pose.position.x - self.park_side_camera_offset_x

        # ========================================
        # Phase 1: ANGLE 기반 대략적 정렬
        # ========================================
        if self._park_align_phase == 'ANGLE':
            error_yaw = angle

            # angle이 threshold 이하이면 Phase 2로 전환
            if abs(angle) < self.park_marker_threshold:
                self._park_align_phase = 'CENTER'
                self._park_action = 'MOVE'
                self._park_action_start_time = 0.0
                # Phase 전환: stall 추적값 리셋 (angle → offset_x)
                self._park_stall_boost = 0.0
                self._park_consecutive_stalls = 0
                self._park_stall_pulse_mode = False
                self._park_prev_stall_value = offset_x
                if hasattr(self, '_park_align_stable_start'):
                    delattr(self, '_park_align_stable_start')
                self.get_logger().info(
                    f'Phase 1 (ANGLE) done: angle={math.degrees(angle):.1f}deg, '
                    f'switching to Phase 2 (CENTER), offset={offset_x*100:.1f}cm'
                )
                # Fall through to CENTER phase below

            else:
                # Move-pause-observe pattern
                if self._park_action_start_time == 0.0:
                    self._park_action = 'MOVE'
                    self._park_action_start_time = now

                action_elapsed = now - self._park_action_start_time

                if self._park_action == 'SETTLE':
                    if action_elapsed >= self.align_settle_duration:
                        self._park_action = 'MOVE'
                        self._park_action_start_time = now
                    detail = f'[ANGLE] SETTLE {action_elapsed:.2f}s, a={math.degrees(angle):.1f}deg'
                    return cmd, error_x, error_y, error_yaw, detail

                if action_elapsed >= self.align_move_duration:
                    self._park_action = 'SETTLE'
                    self._park_action_start_time = now
                    detail = f'[ANGLE] → SETTLE, a={math.degrees(angle):.1f}deg'
                    return cmd, error_x, error_y, error_yaw, detail

                # Stall check (angle 변화 추적)
                self._check_park_stall(angle, 0.003)  # ~0.17deg

                # Control: angle -> vx
                # angle > 0 → 마커가 뒤에 → 후진 필요 → -vx
                vx = -self.park_align_kp * angle
                vx = self._clamp(vx, -self.park_max_vx, self.park_max_vx)

                if abs(vx) < self.park_min_speed and abs(angle) > self.park_marker_threshold * 0.5:
                    vx = -self.park_min_speed if angle > 0 else self.park_min_speed

                vx = self._apply_park_stall_boost(vx)
                cmd.linear.x = vx
                detail = f'[ANGLE] MOVE: a={math.degrees(angle):.1f}deg, vx={vx:.3f}, boost={self._park_stall_boost:.3f}'
                return cmd, error_x, error_y, error_yaw, detail

        # ========================================
        # Phase 2: CENTER 기반 미세 정렬
        # ========================================
        if self._park_align_phase == 'CENTER':
            error_x = offset_x

            # angle이 크게 벗어나면 Phase 1로 복귀
            if abs(angle) > self.park_marker_threshold * 3.0:
                self._park_align_phase = 'ANGLE'
                self._park_action = 'MOVE'
                self._park_action_start_time = 0.0
                # Phase 전환: stall 추적값 리셋 (offset_x → angle)
                self._park_stall_boost = 0.0
                self._park_consecutive_stalls = 0
                self._park_stall_pulse_mode = False
                self._park_prev_stall_value = angle
                if hasattr(self, '_park_align_stable_start'):
                    delattr(self, '_park_align_stable_start')
                self.get_logger().info(
                    f'Angle drifted ({math.degrees(angle):.1f}deg), back to Phase 1 (ANGLE)'
                )
                return cmd, error_x, error_y, error_yaw, f'[CENTER] → ANGLE (drift)'

            # 중앙 정렬 완료 체크 (stability)
            if abs(offset_x) < self.park_center_threshold:
                if not hasattr(self, '_park_align_stable_start'):
                    self._park_align_stable_start = now
                elif now - self._park_align_stable_start > 0.5:
                    if hasattr(self, '_park_align_stable_start'):
                        delattr(self, '_park_align_stable_start')
                    self._publish_park_align_marker_done()
                    self._park_align_marker_done_sent = True
                    detail = f'Centered! off={offset_x*100:.1f}cm'
                    return cmd, error_x, error_y, error_yaw, detail
                detail = f'[CENTER] stable: off={offset_x*100:.1f}cm, {now - self._park_align_stable_start:.1f}/0.5s'
                return cmd, error_x, error_y, error_yaw, detail
            else:
                if hasattr(self, '_park_align_stable_start'):
                    delattr(self, '_park_align_stable_start')

            # Move-pause-observe pattern
            if self._park_action_start_time == 0.0:
                self._park_action = 'MOVE'
                self._park_action_start_time = now

            action_elapsed = now - self._park_action_start_time

            if self._park_action == 'SETTLE':
                if action_elapsed >= self.align_settle_duration:
                    self._park_action = 'MOVE'
                    self._park_action_start_time = now
                detail = f'[CENTER] SETTLE {action_elapsed:.2f}s, off={offset_x*100:.1f}cm'
                return cmd, error_x, error_y, error_yaw, detail

            if action_elapsed >= self.align_move_duration:
                self._park_action = 'SETTLE'
                self._park_action_start_time = now
                detail = f'[CENTER] → SETTLE, off={offset_x*100:.1f}cm'
                return cmd, error_x, error_y, error_yaw, detail

            # Stall check (offset_x 변화 추적)
            self._check_park_stall(offset_x, 0.001)  # 1mm

            # Control: offset_x -> vx (미터 단위 직접 제어)
            # offset_x > 0 → 마커가 앞에 → 후진 (-vx)
            vx = -self.park_align_kp * offset_x
            vx = self._clamp(vx, -self.park_max_vx, self.park_max_vx)

            if abs(vx) < self.park_min_speed and abs(offset_x) > self.park_center_threshold * 0.5:
                vx = -self.park_min_speed if offset_x > 0 else self.park_min_speed

            vx = self._apply_park_stall_boost(vx)
            cmd.linear.x = vx
            detail = f'[CENTER] MOVE: off={offset_x*100:.1f}cm, vx={vx:.3f}, boost={self._park_stall_boost:.3f}'
            return cmd, error_x, error_y, error_yaw, detail

        detail = f'No side marker, waiting...'
        return cmd, error_x, error_y, error_yaw, detail

    def _park_align_rect_control(self):
        """
        PARK_ALIGN_RECT: Align left/right using yellow rectangle center offset.

        Rectangle offset mapping (side camera):
        - offset_x > 0: rectangle center to the right = car too far left = move right (+vy)
        - offset_x < 0: rectangle center to the left = car too far right = move left (-vy)

        Control: rect offset -> vy (lateral movement)
        """
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0

        if self._park_align_rect_done_sent:
            return cmd, error_x, error_y, error_yaw, 'Rect align done, waiting'

        if self._slot_rect is None or not self._slot_rect.valid:
            detail = 'No rectangle detected, waiting...'
            return cmd, error_x, error_y, error_yaw, detail

        rect = self._slot_rect
        offset = rect.center_offset_x  # Already in meters
        error_y = offset

        # Check if aligned
        if abs(offset) < self.park_rect_threshold:
            self._publish_park_align_rect_done()
            self._park_align_rect_done_sent = True
            detail = f'Rect aligned, offset={offset*100:.1f}cm'
            return cmd, error_x, error_y, error_yaw, detail

        # Control: offset -> vy
        # For side camera: positive offset means rect center is right of image center
        # This means car needs to move right (positive vy in our convention)
        vy = self.park_rect_kp * offset
        vy = self._clamp(vy, -self.park_max_vy, self.park_max_vy)

        # Minimum speed
        if abs(vy) < self.park_min_speed and abs(offset) > self.park_rect_threshold * 0.5:
            vy = self.park_min_speed if offset > 0 else -self.park_min_speed

        cmd.linear.y = vy

        detail = f'Align rect: offset={offset*100:.1f}cm, vy={vy:.3f}'

        return cmd, error_x, error_y, error_yaw, detail

    def _park_final_control(self):
        """
        PARK_FINAL: Final distance adjustment using side marker distance.

        Control: distance error -> vy (move closer/farther from marker)
        Target: park_target_distance from marker
        """
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0

        if self._park_final_done_sent:
            return cmd, error_x, error_y, error_yaw, 'Final done, waiting'

        now = self.get_clock().now().nanoseconds / 1e9

        # Phase: FORWARD (거리 조정 완료 후 고정 전진) 차량 중심이랑 카메라 중심이랑 오차 맞춰주기.
        if getattr(self, '_park_final_phase', 'DISTANCE') == 'FORWARD':
            fwd_time = self.park_align_forward / self.park_min_speed
            elapsed = now - self._park_forward_start_time

            if elapsed >= fwd_time:
                self._publish_park_final_done()
                self._park_final_done_sent = True
                detail = f'[FORWARD] done! {self.park_align_forward*100:.1f}cm in {elapsed:.1f}s'
                return cmd, error_x, error_y, error_yaw, detail

            cmd.linear.x = self.park_min_speed
            detail = f'[FORWARD] {elapsed:.2f}/{fwd_time:.1f}s'
            return cmd, error_x, error_y, error_yaw, detail

        if self._side_marker is None:
            # 마커 소실 시: 마지막 거리가 목표 근처였으면 완료 처리
            last_dist = getattr(self, '_park_final_last_distance', -1.0)
            lost_time = getattr(self, '_park_final_lost_start', 0.0)
            if lost_time == 0.0:
                self._park_final_lost_start = now
            lost_elapsed = now - self._park_final_lost_start

            if last_dist > 0:
                remaining = last_dist - self.park_target_distance
                # 마커 근처(오차 5cm 이내)에서 소실 → 2초 후 완료 처리
                if remaining < 0.05 and lost_elapsed > 2.0:
                    self.get_logger().info(
                        f'Side marker lost near target (last={last_dist*100:.1f}cm, '
                        f'target={self.park_target_distance*100:.1f}cm) → completing'
                    )
                    if self.park_align_forward > 0.001:
                        self._park_final_phase = 'FORWARD'
                        self._park_forward_start_time = now
                    else:
                        self._publish_park_final_done()
                        self._park_final_done_sent = True
                    detail = f'Marker lost, completing (last={last_dist*100:.1f}cm)'
                    return cmd, error_x, error_y, error_yaw, detail

            detail = f'No side marker, waiting... (last={last_dist*100:.1f}cm, lost={lost_elapsed:.1f}s)'
            return cmd, error_x, error_y, error_yaw, detail

        # 마커 보이면 lost 타이머 리셋
        self._park_final_lost_start = 0.0
        marker = self._side_marker
        distance = marker.distance
        self._park_final_last_distance = distance
        error = distance - self.park_target_distance
        error_x = distance

        # Check if done (with stability check)
        if abs(error) < self.park_distance_threshold:
            if not hasattr(self, '_park_final_stable_start'):
                self._park_final_stable_start = now
            elif now - self._park_final_stable_start > 0.5:  # 0.5초 안정 유지
                if hasattr(self, '_park_final_stable_start'):
                    delattr(self, '_park_final_stable_start')
                # 거리 조정 완료 → FORWARD 전진
                if self.park_align_forward > 0.001:
                    self._park_final_phase = 'FORWARD'
                    self._park_forward_start_time = now
                    fwd_time = self.park_align_forward / self.park_min_speed
                    self.get_logger().info(
                        f'PARK_FINAL distance done, forward {self.park_align_forward*100:.1f}cm (~{fwd_time:.1f}s)'
                    )
                else:
                    self._publish_park_final_done()
                    self._park_final_done_sent = True
                detail = f'Final complete, dist={distance*100:.1f}cm'
                return cmd, error_x, error_y, error_yaw, detail
            detail = f'Final stable check: d={distance*100:.1f}cm, {now - self._park_final_stable_start:.1f}/0.5s'
            return cmd, error_x, error_y, error_yaw, detail
        else:
            if hasattr(self, '_park_final_stable_start'):
                delattr(self, '_park_final_stable_start')

        # Move-pause-observe pattern
        if self._park_action_start_time == 0.0:
            self._park_action = 'MOVE'
            self._park_action_start_time = now

        action_elapsed = now - self._park_action_start_time

        if self._park_action == 'SETTLE':
            if action_elapsed >= self.align_settle_duration:
                self._park_action = 'MOVE'
                self._park_action_start_time = now
            detail = f'Final SETTLE {action_elapsed:.2f}s, d={distance*100:.1f}cm'
            return cmd, error_x, error_y, error_yaw, detail

        if action_elapsed >= self.align_move_duration:
            self._park_action = 'SETTLE'
            self._park_action_start_time = now
            detail = f'Final → SETTLE, d={distance*100:.1f}cm'
            return cmd, error_x, error_y, error_yaw, detail

        # Stall check (distance 변화 추적)
        self._check_park_stall(distance, 0.001)  # 1mm

        # Control: distance error -> vy (좌측 카메라, 마커가 왼쪽에 있음)
        # error > 0 = too far from marker = need to move left (toward marker, -vy)
        # error < 0 = too close to marker = need to move right (away, +vy)
        vy = -self.park_final_kp * error
        vy = self._clamp(vy, -self.park_max_vy, self.park_max_vy)

        # Minimum speed
        if abs(vy) < self.park_min_speed and abs(error) > self.park_distance_threshold * 0.5:
            vy = -self.park_min_speed if error > 0 else self.park_min_speed

        vy = self._apply_park_stall_boost(vy)
        cmd.linear.y = vy

        detail = f'Final MOVE: d={distance*100:.1f}cm, target={self.park_target_distance*100:.1f}cm, vy={vy:.3f}, boost={self._park_stall_boost:.3f}'

        return cmd, error_x, error_y, error_yaw, detail

    def _publish_park_align_marker_done(self):
        """Notify parking marker alignment complete."""
        msg = Bool()
        msg.data = True
        self.pub_park_align_marker_done.publish(msg)
        self.get_logger().info('Parking marker alignment complete')

    def _publish_park_align_rect_done(self):
        """Notify parking rectangle alignment complete."""
        msg = Bool()
        msg.data = True
        self.pub_park_align_rect_done.publish(msg)
        self.get_logger().info('Parking rectangle alignment complete')

    def _publish_park_final_done(self):
        """Notify parking final positioning complete."""
        msg = Bool()
        msg.data = True
        self.pub_park_final_done.publish(msg)
        self.get_logger().info('Parking final positioning complete')

    def _stop(self):
        """Stop the robot."""
        cmd = Twist()
        self._publish_cmd(cmd)
        self.lane_vy_pid.reset()
        self.lane_wz_pid.reset()

    def _publish_cmd(self, cmd: Twist):
        """Publish velocity command."""
        self.pub_cmd_vel.publish(cmd)

    def _publish_state(self, state: str, ex: float, ey: float, eyaw: float, detail: str):
        """Publish driving state."""
        if not self._has_interface:
            return

        msg = self._DrivingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        msg.target_marker_id = self._target_marker_id
        msg.observed_marker_id = (
            self._tracked_marker.id if self._tracked_marker else -1
        )
        msg.error_x = ex
        msg.error_y = ey
        msg.error_yaw = eyaw
        msg.detail = detail

        self.pub_driving_state.publish(msg)

    def _publish_marker_reached(self, marker_id: int):
        """Notify marker reached."""
        msg = Int32()
        msg.data = marker_id
        self.pub_marker_reached.publish(msg)
        self.get_logger().info(f'Marker {marker_id} reached')

    def _publish_turn_done(self):
        """Notify turn complete."""
        msg = Bool()
        msg.data = True
        self.pub_turn_done.publish(msg)
        self.get_logger().info('Turn complete')

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))



def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
