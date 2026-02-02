#!/usr/bin/env python3
"""
controller_node.py - Motion Controller Node

Controls robot motion based on perception inputs:
- Lane following mode: Follow detected lanes
- Marker approach mode: Approach target marker
- Turn mode: In-place rotation

Subscribes:
    /perception/lane_status: LaneStatus
    /perception/tracked_marker: TrackedMarker
    /mission/state: String (mission state)
    /mission/target_marker: Int32

Publishes:
    /control/cmd_vel: Twist (velocity commands)
    /driving/state: DrivingState (diagnostics)
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
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MotionControllerNode(Node):
    """Motion controller with multiple control modes."""

    # Control modes
    MODE_IDLE = 'IDLE'
    MODE_DRIVE = 'DRIVE'
    MODE_STOP_AT_MARKER = 'STOP_AT_MARKER'
    MODE_ADVANCE_TO_CENTER = 'ADVANCE_TO_CENTER'
    MODE_TURN = 'TURN'
    MODE_STOP = 'STOP'

    def __init__(self):
        super().__init__('motion_controller_node')

        # Parameters - general
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_vx', 0.008)
        self.declare_parameter('max_vy', 0.008)
        self.declare_parameter('max_wz', 0.05)

        # Parameters - fallback (센서 없을 때)
        self.declare_parameter('reverse_vx', 0.015)  # 후진 속도 (저속)

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
        self.declare_parameter('marker_reach_distance', 0.25)  # 마커 사라지기 전 도달 판정
        self.declare_parameter('marker_min_distance', 0.08)
        self.declare_parameter('marker_lost_reach_distance', 0.35)  # 이 거리 이내에서 사라지면 도달로 판정
        self.declare_parameter('advance_vx', 0.02)  # ADVANCE_TO_CENTER 전진 속도

        # Parameters - turn (PID + 오버슈팅 보정)
        self.declare_parameter('turn_wz', 0.04)
        self.declare_parameter('turn_wz_min', 0.015)     # 최소 회전 속도 (너무 느리면 안 움직임)
        self.declare_parameter('turn_tolerance', 0.05)  # rad (~3도)
        self.declare_parameter('turn_kp', 1.5)          # 비례 게인
        self.declare_parameter('turn_kd', 0.3)          # 미분 게인 (급정지 방지)
        self.declare_parameter('turn_decel_zone', 0.5)  # 감속 시작 구간 (rad, ~30도)

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
        self.advance_vx = self.get_parameter('advance_vx').value
        self.turn_wz = self.get_parameter('turn_wz').value
        self.turn_wz_min = self.get_parameter('turn_wz_min').value
        self.turn_tolerance = self.get_parameter('turn_tolerance').value
        self.turn_kp = self.get_parameter('turn_kp').value
        self.turn_kd = self.get_parameter('turn_kd').value
        self.turn_decel_zone = self.get_parameter('turn_decel_zone').value

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
        self._turn_target_rad = 0.0
        self._turn_start_yaw = 0.0  # TURN 시작 시 yaw
        self._current_yaw = 0.0
        self._prev_turn_error = 0.0  # 이전 턴 에러 (미분용)
        self._turn_done_sent = False  # turn_done 중복 발행 방지

        # Perception data
        self._tracked_marker = None
        self._imu_data = None

        # 마커 추적 상태 (사라짐 감지용)
        self._last_marker_distance = float('inf')
        self._last_marker_id = -1

        # ADVANCE_TO_CENTER 상태용
        self._advance_target_distance = 0.0  # 전진할 거리
        self._advance_start_time = 0.0       # 전진 시작 시간
        self._advance_done_sent = False      # 완료 발행 여부

        # 로그 출력 주기 (1초마다)
        self._last_log_time = 0.0
        self._log_interval = 1.0

        # Import custom interfaces
        try:
            from robot_interfaces.msg import TrackedMarker, DrivingState
            self._has_interface = True
            self._TrackedMarker = TrackedMarker
            self._DrivingState = DrivingState
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

        # Subscribers
        self.sub_mission_state = self.create_subscription(
            String, '/mission/state', self._mission_state_callback, 10
        )
        self.sub_target_marker = self.create_subscription(
            Int32, '/mission/target_marker', self._target_marker_callback, 10
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

        # IMU subscription (for TURN mode)
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data',
            self._imu_callback, qos_profile_sensor_data
        )

        # Control timer
        self.timer = self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info('MotionControllerNode started')

    def _mission_state_callback(self, msg: String):
        """Handle mission state changes."""
        state = msg.data.upper()
        prev_mode = self._mode

        if state == 'IDLE':
            self._mode = self.MODE_IDLE
        elif state in ('DRIVE', 'LANE_FOLLOW', 'MARKER_APPROACH', 'APPROACH'):
            self._mode = self.MODE_DRIVE
        elif state == 'STOP_AT_MARKER':
            self._mode = self.MODE_STOP_AT_MARKER
            # 현재 마커 거리 저장
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
            # TURN 모드 진입 시 상태 초기화
            if prev_mode != self.MODE_TURN:
                self._turn_start_yaw = self._current_yaw
                self._prev_turn_error = 0.0
                self._turn_done_sent = False
                self.get_logger().info(f'TURN started: start_yaw={math.degrees(self._turn_start_yaw):.1f}deg, target={math.degrees(self._turn_target_rad):.1f}deg')
        elif state in ('STOP', 'PARK', 'FINISH', 'ALIGN_TO_MARKER', 'STOP_BUMP'):
            self._mode = self.MODE_STOP
        else:
            self._mode = self.MODE_IDLE

        self.get_logger().debug(f'Mode changed: {self._mode}')

    def _target_marker_callback(self, msg: Int32):
        """Handle target marker changes."""
        self._target_marker_id = msg.data

    def _enable_callback(self, msg: Bool):
        """Handle enable/disable commands."""
        self._enable = msg.data
        if not self._enable:
            self._stop()

    def _turn_target_callback(self, msg):
        """Handle turn target angle (using Int32 as degrees * 100)."""
        self._turn_target_rad = msg.data / 100.0  # Convert from centidegrees

    def _marker_callback(self, msg):
        """Handle tracked marker updates."""
        self._tracked_marker = msg

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
            # 마커 기반 주행
            cmd, error_x, error_y, error_yaw, detail = self._drive_with_marker_only()

        elif self._mode == self.MODE_STOP_AT_MARKER:
            # 정지 상태 - 마커 거리 측정 중
            detail = f'Stopped at marker, dist={self._advance_target_distance:.3f}m'

        elif self._mode == self.MODE_ADVANCE_TO_CENTER:
            cmd, error_x, error_y, error_yaw, detail = self._advance_to_center()

        elif self._mode == self.MODE_TURN:
            cmd, error_x, error_y, error_yaw, detail = self._turn_control()

        elif self._mode == self.MODE_STOP:
            detail = 'Stopped'

        # Apply velocity limits
        cmd.linear.x = self._clamp(cmd.linear.x, -self.max_vx, self.max_vx)
        cmd.linear.y = self._clamp(cmd.linear.y, -self.max_vy, self.max_vy)
        cmd.angular.z = self._clamp(cmd.angular.z, -self.max_wz, self.max_wz)

        self._publish_cmd(cmd)
        self._publish_state(self._mode, error_x, error_y, error_yaw, detail)

        # 주기적 로그 출력
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log_time >= self._log_interval:
            self._last_log_time = now
            self.get_logger().info(
                f'[{self._mode}] target={self._target_marker_id}, '
                f'vx={cmd.linear.x:.4f}, vy={cmd.linear.y:.4f}, wz={cmd.angular.z:.4f} | {detail}'
            )

    def _drive_with_marker_only(self):
        """마커 전용 주행 (lane 사용 안 함)."""
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0

        # 마커 확인 (target ID와 일치하는 마커, ID 16 이상은 주차 슬롯이므로 주행 시 무시)
        marker_ok = (self._tracked_marker is not None and
                     self._tracked_marker.id == self._target_marker_id and
                     self._tracked_marker.id < 16 and  # 주차 슬롯 마커 무시
                     (self._tracked_marker.is_detected or
                      self._tracked_marker.prediction_confidence > 0.3))

        # 마커가 있으면 마커 기반 주행
        if marker_ok:
            marker = self._tracked_marker
            distance = marker.distance
            angle = marker.angle

            # 마지막 마커 정보 저장 (사라짐 감지용)
            self._last_marker_distance = distance
            self._last_marker_id = marker.id

            error_x = distance
            error_yaw = angle

            # 마커 도달 체크
            if distance < self.marker_reach_dist:
                self._publish_marker_reached(marker.id)
                # 거리는 리셋하지 않음 - STOP_AT_MARKER에서 사용
                return cmd, error_x, error_y, error_yaw, f'Reached marker {marker.id}'

            # 마커 방향으로 주행
            vx = self.marker_vx_kp * (distance - self.marker_min_dist)
            vx = max(0.0, vx)
            vy = self.marker_vy_kp * angle  # 횡방향 보정 ON
            wz = 0.0  # 회전 OFF

            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz

            pred = 'pred' if not marker.is_detected else 'det'
            detail = f'[MARKER] d={distance:.2f}m, a={math.degrees(angle):.1f}deg [{pred}]'
            return cmd, error_x, error_y, error_yaw, detail

        # 마커 없음 - 가까이 왔다가 사라지면 도달로 판정
        if (self._last_marker_id == self._target_marker_id and
            self._last_marker_distance < self.marker_lost_reach_dist):
            self._publish_marker_reached(self._last_marker_id)
            self.get_logger().info(
                f'Marker {self._last_marker_id} lost at {self._last_marker_distance:.2f}m - counted as reached'
            )
            # 거리는 리셋하지 않음 - STOP_AT_MARKER에서 사용
            return cmd, error_x, error_y, error_yaw, f'Marker lost but reached'

        # 마커 없음 → 저속 직진 (마커 찾을 때까지)
        cmd.linear.x = self.lane_vx  # 저속 전진
        return cmd, error_x, error_y, error_yaw, 'No marker - slow forward'

    def _drive_with_marker_priority(self):
        """마커 우선 주행 (lane 없이 마커만 사용)."""
        return self._drive_with_marker_only()

    def _lane_follow(self):
        """Legacy lane following (사용 안 함)."""
        return self._drive_with_marker_only()

    def _marker_approach(self):
        """Marker approach control (마커 전용)."""
        # 마커 전용 주행으로 통합
        return self._drive_with_marker_only()

    def _advance_to_center(self):
        """마커 중심까지 전진 (시간 기반)."""
        cmd = Twist()
        error_x = self._advance_target_distance
        error_y = 0.0
        error_yaw = 0.0

        # 전진 속도 (파라미터 사용)
        advance_vx = self.advance_vx

        # 거리 / 속도 = 시간
        if advance_vx > 0:
            required_time = self._advance_target_distance / advance_vx
        else:
            required_time = 3.0  # fallback

        # 경과 시간
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._advance_start_time

        if elapsed >= required_time:
            # 전진 완료
            if not self._advance_done_sent:
                self._publish_align_done()
                self._advance_done_sent = True
            detail = f'Advance complete, elapsed={elapsed:.2f}s'
            return cmd, 0.0, error_y, error_yaw, detail

        # 전진 중
        cmd.linear.x = advance_vx
        remaining = required_time - elapsed
        detail = f'Advancing: {elapsed:.2f}/{required_time:.2f}s, remaining={remaining:.2f}s'

        return cmd, error_x, error_y, error_yaw, detail

    def _publish_align_done(self):
        """Notify alignment/advance complete."""
        msg = Bool()
        msg.data = True
        # /mission/align_done 토픽 사용
        if not hasattr(self, 'pub_align_done'):
            self.pub_align_done = self.create_publisher(Bool, '/mission/align_done', 10)
        self.pub_align_done.publish(msg)
        self.get_logger().info('Advance to center complete')

    def _turn_control(self):
        """In-place turn control (상대 회전 + PD 제어 + 오버슈팅 보정)."""
        cmd = Twist()

        # IMU 데이터 없으면 정지
        if self._imu_data is None:
            return cmd, 0.0, 0.0, 0.0, 'No IMU data'

        # 상대 회전량 계산: 현재 yaw - 시작 yaw
        turned = self._current_yaw - self._turn_start_yaw

        # Normalize to [-pi, pi]
        while turned > math.pi:
            turned -= 2 * math.pi
        while turned < -math.pi:
            turned += 2 * math.pi

        # 목표까지 남은 회전량 (에러)
        error_yaw = self._turn_target_rad - turned

        # Normalize error to [-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi

        # 완료 체크 (중복 발행 방지)
        if abs(error_yaw) < self.turn_tolerance:
            if not self._turn_done_sent:
                self._publish_turn_done()
                self._turn_done_sent = True
            detail = f'Turn complete, turned={math.degrees(turned):.1f}deg'
            return cmd, 0.0, 0.0, error_yaw, detail

        # ===== PD 제어 + 오버슈팅 방지 =====

        # 미분항: 에러 변화율 (급격한 변화 감쇠)
        d_error = error_yaw - self._prev_turn_error
        self._prev_turn_error = error_yaw

        # 비례항
        p_term = self.turn_kp * error_yaw

        # 미분항 (에러가 줄어드는 방향이면 감속)
        d_term = self.turn_kd * d_error

        # PD 출력
        wz = p_term + d_term

        # 감속 구간: 목표 근처에서 속도 제한
        if abs(error_yaw) < self.turn_decel_zone:
            # 선형 감속: 에러가 작을수록 최대 속도 감소
            scale = abs(error_yaw) / self.turn_decel_zone
            max_wz_now = self.turn_wz_min + (self.turn_wz - self.turn_wz_min) * scale
        else:
            max_wz_now = self.turn_wz

        # 속도 제한 (최소/최대)
        if abs(wz) > max_wz_now:
            wz = max_wz_now if wz > 0 else -max_wz_now
        elif abs(wz) < self.turn_wz_min and abs(error_yaw) > self.turn_tolerance:
            # 너무 느리면 최소 속도 유지
            wz = self.turn_wz_min if error_yaw > 0 else -self.turn_wz_min

        cmd.angular.z = wz

        detail = f'Turn: tgt={math.degrees(self._turn_target_rad):.0f}, turned={math.degrees(turned):.1f}, err={math.degrees(error_yaw):.1f}, wz={wz:.2f}'

        return cmd, 0.0, 0.0, error_yaw, detail

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
