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

from .pid_controller import PIDController


class MotionControllerNode(Node):
    """Motion controller with multiple control modes."""

    # Control modes
    MODE_IDLE = 'IDLE'
    MODE_LANE_FOLLOW = 'LANE_FOLLOW'
    MODE_MARKER_APPROACH = 'MARKER_APPROACH'
    MODE_TURN = 'TURN'
    MODE_STOP = 'STOP'

    def __init__(self):
        super().__init__('motion_controller_node')

        # Parameters - general
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_vx', 0.05)
        self.declare_parameter('max_vy', 0.05)
        self.declare_parameter('max_wz', 0.5)

        # Parameters - lane following
        self.declare_parameter('lane_vx', 0.03)
        self.declare_parameter('lane_vy_kp', 0.03)
        self.declare_parameter('lane_vy_ki', 0.0)
        self.declare_parameter('lane_vy_kd', 0.005)
        self.declare_parameter('lane_wz_kp', 0.3)
        self.declare_parameter('lane_wz_ki', 0.0)
        self.declare_parameter('lane_wz_kd', 0.05)

        # Parameters - marker approach
        self.declare_parameter('marker_vx_kp', 0.1)
        self.declare_parameter('marker_vy_kp', 0.05)
        self.declare_parameter('marker_wz_kp', 0.3)
        self.declare_parameter('marker_reach_distance', 0.15)
        self.declare_parameter('marker_min_distance', 0.08)

        # Parameters - turn
        self.declare_parameter('turn_wz', 0.3)
        self.declare_parameter('turn_tolerance', 0.05)  # rad

        # Load parameters
        control_rate = self.get_parameter('control_rate').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value

        self.lane_vx = self.get_parameter('lane_vx').value
        self.marker_reach_dist = self.get_parameter('marker_reach_distance').value
        self.marker_min_dist = self.get_parameter('marker_min_distance').value
        self.turn_wz = self.get_parameter('turn_wz').value
        self.turn_tolerance = self.get_parameter('turn_tolerance').value

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
        self._current_yaw = 0.0

        # Perception data
        self._lane_status = None
        self._tracked_marker = None

        # Import custom interfaces
        try:
            from robot_interfaces.msg import LaneStatus, TrackedMarker, DrivingState
            self._has_interface = True
            self._LaneStatus = LaneStatus
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
            self.sub_lane = self.create_subscription(
                self._LaneStatus, '/perception/lane_status',
                self._lane_callback, qos_profile_sensor_data
            )
            self.sub_marker = self.create_subscription(
                self._TrackedMarker, '/perception/tracked_marker',
                self._marker_callback, qos_profile_sensor_data
            )

        # Control timer
        self.timer = self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info('MotionControllerNode started')

    def _mission_state_callback(self, msg: String):
        """Handle mission state changes."""
        state = msg.data.upper()

        if state == 'IDLE':
            self._mode = self.MODE_IDLE
        elif state in ('DRIVE', 'LANE_FOLLOW'):
            self._mode = self.MODE_LANE_FOLLOW
        elif state in ('MARKER_APPROACH', 'APPROACH'):
            self._mode = self.MODE_MARKER_APPROACH
        elif state in ('TURN', 'TURNING'):
            self._mode = self.MODE_TURN
        elif state in ('STOP', 'PARK', 'FINISH'):
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

    def _lane_callback(self, msg):
        """Handle lane status updates."""
        self._lane_status = msg

    def _marker_callback(self, msg):
        """Handle tracked marker updates."""
        self._tracked_marker = msg

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

        elif self._mode == self.MODE_LANE_FOLLOW:
            cmd, error_x, error_y, error_yaw, detail = self._lane_follow()

        elif self._mode == self.MODE_MARKER_APPROACH:
            cmd, error_x, error_y, error_yaw, detail = self._marker_approach()

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

    def _lane_follow(self):
        """Lane following control."""
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0
        detail = 'No lane'

        if self._lane_status is None or not self._lane_status.valid:
            # No lane detected - slow forward
            cmd.linear.x = self.lane_vx * 0.5
            return cmd, error_x, error_y, error_yaw, 'Lane lost - slow'

        # Lane detected
        offset = self._lane_status.offset_normalized  # -1 to +1
        angle = self._lane_status.angle  # rad

        error_y = offset
        error_yaw = angle

        # PID control
        cmd.linear.x = self.lane_vx
        cmd.linear.y = -self.lane_vy_pid.update(offset)  # Negative to correct
        cmd.angular.z = -self.lane_wz_pid.update(angle)  # Negative to correct

        confidence = self._lane_status.confidence
        detail = f'offset={offset:.2f}, angle={math.degrees(angle):.1f}deg, conf={confidence:.2f}'

        return cmd, error_x, error_y, error_yaw, detail

    def _marker_approach(self):
        """Marker approach control."""
        cmd = Twist()
        error_x = 0.0
        error_y = 0.0
        error_yaw = 0.0
        detail = 'No marker'

        if self._tracked_marker is None:
            return cmd, error_x, error_y, error_yaw, 'No tracked marker'

        marker = self._tracked_marker

        if marker.id != self._target_marker_id:
            return cmd, error_x, error_y, error_yaw, f'Wrong marker (got {marker.id})'

        if not marker.is_detected and marker.prediction_confidence < 0.3:
            return cmd, error_x, error_y, error_yaw, 'Marker lost, low confidence'

        # Get marker position (in camera frame: z=forward, x=right)
        distance = marker.distance
        angle = marker.angle

        error_x = distance
        error_yaw = angle

        # Check if reached
        if distance < self.marker_reach_dist:
            self._publish_marker_reached(marker.id)
            detail = f'Reached marker {marker.id}'
            return cmd, error_x, error_y, error_yaw, detail

        # Proportional control toward marker
        # Forward velocity proportional to distance
        vx = self.marker_vx_kp * (distance - self.marker_min_dist)
        vx = max(0.0, vx)  # Only forward

        # Lateral correction based on angle
        vy = -self.marker_vy_kp * math.sin(angle) * distance

        # Yaw correction
        wz = -self.marker_wz_kp * angle

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz

        pred = 'pred' if not marker.is_detected else 'det'
        detail = f'd={distance:.2f}m, a={math.degrees(angle):.1f}deg [{pred}]'

        return cmd, error_x, error_y, error_yaw, detail

    def _turn_control(self):
        """In-place turn control."""
        cmd = Twist()
        error_yaw = self._turn_target_rad - self._current_yaw

        # Normalize angle to [-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi

        # Check if done
        if abs(error_yaw) < self.turn_tolerance:
            self._publish_turn_done()
            detail = f'Turn complete, error={math.degrees(error_yaw):.1f}deg'
            return cmd, 0.0, 0.0, error_yaw, detail

        # Proportional turn
        wz = self.turn_wz if error_yaw > 0 else -self.turn_wz

        # Reduce speed near target
        if abs(error_yaw) < 0.3:  # ~17 degrees
            wz *= 0.5

        cmd.angular.z = wz

        detail = f'Turning: target={math.degrees(self._turn_target_rad):.1f}, error={math.degrees(error_yaw):.1f}'

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
