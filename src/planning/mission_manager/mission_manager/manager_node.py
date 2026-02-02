#!/usr/bin/env python3
"""
manager_node.py - Mission Manager Node

Orchestrates mission execution using FSM.
Includes dynamic heading estimation and turn angle calculation.

Subscribes:
    /server/task_cmd: MissionCommand
    /mission/marker_reached: Int32
    /mission/turn_done: Bool
    /mission/align_done: Bool

Publishes:
    /mission/state: String
    /mission/target_marker: Int32
    /mission/turn_target_rad: Int32 (centiradians)
    /control/enable_drive: Bool
    /server/task_status: MissionStatus
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool, Float32

from .state_machine import StateMachine, MissionState, MissionContext


def wrap_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class MissionManagerNode(Node):
    """Mission manager with FSM and dynamic heading-based turn calculation."""

    def __init__(self):
        super().__init__('mission_manager_node')

        # Parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('default_turn_angle', 1.57)  # 90 degrees
        self.declare_parameter('marker_map_yaml', '')
        self.declare_parameter('default_marker_yaw', 1.5708)  # π/2
        self.declare_parameter('unit_scale', 0.01)  # cm to m

        update_rate = self.get_parameter('update_rate').value
        self.default_turn_angle = self.get_parameter('default_turn_angle').value
        self.default_marker_yaw = self.get_parameter('default_marker_yaw').value
        self.unit_scale = self.get_parameter('unit_scale').value

        # Load marker map
        self._marker_positions = {}  # {marker_id: (x, y)}
        self._marker_yaw = {}  # {marker_id: yaw}
        self._load_marker_map()

        # State machine
        self.fsm = StateMachine()
        self.fsm.set_state_change_callback(self._on_state_change)

        # Heading tracking
        self._previous_marker_id = -1
        self._current_heading = math.pi / 2  # 초기값: +Y 방향 (아래쪽)

        # Import custom interfaces
        try:
            from robot_interfaces.msg import MissionCommand, MissionStatus
            self._has_interface = True
            self._MissionCommand = MissionCommand
            self._MissionStatus = MissionStatus
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False

        # Publishers
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_target_marker = self.create_publisher(Int32, '/mission/target_marker', 10)
        self.pub_turn_target = self.create_publisher(Int32, '/mission/turn_target_rad', 10)
        self.pub_enable_drive = self.create_publisher(Bool, '/control/enable_drive', 10)
        self.pub_heading = self.create_publisher(Float32, '/mission/current_heading', 10)

        if self._has_interface:
            self.pub_task_status = self.create_publisher(
                self._MissionStatus, '/server/task_status', 10
            )

        # Subscribers
        if self._has_interface:
            self.sub_task_cmd = self.create_subscription(
                self._MissionCommand, '/server/task_cmd',
                self._task_cmd_callback, 10
            )

        self.sub_marker_reached = self.create_subscription(
            Int32, '/mission/marker_reached', self._marker_reached_callback, 10
        )
        self.sub_turn_done = self.create_subscription(
            Bool, '/mission/turn_done', self._turn_done_callback, 10
        )
        self.sub_align_done = self.create_subscription(
            Bool, '/mission/align_done', self._align_done_callback, 10
        )

        # Test command subscriber (for debugging)
        self.sub_test_cmd = self.create_subscription(
            String, '/mission/test_cmd', self._test_cmd_callback, 10
        )

        # Update timer
        self.timer = self.create_timer(1.0 / update_rate, self._update_callback)

        self.get_logger().info('MissionManagerNode started')

    def _task_cmd_callback(self, msg):
        """Handle mission command from server."""
        if msg.command.upper() == 'START':
            self.get_logger().info(
                f'Starting mission: waypoints={msg.waypoint_ids}, goal={msg.final_goal_id}'
            )
            self.fsm.start_mission(
                waypoint_ids=list(msg.waypoint_ids),
                final_goal_id=msg.final_goal_id,
                task_id=msg.task_id,
                task_type=msg.task_type
            )
        elif msg.command.upper() == 'STOP':
            self.fsm.cancel_mission()
            self._disable_drive()

    def _marker_reached_callback(self, msg: Int32):
        """Handle marker reached notification."""
        marker_id = msg.data
        self.get_logger().info(f'Marker {marker_id} reached')

        # Update heading based on path (previous marker -> current marker)
        prev_id = self.fsm.context.current_marker_id
        if prev_id >= 0 and prev_id in self._marker_positions and marker_id in self._marker_positions:
            self._update_heading_from_path(prev_id, marker_id)

        self._previous_marker_id = prev_id
        self.fsm.notify_marker_reached(marker_id)

    def _turn_done_callback(self, msg: Bool):
        """Handle turn complete notification."""
        if msg.data:
            # Update heading after turn
            turn_angle = self.fsm.context.turn_target_rad
            self._current_heading = wrap_angle(self._current_heading + turn_angle)

            self.get_logger().info(
                f'Turn complete, new heading={math.degrees(self._current_heading):.1f}deg'
            )

            # Publish updated heading
            heading_msg = Float32()
            heading_msg.data = self._current_heading
            self.pub_heading.publish(heading_msg)

            self.fsm.notify_turn_complete()

    def _align_done_callback(self, msg: Bool):
        """Handle alignment complete notification."""
        if msg.data:
            self.get_logger().info('Alignment complete')
            self.fsm.notify_align_complete()

    def _test_cmd_callback(self, msg: String):
        """Handle test commands for debugging."""
        cmd = msg.data.strip().upper()

        if cmd.startswith('START'):
            # Parse: START 1,2,3 -> waypoints [1,2], goal 3
            parts = cmd.split()
            if len(parts) > 1:
                ids = [int(x) for x in parts[1].split(',')]
                if len(ids) >= 1:
                    waypoints = ids[:-1]
                    goal = ids[-1]
                    self.fsm.start_mission(waypoints, goal, 'test', 'TEST')
                    self.get_logger().info(f'Test mission started: {waypoints} -> {goal}')
        elif cmd == 'STOP':
            self.fsm.cancel_mission()
            self._disable_drive()
        elif cmd == 'REACHED':
            # Simulate marker reached
            self.fsm.notify_marker_reached(self.fsm.context.current_target_marker)

    def _update_callback(self):
        """Periodic FSM update."""
        self.fsm.update()

        # Publish current state
        state_msg = String()
        state_msg.data = self.fsm.state.name
        self.pub_state.publish(state_msg)

        # Publish target marker
        target_msg = Int32()
        target_msg.data = self.fsm.context.current_target_marker
        self.pub_target_marker.publish(target_msg)

        # Publish mission status
        self._publish_task_status()

    def _on_state_change(self, old_state: MissionState, new_state: MissionState):
        """Handle state transitions."""
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')

        # Enable/disable drive based on state
        drive_states = {
            MissionState.DRIVE,
            MissionState.ADVANCE_TO_CENTER,
            MissionState.ALIGN_TO_MARKER,
            MissionState.TURNING,
            MissionState.PARK,
        }

        if new_state in drive_states:
            self._enable_drive()
        else:
            self._disable_drive()

        # Handle specific state entries
        if new_state == MissionState.TURNING:
            self._setup_turn()
        elif new_state == MissionState.FINISH:
            self.get_logger().info('Mission complete!')
        elif new_state == MissionState.ERROR:
            self.get_logger().error(f'Mission error: {self.fsm.context.error_message}')

    def _setup_turn(self):
        """Setup turn target for the next waypoint using dynamic heading calculation."""
        ctx = self.fsm.context
        current_marker = ctx.current_marker_id

        # TURNING 상태에서는 다음 타겟을 봐야 함 (advance_waypoint 전이므로)
        next_idx = ctx.current_waypoint_idx + 1
        if next_idx < len(ctx.waypoint_ids):
            target_marker = ctx.waypoint_ids[next_idx]
        else:
            target_marker = ctx.final_goal_id

        # Calculate turn angle dynamically
        turn_angle = self._calculate_turn_angle(current_marker, target_marker)

        self.fsm.set_turn_target(turn_angle)

        # Publish turn target (as centiradians for Int32)
        turn_msg = Int32()
        turn_msg.data = int(turn_angle * 100)  # radians * 100
        self.pub_turn_target.publish(turn_msg)

        self.get_logger().info(
            f'Turn setup: heading={math.degrees(self._current_heading):.1f}deg, '
            f'turn={math.degrees(turn_angle):.1f}deg, '
            f'target_marker={target_marker}'
        )

    def _load_marker_map(self):
        """Load marker positions from YAML file."""
        yaml_path = self.get_parameter('marker_map_yaml').value

        if not yaml_path:
            self.get_logger().warn('No marker_map_yaml specified, using empty map')
            return

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            # Get unit scale from file or parameter
            unit_scale = data.get('unit_scale', self.unit_scale)
            default_yaw = data.get('default_marker_yaw', self.default_marker_yaw)

            # Load road markers
            road_markers = data.get('road_markers', {})
            for marker_id, info in road_markers.items():
                pos = info.get('position', [0, 0])
                yaw = info.get('yaw', default_yaw)
                self._marker_positions[int(marker_id)] = (
                    pos[0] * unit_scale,
                    pos[1] * unit_scale
                )
                self._marker_yaw[int(marker_id)] = yaw

            # Load parking markers
            parking_markers = data.get('parking_markers', {})
            for marker_id, info in parking_markers.items():
                pos = info.get('position', [0, 0])
                yaw = info.get('yaw', default_yaw)
                self._marker_positions[int(marker_id)] = (
                    pos[0] * unit_scale,
                    pos[1] * unit_scale
                )
                self._marker_yaw[int(marker_id)] = yaw

            # Also support simple list format (config/marker_map.yaml)
            markers_list = data.get('markers', [])
            for marker in markers_list:
                marker_id = marker.get('id')
                if marker_id is not None:
                    x = marker.get('x', 0) * unit_scale
                    y = marker.get('y', 0) * unit_scale
                    yaw = marker.get('yaw', default_yaw)
                    self._marker_positions[int(marker_id)] = (x, y)
                    self._marker_yaw[int(marker_id)] = yaw

            self.get_logger().info(
                f'Loaded {len(self._marker_positions)} markers from {yaml_path}'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to load marker map: {e}')

    def _update_heading_from_path(self, from_marker: int, to_marker: int):
        """Update current heading based on travel direction between markers."""
        if from_marker not in self._marker_positions or to_marker not in self._marker_positions:
            return

        from_pos = self._marker_positions[from_marker]
        to_pos = self._marker_positions[to_marker]

        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]

        if abs(dx) > 0.001 or abs(dy) > 0.001:  # Avoid division by zero
            self._current_heading = math.atan2(dy, dx)
            self.get_logger().debug(
                f'Heading updated: {math.degrees(self._current_heading):.1f}deg '
                f'(from marker {from_marker} to {to_marker})'
            )

            # Publish current heading
            heading_msg = Float32()
            heading_msg.data = self._current_heading
            self.pub_heading.publish(heading_msg)

    def _calculate_turn_angle(self, current_marker: int, target_marker: int) -> float:
        """Calculate turn angle to face target marker from current position and heading."""
        if current_marker not in self._marker_positions:
            self.get_logger().warn(f'Current marker {current_marker} not in map')
            return self.default_turn_angle

        if target_marker not in self._marker_positions:
            self.get_logger().warn(f'Target marker {target_marker} not in map')
            return self.default_turn_angle

        current_pos = self._marker_positions[current_marker]
        target_pos = self._marker_positions[target_marker]

        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]

        if abs(dx) < 0.001 and abs(dy) < 0.001:
            self.get_logger().warn('Target marker at same position as current')
            return 0.0

        # Target direction (absolute angle in map frame)
        target_direction = math.atan2(dy, dx)

        # Turn angle = target direction - current heading
        turn_angle = wrap_angle(target_direction - self._current_heading)

        self.get_logger().debug(
            f'Turn calculation: current_heading={math.degrees(self._current_heading):.1f}deg, '
            f'target_direction={math.degrees(target_direction):.1f}deg, '
            f'turn_angle={math.degrees(turn_angle):.1f}deg'
        )

        return turn_angle

    def _enable_drive(self):
        """Enable motor control."""
        msg = Bool()
        msg.data = True
        self.pub_enable_drive.publish(msg)

    def _disable_drive(self):
        """Disable motor control."""
        msg = Bool()
        msg.data = False
        self.pub_enable_drive.publish(msg)

    def _publish_task_status(self):
        """Publish task status to server."""
        if not self._has_interface:
            return

        ctx = self.fsm.context
        state = self.fsm.state

        msg = self._MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = ctx.task_id

        # Map FSM state to status
        if state == MissionState.FINISH:
            msg.status = 'COMPLETED'
        elif state == MissionState.ERROR:
            msg.status = 'FAILED'
        elif state == MissionState.IDLE:
            msg.status = 'IDLE'
        else:
            msg.status = 'RUNNING'

        msg.current_state = state.name
        msg.current_waypoint_idx = ctx.current_waypoint_idx
        msg.current_marker_id = ctx.current_marker_id
        msg.progress = self.fsm.get_progress()
        msg.message = ctx.error_message if ctx.error_message else f'At {state.name}'

        self.pub_task_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
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
