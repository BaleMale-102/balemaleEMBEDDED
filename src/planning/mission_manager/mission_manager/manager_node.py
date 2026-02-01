#!/usr/bin/env python3
"""
manager_node.py - Mission Manager Node

Orchestrates mission execution using FSM.

Subscribes:
    /server/task_cmd: MissionCommand
    /mission/marker_reached: Int32
    /mission/turn_done: Bool
    /mission/align_done: Bool

Publishes:
    /mission/state: String
    /mission/target_marker: Int32
    /mission/turn_target_rad: Int32 (centidegrees)
    /control/enable_drive: Bool
    /server/task_status: MissionStatus
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool

from .state_machine import StateMachine, MissionState, MissionContext


class MissionManagerNode(Node):
    """Mission manager with FSM."""

    # Turn direction map for waypoints (marker_from -> marker_to -> angle)
    # This should be loaded from config in production
    TURN_MAP = {
        # Example: from marker 1, turn left to go to marker 2
        (1, 2): -math.pi / 2,  # -90 degrees (left)
        (1, 3): math.pi / 2,   # +90 degrees (right)
        # Add more mappings as needed
    }

    def __init__(self):
        super().__init__('mission_manager_node')

        # Parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('default_turn_angle', 1.57)  # 90 degrees

        update_rate = self.get_parameter('update_rate').value
        self.default_turn_angle = self.get_parameter('default_turn_angle').value

        # State machine
        self.fsm = StateMachine()
        self.fsm.set_state_change_callback(self._on_state_change)

        # Track previous marker for turn direction
        self._previous_marker_id = -1

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
        self._previous_marker_id = self.fsm.context.current_marker_id
        self.fsm.notify_marker_reached(marker_id)

    def _turn_done_callback(self, msg: Bool):
        """Handle turn complete notification."""
        if msg.data:
            self.get_logger().info('Turn complete')
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
        """Setup turn target for the next waypoint."""
        ctx = self.fsm.context
        from_marker = self._previous_marker_id
        to_marker = ctx.current_target_marker

        # Look up turn angle from map
        turn_angle = self.TURN_MAP.get((from_marker, to_marker), self.default_turn_angle)

        self.fsm.set_turn_target(turn_angle)

        # Publish turn target (as centidegrees for Int32)
        turn_msg = Int32()
        turn_msg.data = int(turn_angle * 100)  # radians * 100
        self.pub_turn_target.publish(turn_msg)

        self.get_logger().info(f'Turn setup: {math.degrees(turn_angle):.1f} degrees')

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
