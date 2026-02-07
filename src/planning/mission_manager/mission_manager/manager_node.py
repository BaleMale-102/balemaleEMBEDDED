#!/usr/bin/env python3
"""
manager_node.py - Mission Manager Node

Orchestrates full mission execution using FSM.
Handles: vehicle waiting, ANPR recognition, loading, navigation, parking, unloading, return.

Subscribes:
    /server/task_cmd: MissionCommand
    /mission/marker_reached: Int32
    /mission/turn_done: Bool
    /mission/align_done: Bool
    /perception/side_markers: MarkerArray (for parking)
    /perception/anpr/detections: DetectionArray (for plate recognition)
    /plate/response: PlateResponse (from server)
    /loader/status: LoaderStatus

Publishes:
    /mission/state: String
    /mission/target_marker: Int32
    /mission/turn_target_rad: Int32 (centiradians)
    /control/enable_drive: Bool
    /server/task_status: MissionStatus
    /parking/target_slot: Int32
    /parking/status: ParkingStatus
    /plate/query: PlateQuery
    /loader/command: LoaderCommand
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Int32, Bool, Float32

from .state_machine import (
    StateMachine, MissionState, HOME_MARKER_ID,
    get_slot_zone, is_same_zone, get_slot_direction
)


def wrap_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class MissionManagerNode(Node):
    """Mission manager with FSM for full autonomous parking robot operation."""

    def __init__(self):
        super().__init__('mission_manager_node')

        # Parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('default_turn_angle', 1.57)
        self.declare_parameter('marker_map_yaml', '')
        self.declare_parameter('default_marker_yaw', 1.5708)
        self.declare_parameter('unit_scale', 0.01)
        self.declare_parameter('car_id', 'car_01')
        self.declare_parameter('auto_start_waiting', False)  # Auto-start in WAIT_VEHICLE mode
        # Approach/retreat parameters (hardcoded movement)
        self.declare_parameter('approach_distance', 0.30)   # Distance to vehicle (m)
        self.declare_parameter('approach_speed', 0.01)       # Movement speed (m/s)
        self.declare_parameter('retreat_from_slot_distance', 0.25)  # 주차 슬롯→길 복귀 거리 (m)
        # Anomaly detection parameters
        self.declare_parameter('anomaly_stop_distance', 30.0)   # Stop if obstacle closer than this (cm)
        self.declare_parameter('anomaly_clear_timeout', 3.0)    # Resume after N seconds without detection

        # State timeout parameters
        self.declare_parameter('timeout_stop_at_marker', 2.0)
        self.declare_parameter('timeout_advance_to_center', 0.5)
        self.declare_parameter('timeout_align_to_marker', 5.0)
        self.declare_parameter('timeout_stop_bump', 0.5)
        self.declare_parameter('timeout_turning', 30.0)
        self.declare_parameter('timeout_park', 30.0)

        # Parking timeout parameters
        self.declare_parameter('timeout_park_detect', 10.0)
        self.declare_parameter('timeout_park_recovery', 15.0)
        self.declare_parameter('timeout_park_align_marker', 15.0)
        self.declare_parameter('timeout_park_align_rect', 10.0)
        self.declare_parameter('timeout_park_final', 10.0)

        # Full mission timeout parameters
        self.declare_parameter('timeout_recognize', 30.0)
        self.declare_parameter('timeout_load', 30.0)
        self.declare_parameter('timeout_unload', 30.0)

        # State delay parameters
        self.declare_parameter('delay_stop_at_marker', 0.5)
        self.declare_parameter('delay_stop_bump', 0.3)

        update_rate = self.get_parameter('update_rate').value
        self.default_turn_angle = self.get_parameter('default_turn_angle').value
        self.default_marker_yaw = self.get_parameter('default_marker_yaw').value
        self.unit_scale = self.get_parameter('unit_scale').value
        self.car_id = self.get_parameter('car_id').value
        auto_start_waiting = self.get_parameter('auto_start_waiting').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.retreat_from_slot_distance = self.get_parameter('retreat_from_slot_distance').value
        self.retreat_from_slot_speed = self.get_parameter('retreat_from_slot_speed').value
        self.anomaly_stop_distance = self.get_parameter('anomaly_stop_distance').value
        self.anomaly_clear_timeout = self.get_parameter('anomaly_clear_timeout').value

        # Build timeout/delay configs for state machine
        # 테스트용: 주행/주차 관련 타임아웃 비활성화 (0 = 무제한)
        timeouts = {
            'STOP_AT_MARKER': self.get_parameter('timeout_stop_at_marker').value,
            'ADVANCE_TO_CENTER': self.get_parameter('timeout_advance_to_center').value,
            'ALIGN_TO_MARKER': self.get_parameter('timeout_align_to_marker').value,
            'STOP_BUMP': self.get_parameter('timeout_stop_bump').value,
            'TURNING': self.get_parameter('timeout_turning').value,
            'PARK': 0,  # 비활성화
            'PARK_DETECT': 0,  # 비활성화
            'PARK_RECOVERY': 0,  # 비활성화
            'PARK_ALIGN_MARKER': 0,  # 비활성화
            'PARK_ALIGN_RECT': 0,  # 비활성화
            'PARK_FINAL': 0,  # 비활성화
            'RECOGNIZE': self.get_parameter('timeout_recognize').value,
            'LOAD': self.get_parameter('timeout_load').value,
            'UNLOAD': self.get_parameter('timeout_unload').value,
        }
        delays = {
            'stop_at_marker': self.get_parameter('delay_stop_at_marker').value,
            'stop_bump': self.get_parameter('delay_stop_bump').value,
        }

        # Load marker map
        self._marker_positions = {}
        self._marker_yaw = {}
        self._load_marker_map()

        # State machine
        self.fsm = StateMachine(timeouts=timeouts, delays=delays)
        self.fsm.set_state_change_callback(self._on_state_change)

        # Heading tracking
        self._previous_marker_id = -1
        self._current_heading = math.pi / 2

        # Side camera marker tracking (for parking)
        self._side_marker = None
        self._side_marker_time = 0.0

        # Loader status tracking
        self._loader_status = None
        self._loader_is_loaded = False

        # ANPR detection tracking
        self._last_plate_number = ''
        self._plate_query_pending = False

        # Anomaly detection tracking
        self._anomaly_paused = False
        self._anomaly_type = ''
        self._anomaly_last_detection_time = 0.0

        # Import custom interfaces
        try:
            from robot_interfaces.msg import (
                MissionCommand, MissionStatus,
                MarkerArray, ParkingStatus,
                DetectionArray, PlateQuery, PlateResponse,
                LoaderCommand, LoaderStatus
            )
            self._has_interface = True
            self._MissionCommand = MissionCommand
            self._MissionStatus = MissionStatus
            self._MarkerArray = MarkerArray
            self._ParkingStatus = ParkingStatus
            self._DetectionArray = DetectionArray
            self._PlateQuery = PlateQuery
            self._PlateResponse = PlateResponse
            self._LoaderCommand = LoaderCommand
            self._LoaderStatus = LoaderStatus
        except ImportError:
            self.get_logger().warn('robot_interfaces not found')
            self._has_interface = False

        # Publishers
        self.pub_state = self.create_publisher(String, '/mission/state', 10)
        self.pub_target_marker = self.create_publisher(Int32, '/mission/target_marker', 10)
        self.pub_turn_target = self.create_publisher(Int32, '/mission/turn_target_rad', 10)
        self.pub_enable_drive = self.create_publisher(Bool, '/control/enable_drive', 10)
        self.pub_heading = self.create_publisher(Float32, '/mission/current_heading', 10)
        self.pub_parking_target = self.create_publisher(Int32, '/parking/target_slot', 10)

        if self._has_interface:
            self.pub_task_status = self.create_publisher(
                self._MissionStatus, '/server/task_status', 10
            )
            self.pub_parking_status = self.create_publisher(
                self._ParkingStatus, '/parking/status', 10
            )
            self.pub_plate_query = self.create_publisher(
                self._PlateQuery, '/plate/query', 10
            )
            self.pub_loader_cmd = self.create_publisher(
                self._LoaderCommand, '/loader/command', 10
            )

        # Anomaly report publisher (JSON string for server_bridge)
        self.pub_anomaly_report = self.create_publisher(String, '/server/anomaly_report', 10)

        # Subscribers
        if self._has_interface:
            self.sub_task_cmd = self.create_subscription(
                self._MissionCommand, '/server/task_cmd',
                self._task_cmd_callback, 10
            )
            # Side marker subscription for parking
            self.sub_side_markers = self.create_subscription(
                self._MarkerArray, '/perception/side_markers',
                self._side_markers_callback, qos_profile_sensor_data
            )
            # ANPR detections
            self.sub_anpr_detections = self.create_subscription(
                self._DetectionArray, '/perception/anpr/detections',
                self._anpr_detections_callback, qos_profile_sensor_data
            )
            # Anomaly detections (person/box/cone)
            self.sub_anomaly_detections = self.create_subscription(
                self._DetectionArray, '/perception/anomaly/detections',
                self._anomaly_detections_callback, qos_profile_sensor_data
            )
            # Side anomaly detections (for slot blocking)
            self.sub_side_anomaly_detections = self.create_subscription(
                self._DetectionArray, '/perception/side_anomaly/detections',
                self._side_anomaly_detections_callback, qos_profile_sensor_data
            )
            # Plate response from server
            self.sub_plate_response = self.create_subscription(
                self._PlateResponse, '/plate/response',
                self._plate_response_callback, 10
            )
            # Loader status
            self.sub_loader_status = self.create_subscription(
                self._LoaderStatus, '/loader/status',
                self._loader_status_callback, 10
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

        # Parking event subscribers
        self.sub_park_align_marker_done = self.create_subscription(
            Bool, '/parking/align_marker_done', self._park_align_marker_done_callback, 10
        )
        self.sub_park_align_rect_done = self.create_subscription(
            Bool, '/parking/align_rect_done', self._park_align_rect_done_callback, 10
        )
        self.sub_park_final_done = self.create_subscription(
            Bool, '/parking/final_done', self._park_final_done_callback, 10
        )

        # Test command subscriber
        self.sub_test_cmd = self.create_subscription(
            String, '/mission/test_cmd', self._test_cmd_callback, 10
        )

        # Map update subscriber (from server_bridge)
        self.sub_map_updated = self.create_subscription(
            Bool, '/server/map_updated', self._map_updated_callback, 10
        )

        # Update timer
        self.timer = self.create_timer(1.0 / update_rate, self._update_callback)

        # Auto-start in waiting mode if configured
        if auto_start_waiting:
            self.get_logger().info('Auto-starting in WAIT_VEHICLE mode')
            self.fsm.start_waiting()

        self.get_logger().info('MissionManagerNode started (full mission support)')

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
            self._send_loader_command('STOP')
        elif msg.command.upper() == 'WAIT':
            # Start waiting mode
            self.fsm.start_waiting()

    def _marker_reached_callback(self, msg: Int32):
        """Handle marker reached notification."""
        marker_id = msg.data
        self.get_logger().info(f'Marker {marker_id} reached')

        prev_id = self.fsm.context.current_marker_id
        if prev_id >= 0 and prev_id in self._marker_positions and marker_id in self._marker_positions:
            self._update_heading_from_path(prev_id, marker_id)

        self._previous_marker_id = prev_id
        self.fsm.notify_marker_reached(marker_id)

    def _turn_done_callback(self, msg: Bool):
        """Handle turn complete notification."""
        if msg.data:
            turn_angle = self.fsm.context.turn_target_rad
            old_heading = self._current_heading
            self._current_heading = wrap_angle(self._current_heading + turn_angle)

            self.get_logger().info(
                f'[HEADING] Turn complete: {math.degrees(old_heading):.1f}deg + '
                f'turn({math.degrees(turn_angle):.1f}deg) = {math.degrees(self._current_heading):.1f}deg'
            )

            heading_msg = Float32()
            heading_msg.data = self._current_heading
            self.pub_heading.publish(heading_msg)

            self.fsm.notify_turn_complete()

    def _align_done_callback(self, msg: Bool):
        """Handle alignment complete notification."""
        if msg.data:
            self.get_logger().info('Alignment complete')
            self.fsm.notify_align_complete()

    def _side_markers_callback(self, msg):
        """Handle side camera marker detections for parking."""
        # Parking logic using markers (existing logic)
        if self.fsm.state in (
            MissionState.PARK_DETECT,
            MissionState.PARK_ALIGN_MARKER,
            MissionState.PARK_FINAL
        ):
            if not msg.markers:
                return

            target_slot = self.fsm.context.parking.target_slot_id
            if target_slot < 0:
                return

            best_marker = None
            best_score = -1

            for marker in msg.markers:
                if marker.id < 16 or marker.id > 27:
                    continue

                score = 0
                if marker.id == target_slot:
                    score = 100
                elif is_same_zone(marker.id, target_slot):
                    score = 50
                else:
                    score = 10

                score += max(0, 10 - marker.distance * 10)

                if score > best_score:
                    best_score = score
                    best_marker = marker

            if best_marker:
                self._side_marker = best_marker
                self._side_marker_time = self.get_clock().now().nanoseconds / 1e9

                if self.fsm.state == MissionState.PARK_DETECT:
                    self.fsm.notify_park_detect_done(
                        best_marker.id,
                        best_marker.distance,
                        best_marker.angle
                    )
                    self.get_logger().info(
                        f'Side marker detected: id={best_marker.id}, '
                        f'd={best_marker.distance:.2f}m, a={math.degrees(best_marker.angle):.1f}deg'
                    )

        # Also just track the last seen side marker in general for anomaly association
        if msg.markers:
            # Just take the closest one
            closest = min(msg.markers, key=lambda m: m.distance)
            if 16 <= closest.id <= 27:
                self._side_marker = closest
                self._side_marker_time = self.get_clock().now().nanoseconds / 1e9

    def _anpr_detections_callback(self, msg):
        """Handle ANPR detections for plate recognition."""
        # Only process in WAIT_VEHICLE state
        if self.fsm.state != MissionState.WAIT_VEHICLE:
            return

        # Find plate detection
        for det in msg.detections:
            if det.class_id == 0 and det.text:  # Plate class
                plate = det.text.strip()
                if plate and plate != self._last_plate_number:
                    self._last_plate_number = plate
                    self.get_logger().info(f'Plate detected: {plate}')
                    self.fsm.notify_plate_detected(plate)
                break

    def _plate_response_callback(self, msg):
        """Handle plate verification response from server."""
        if self.fsm.state != MissionState.RECOGNIZE:
            return

        self.get_logger().info(
            f'Plate response: verified={msg.verified}, slot={msg.assigned_slot_id}'
        )

        self.fsm.notify_plate_response(
            verified=msg.verified,
            slot_id=msg.assigned_slot_id,
            waypoints=list(msg.waypoint_ids),
            message=msg.message
        )

        self._plate_query_pending = False

    def _loader_status_callback(self, msg):
        """Handle loader status updates."""
        self._loader_status = msg.status
        self._loader_is_loaded = msg.is_loaded

        # Check for completion in relevant states
        if self.fsm.state == MissionState.LOAD:
            if msg.status == 'DONE' and msg.is_loaded:
                self.get_logger().info('Load complete')
                self.fsm.notify_load_complete()
            elif msg.status == 'ERROR':
                self.get_logger().error(f'Loader error: {msg.message}')
                self.fsm.notify_loader_error()

        elif self.fsm.state == MissionState.UNLOAD:
            if msg.status == 'DONE' and not msg.is_loaded:
                self.get_logger().info('Unload complete')
                self.fsm.notify_unload_complete()
            elif msg.status == 'ERROR':
                self.get_logger().error(f'Loader error: {msg.message}')
                self.fsm.notify_loader_error()

    def _anomaly_detections_callback(self, msg):
        """Handle anomaly detections for emergency stop."""
        # Only process during active movement states
        active_states = {
            MissionState.DRIVE,
            MissionState.ADVANCE_TO_CENTER,
            MissionState.ALIGN_TO_MARKER,
            MissionState.TURNING,
            MissionState.PARK_DETECT,
            MissionState.PARK_RECOVERY,
            MissionState.PARK_ALIGN_MARKER,
            MissionState.PARK_FINAL,
            MissionState.APPROACH_VEHICLE,
            MissionState.RETREAT_FROM_VEHICLE,
            MissionState.RETREAT_FROM_SLOT,
        }
        if self.fsm.state not in active_states:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Find closest obstacle (person=2, box=3, cone=4)
        closest_distance = float('inf')
        closest_class = ''
        for det in msg.detections:
            if det.class_id in (2, 3, 4) and det.distance > 0:
                if det.distance < closest_distance:
                    closest_distance = det.distance
                    closest_class = det.class_name

        if closest_distance < self.anomaly_stop_distance:
            self._anomaly_last_detection_time = now

            if not self._anomaly_paused:
                self._anomaly_paused = True
                self._anomaly_type = 'HUMAN' if closest_class == 'person' else 'OBSTACLE'
                self.get_logger().warn(
                    f'EMERGENCY STOP: {closest_class} at {closest_distance:.1f}cm')
                self._disable_drive()
                self._send_anomaly_report(
                    anomaly_type=self._anomaly_type,
                    distance=closest_distance,
                    event_type='ESTOP'
                )

    # Parking states where side anomaly should trigger emergency stop
    _PARKING_STATES = {
        MissionState.PARK_DETECT,
        MissionState.PARK_RECOVERY,
        MissionState.PARK_ALIGN_MARKER,
        MissionState.PARK_ALIGN_RECT,
        MissionState.PARK_FINAL,
    }

    def _side_anomaly_detections_callback(self, msg):
        """Handle side anomaly detections for slot blocking reporting.

        During PARKING states: stop parking + send ESTOP to server.
        Otherwise: log + send WARNING only.
        """
        if not msg.detections:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Check for any valid side anomaly (person/box/cone)
        closest_dist = float('inf')
        anomaly_detected = False
        for det in msg.detections:
            if det.class_id in (2, 3, 4) and det.distance > 0:
                if det.distance < closest_dist:
                    closest_dist = det.distance
                    anomaly_detected = True

        if not anomaly_detected:
            return

        # Determine marker_id for report
        marker_id = -1
        if self._side_marker is not None and (now - self._side_marker_time) <= 2.0:
            marker_id = self._side_marker.id

        # PARKING states: emergency stop + ESTOP report
        if self.fsm.state in self._PARKING_STATES:
            self._anomaly_last_detection_time = now

            if not self._anomaly_paused:
                self._anomaly_paused = True
                self._anomaly_type = 'SLOT_BLOCKED'
                self.get_logger().warn(
                    f'PARKING ESTOP: obstacle at {closest_dist:.1f}cm during {self.fsm.state.name}')
                self._disable_drive()
                self._send_anomaly_report(
                    anomaly_type='SLOT_BLOCKED',
                    distance=closest_dist,
                    event_type='ESTOP',
                    marker_id=marker_id if marker_id >= 0 else self.fsm.context.parking.target_slot_id
                )
            return

        # Non-parking states: WARNING only (no stop)
        if marker_id >= 0:
            self.get_logger().info(
                f'Slot Blocked: Slot {marker_id} blocked by obstacle at {closest_dist:.1f}cm'
            )
            self._send_anomaly_report(
                anomaly_type='SLOT_BLOCKED',
                distance=closest_dist,
                event_type='WARNING',
                marker_id=marker_id
            )

    def _send_anomaly_report(self, anomaly_type: str, distance: float, 
                             event_type: str = 'ESTOP', marker_id: int = -1):
        """Send anomaly report to server bridge via ROS topic."""
        import json
        
        # Use provided marker_id if valid, else current context marker
        target_marker = marker_id if marker_id >= 0 else self.fsm.context.current_marker_id

        report = json.dumps({
            'anomaly_type': anomaly_type,
            'distance_cm': distance,
            'event_type': event_type,
            'state': self.fsm.state.name,
            'marker_id': target_marker,
            'next_marker': self.fsm.context.current_target_marker,
        })
        report_msg = String()
        report_msg.data = report
        self.pub_anomaly_report.publish(report_msg)
        self.get_logger().info(f'Anomaly report sent: {anomaly_type} (Event: {event_type})')

    def _park_align_marker_done_callback(self, msg: Bool):
        """Handle parking marker alignment complete."""
        if msg.data:
            self.get_logger().info('Parking marker alignment complete')
            self.fsm.notify_park_align_marker_done()

    def _park_align_rect_done_callback(self, msg: Bool):
        """Handle parking rectangle alignment complete."""
        if msg.data:
            self.get_logger().info('Parking rectangle alignment complete')
            self.fsm.notify_park_align_rect_done()

    def _park_final_done_callback(self, msg: Bool):
        """Handle parking final positioning complete."""
        if msg.data:
            self.get_logger().info('Parking final positioning complete')
            self.fsm.notify_park_final_done()

    def _test_cmd_callback(self, msg: String):
        """Handle test commands for debugging."""
        cmd = msg.data.strip().upper()

        if cmd == 'WAIT':
            # Start waiting for vehicle
            self.fsm.start_waiting()
            self.get_logger().info('Started waiting for vehicle')
        elif cmd.startswith('START_FULL'):
            # START_FULL 1,3,9 17 - waypoints then slot
            parts = cmd.split()
            if len(parts) >= 3:
                waypoints = [int(x) for x in parts[1].split(',')]
                slot_id = int(parts[2])
                self.fsm.start_mission(waypoints, slot_id, 'test_full', 'PARK')
                self.fsm.context.is_full_mission = True
                self.get_logger().info(f'Full mission started: {waypoints} -> slot {slot_id} (is_full_mission=True)')
        elif cmd.startswith('START_PARK'):
            parts = cmd.split()
            if len(parts) > 1:
                slot_id = int(parts[1])
                self.fsm.start_mission([], slot_id, 'test_park', 'PARK')
                self.get_logger().info(f'Test parking mission started: slot {slot_id}')
        elif cmd.startswith('START'):
            parts = cmd.split()
            if len(parts) > 1:
                ids = [int(x) for x in parts[1].split(',')]
                if len(ids) >= 1:
                    waypoints = ids[:-1]
                    goal = ids[-1]
                    self.fsm.start_mission(waypoints, goal, 'test', '')
                    self.get_logger().info(f'Test mission started: {waypoints} -> {goal}')
        elif cmd == 'STOP':
            self.fsm.cancel_mission()
            self._disable_drive()
            self._send_loader_command('STOP')
        elif cmd == 'REACHED':
            self.fsm.notify_marker_reached(self.fsm.context.current_target_marker)
        elif cmd == 'LOAD':
            self._send_loader_command('LOAD')
        elif cmd == 'UNLOAD':
            self._send_loader_command('UNLOAD')
        elif cmd.startswith('PLATE'):
            # Simulate plate detection: PLATE ABC123
            parts = cmd.split()
            if len(parts) > 1:
                plate = parts[1]
                self.fsm.notify_plate_detected(plate)
                self.get_logger().info(f'Simulated plate: {plate}')
        elif cmd.startswith('VERIFY'):
            # Simulate server response: VERIFY 17 or VERIFY 17 1,5,6
            parts = cmd.split()
            if len(parts) >= 2:
                slot_id = int(parts[1])
                waypoints = []
                if len(parts) > 2:
                    waypoints = [int(x) for x in parts[2].split(',')]
                self.fsm.notify_plate_response(True, slot_id, waypoints)
                self.get_logger().info(f'Simulated verify: slot={slot_id}, waypoints={waypoints}')
        elif cmd.startswith('EXIT'):
            # 출차: EXIT 1,5 17 (waypoints then slot) or EXIT 17 (direct)
            parts = cmd.split()
            if len(parts) >= 3:
                waypoints = [int(x) for x in parts[1].split(',')]
                slot_id = int(parts[2])
                self.fsm.start_mission(waypoints, slot_id, 'test_exit', 'EXIT')
                self.get_logger().info(f'Exit mission started: {waypoints} -> slot {slot_id}')
            elif len(parts) == 2:
                slot_id = int(parts[1])
                self.fsm.start_mission([], slot_id, 'test_exit', 'EXIT')
                self.get_logger().info(f'Exit mission started: direct to slot {slot_id}')
        elif cmd == 'HOME' or cmd == 'RETURN_HOME':
            # Start return to home (marker 0)
            self.fsm.start_return_home()
            self._enable_drive()
            self.get_logger().info('Returning home (marker 0)')

    def _update_callback(self):
        """Periodic FSM update."""
        import time as _time

        # Handle anomaly pause
        if self._anomaly_paused:
            self._disable_drive()
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._anomaly_last_detection_time > self.anomaly_clear_timeout:
                self._anomaly_paused = False
                self._anomaly_type = ''
                self.get_logger().info('Obstacle cleared, resuming mission')
                # Reset state enter time to prevent timeout from pause duration
                self.fsm.context.state_enter_time = _time.time()
                self._enable_drive()
            else:
                # Still paused - publish current state but skip FSM update
                state_msg = String()
                state_msg.data = self.fsm.state.name
                self.pub_state.publish(state_msg)
                return

        self.fsm.update()

        # Handle state-specific actions
        self._handle_state_actions()

        # Publish current state
        state_msg = String()
        state_msg.data = self.fsm.state.name
        self.pub_state.publish(state_msg)

        # Publish target marker
        # TURNING 중에는 다음 타겟(회전 목적지)을 publish
        target_msg = Int32()
        if self.fsm.state == MissionState.TURNING:
            next_idx = self.fsm.context.current_waypoint_idx + 1
            if next_idx < len(self.fsm.context.waypoint_ids):
                target_msg.data = self.fsm.context.waypoint_ids[next_idx]
            else:
                target_msg.data = self.fsm.context.final_goal_id
        else:
            target_msg.data = self.fsm.context.current_target_marker
        self.pub_target_marker.publish(target_msg)

        # Publish parking target slot
        parking_target_msg = Int32()
        parking_target_msg.data = self.fsm.context.parking.target_slot_id
        self.pub_parking_target.publish(parking_target_msg)

        # Publish mission status
        self._publish_task_status()

        # Publish parking status
        self._publish_parking_status()

    def _handle_state_actions(self):
        """Handle state-specific actions that need periodic execution."""
        state = self.fsm.state

        # RECOGNIZE: Send plate query if not sent yet
        if state == MissionState.RECOGNIZE:
            if not self.fsm.context.recognition.query_sent:
                self._send_plate_query()
                self.fsm.notify_plate_query_sent()

        # LOAD: Send load command on state entry
        # (Handled in _on_state_change)

        # UNLOAD: Send unload command on state entry
        # (Handled in _on_state_change)

    def _on_state_change(self, old_state: MissionState, new_state: MissionState):
        """Handle state transitions."""
        self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')

        # Enable/disable drive based on state
        drive_states = {
            MissionState.DRIVE,
            MissionState.ADVANCE_TO_CENTER,
            MissionState.ALIGN_TO_MARKER,
            MissionState.TURNING,
            MissionState.APPROACH_VEHICLE,
            MissionState.RETREAT_FROM_VEHICLE,
            MissionState.RETREAT_FROM_SLOT,
            MissionState.PARK,
            MissionState.PARK_DETECT,
            MissionState.PARK_RECOVERY,
            MissionState.PARK_ALIGN_MARKER,
            MissionState.PARK_ALIGN_RECT,
            MissionState.PARK_FINAL,
            MissionState.RETURN_HOME,
        }

        if new_state in drive_states:
            self._enable_drive()
        else:
            self._disable_drive()

        # Handle specific state entries
        if new_state == MissionState.TURNING:
            self._setup_turn()
        elif new_state == MissionState.WAIT_VEHICLE:
            self._last_plate_number = ''
            self.get_logger().info('Waiting for vehicle at home position')
        elif new_state == MissionState.RECOGNIZE:
            self.get_logger().info(
                f'Recognizing plate: {self.fsm.context.recognition.plate_number}'
            )
        elif new_state in (MissionState.APPROACH_VEHICLE, MissionState.RETREAT_FROM_VEHICLE):
            duration = self.approach_distance / self.approach_speed if self.approach_speed > 0 else 10.0
            self.fsm.context.approach_duration = duration
            direction = 'toward' if new_state == MissionState.APPROACH_VEHICLE else 'away from'
            self.get_logger().info(
                f'Moving {direction} vehicle: {self.approach_distance:.2f}m '
                f'at {self.approach_speed:.3f}m/s ({duration:.1f}s)'
            )
        elif new_state == MissionState.LOAD:
            if self.fsm.context.task_type == 'EXIT':
                self.get_logger().info('Starting load operation (출차: picking up at slot)')
            else:
                self.get_logger().info('Starting load operation')
            self._send_loader_command('LOAD')
        elif new_state == MissionState.UNLOAD:
            if self.fsm.context.task_type == 'EXIT':
                self.get_logger().info('Starting unload operation (출차: delivering at home)')
            else:
                self.get_logger().info('Starting unload operation')
            self._send_loader_command('UNLOAD')
        elif new_state == MissionState.RETURN_HOME:
            ctx = self.fsm.context
            if ctx.task_type == 'EXIT':
                self.get_logger().info(
                    f'Returning home (출차): waypoints={ctx.waypoint_ids}, '
                    f'current_marker={ctx.current_marker_id}'
                )
            else:
                self.get_logger().info(
                    f'Returning home: waypoints={ctx.waypoint_ids}, '
                    f'current_marker={ctx.current_marker_id}'
                )
        elif new_state == MissionState.RETREAT_FROM_SLOT:
            retreat_vy = self.retreat_from_slot_speed
            if retreat_vy <= 0.0:
                retreat_vy = 0.02
                self.get_logger().warn('retreat_from_slot_speed is 0, using default 0.02')

            duration = self.retreat_from_slot_distance / retreat_vy
            self.fsm.context.approach_duration = duration
            self.get_logger().info(
                f'Retreat from slot: +vy={retreat_vy}, {self.retreat_from_slot_distance:.2f}m, {duration:.1f}s'
            )
        elif new_state == MissionState.PARK_DETECT:
            self.get_logger().info(
                f'Starting parking detection for slot {self.fsm.context.parking.target_slot_id}'
            )
        elif new_state == MissionState.PARK_RECOVERY:
            ctx = self.fsm.context.parking
            self.get_logger().info(
                f'Parking recovery: {ctx.recovery_direction} {ctx.recovery_distance:.2f}m'
            )
        elif new_state == MissionState.FINISH:
            self.get_logger().info('Mission complete!')
        elif new_state == MissionState.ERROR:
            self.get_logger().error(f'Mission error: {self.fsm.context.error_message}')

    def _send_plate_query(self):
        """Send plate verification query to server."""
        if not self._has_interface:
            return

        plate = self.fsm.context.recognition.plate_number
        if not plate:
            return

        msg = self._PlateQuery()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.plate_number = plate
        msg.car_id = self.car_id

        self.pub_plate_query.publish(msg)
        self._plate_query_pending = True
        self.get_logger().info(f'Plate query sent: {plate}')

    def _send_loader_command(self, command: str):
        """Send command to loader."""
        if not self._has_interface:
            return

        msg = self._LoaderCommand()
        msg.command = command

        self.pub_loader_cmd.publish(msg)
        self.get_logger().info(f'Loader command sent: {command}')

    def _setup_turn(self):
        """Setup turn target for the next waypoint."""
        ctx = self.fsm.context
        current_marker = ctx.current_marker_id

        next_idx = ctx.current_waypoint_idx + 1
        if next_idx < len(ctx.waypoint_ids):
            target_marker = ctx.waypoint_ids[next_idx]
        else:
            target_marker = ctx.final_goal_id

        turn_angle = self._calculate_turn_angle(current_marker, target_marker)

        self.fsm.set_turn_target(turn_angle)

        turn_msg = Int32()
        turn_msg.data = int(turn_angle * 100)
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

            unit_scale = data.get('unit_scale', self.unit_scale)
            default_yaw = data.get('default_marker_yaw', self.default_marker_yaw)

            road_markers = data.get('road_markers', {})
            for marker_id, info in road_markers.items():
                pos = info.get('position', [0, 0])
                yaw = info.get('yaw', default_yaw)
                self._marker_positions[int(marker_id)] = (
                    pos[0] * unit_scale,
                    pos[1] * unit_scale
                )
                self._marker_yaw[int(marker_id)] = yaw

            parking_markers = data.get('parking_markers', {})
            for marker_id, info in parking_markers.items():
                pos = info.get('position', [0, 0])
                yaw = info.get('yaw', default_yaw)
                self._marker_positions[int(marker_id)] = (
                    pos[0] * unit_scale,
                    pos[1] * unit_scale
                )
                self._marker_yaw[int(marker_id)] = yaw

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

    def _map_updated_callback(self, msg: Bool):
        """Handle map update notification from server_bridge."""
        if msg.data:
            self.get_logger().info('Map update received, reloading marker map...')
            # Clear existing map
            self._marker_positions.clear()
            self._marker_yaw.clear()
            # Reload from file
            self._load_marker_map()

    def _update_heading_from_path(self, from_marker: int, to_marker: int):
        """Update current heading based on travel direction between markers."""
        if from_marker not in self._marker_positions or to_marker not in self._marker_positions:
            return

        from_pos = self._marker_positions[from_marker]
        to_pos = self._marker_positions[to_marker]

        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]

        if abs(dx) > 0.001 or abs(dy) > 0.001:
            old_heading = self._current_heading
            self._current_heading = math.atan2(dy, dx)
            self.get_logger().info(
                f'[HEADING] Updated: {math.degrees(old_heading):.1f}deg -> {math.degrees(self._current_heading):.1f}deg '
                f'(from marker {from_marker} to {to_marker})'
            )

            heading_msg = Float32()
            heading_msg.data = self._current_heading
            self.pub_heading.publish(heading_msg)

    def _calculate_turn_angle(self, current_marker: int, target_marker: int) -> float:
        """Calculate turn angle to face target marker."""
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

        target_direction = math.atan2(dy, dx)
        turn_angle = wrap_angle(target_direction - self._current_heading)

        self.get_logger().info(
            f'[TURN_ANALYSIS] From:{current_marker} To:{target_marker} | '
            f'CurrHdg:{math.degrees(self._current_heading):.1f}deg | '
            f'TgtDir:{math.degrees(target_direction):.1f}deg | '
            f'FinalTurn:{math.degrees(turn_angle):.1f}deg'
        )

        # ±π 경계 경고: turn_angle이 ±170° 초과 시 방향 불안정 위험
        if abs(turn_angle) > math.radians(170):
            self.get_logger().warn(
                f'[TURN_ANALYSIS] WARNING: turn_angle={math.degrees(turn_angle):.1f}deg near ±180° boundary! '
                f'Direction may be unstable. Check heading accuracy.'
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

        if state == MissionState.FINISH:
            msg.status = 'COMPLETED'
        elif state == MissionState.ERROR:
            msg.status = 'FAILED'
        elif state in (MissionState.IDLE, MissionState.WAIT_VEHICLE):
            msg.status = 'IDLE'
        else:
            msg.status = 'RUNNING'

        msg.current_state = state.name
        msg.current_waypoint_idx = ctx.current_waypoint_idx
        msg.current_marker_id = ctx.current_marker_id
        msg.progress = self.fsm.get_progress()
        msg.message = ctx.error_message if ctx.error_message else f'At {state.name}'

        self.pub_task_status.publish(msg)

    def _publish_parking_status(self):
        """Publish parking status for debugging."""
        if not self._has_interface:
            return

        # Publish during parking and related states
        if self.fsm.state not in (
            MissionState.PARK_DETECT,
            MissionState.PARK_RECOVERY,
            MissionState.PARK_ALIGN_MARKER,
            MissionState.PARK_ALIGN_RECT,
            MissionState.PARK_FINAL,
            MissionState.LOAD,
            MissionState.UNLOAD,
        ):
            return

        ctx = self.fsm.context.parking

        msg = self._ParkingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sub_state = self.fsm.state.name
        msg.detected_slot_id = ctx.detected_slot_id
        msg.target_slot_id = ctx.target_slot_id
        msg.slot_verified = ctx.slot_verified
        msg.marker_distance = ctx.marker_distance
        msg.marker_angle = ctx.marker_angle

        # Build message
        if self.fsm.state == MissionState.PARK_DETECT:
            if ctx.detected_slot_id < 0:
                msg.message = 'Searching for slot marker...'
            else:
                msg.message = f'Detected slot {ctx.detected_slot_id}'
        elif self.fsm.state == MissionState.PARK_RECOVERY:
            msg.message = f'Recovery: {ctx.recovery_direction} {ctx.recovery_distance:.2f}m'
        elif self.fsm.state == MissionState.PARK_ALIGN_MARKER:
            msg.message = f'Aligning marker, angle={math.degrees(ctx.marker_angle):.1f}deg'
        elif self.fsm.state == MissionState.PARK_ALIGN_RECT:
            msg.message = 'Aligning to rectangle'
        elif self.fsm.state == MissionState.PARK_FINAL:
            msg.message = f'Final positioning, dist={ctx.marker_distance:.2f}m'
        elif self.fsm.state == MissionState.LOAD:
            msg.message = f'Loading vehicle, loader={self._loader_status}'
        elif self.fsm.state == MissionState.UNLOAD:
            msg.message = f'Unloading vehicle, loader={self._loader_status}'

        self.pub_parking_status.publish(msg)


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
