#!/usr/bin/env python3
"""
state_machine.py - Mission State Machine

Full mission FSM for autonomous parking robot:
  WAIT_VEHICLE -> RECOGNIZE -> LOAD -> DRIVE -> PARK_* -> UNLOAD -> RETURN_HOME -> WAIT_VEHICLE

Parking sub-states:
  PARK_DETECT -> PARK_ALIGN_MARKER -> PARK_ALIGN_RECT -> PARK_FINAL
                 (or PARK_RECOVERY if wrong zone)
"""

from enum import Enum, auto
from typing import Optional, Callable, Dict, List
from dataclasses import dataclass, field
import time


class MissionState(Enum):
    """Mission FSM states."""
    IDLE = auto()

    # Full mission states
    WAIT_VEHICLE = auto()        # Wait at home for vehicle to arrive
    RECOGNIZE = auto()           # ANPR recognition + server query
    LOAD = auto()                # Loading vehicle onto robot
    UNLOAD = auto()              # Unloading vehicle at slot
    RETURN_HOME = auto()         # Returning to home (marker 0)

    # Navigation states
    DRIVE = auto()               # Lane following + marker approach
    STOP_AT_MARKER = auto()      # Stop at marker
    ADVANCE_TO_CENTER = auto()   # Move to marker center
    ALIGN_TO_MARKER = auto()     # Precise alignment
    STOP_BUMP = auto()           # Brief stop for stability
    TURNING = auto()             # In-place rotation
    PARK = auto()                # Legacy parking (redirects to PARK_DETECT)

    # Parking sub-states
    PARK_DETECT = auto()         # Detect slot marker with side camera
    PARK_RECOVERY = auto()       # Move to correct zone if wrong slot detected
    PARK_ALIGN_MARKER = auto()   # Align using marker angle (vx control)
    PARK_ALIGN_RECT = auto()     # Align using yellow rectangle (vy control)
    PARK_FINAL = auto()          # Final distance adjustment

    FINISH = auto()              # Mission complete
    ERROR = auto()               # Error state


# Home marker ID
HOME_MARKER_ID = 0

# Slot group definitions
SLOT_GROUPS = {
    'A': range(16, 20),  # y=82.5cm (IDs 16-19)
    'B': range(20, 24),  # y=136.33cm (IDs 20-23)
    'C': range(24, 28),  # y=161.66cm (IDs 24-27)
}

# Zone Y positions (meters)
ZONE_Y = {
    'A': 0.825,
    'B': 1.3633,
    'C': 1.6166,
}


def get_slot_zone(slot_id: int) -> Optional[str]:
    """Get zone letter for slot ID."""
    for zone, ids in SLOT_GROUPS.items():
        if slot_id in ids:
            return zone
    return None


def is_same_zone(slot_id1: int, slot_id2: int) -> bool:
    """Check if two slots are in the same zone."""
    zone1 = get_slot_zone(slot_id1)
    zone2 = get_slot_zone(slot_id2)
    return zone1 is not None and zone1 == zone2


def get_slot_direction(detected_id: int, target_id: int) -> int:
    """
    Get horizontal direction from detected slot to target slot.
    Returns: 1 for right (+vy), -1 for left (-vy), 0 if same
    """
    if detected_id == target_id:
        return 0
    return 1 if target_id > detected_id else -1


def get_recovery_info(detected_id: int, target_id: int) -> tuple:
    """
    Get recovery direction and distance when in wrong zone.

    Returns:
        (direction, distance): 'FORWARD'/'BACKWARD' and distance in meters
    """
    current_zone = get_slot_zone(detected_id)
    target_zone = get_slot_zone(target_id)

    if current_zone is None or target_zone is None:
        return ('FORWARD', 0.3)  # Default fallback

    if current_zone == target_zone:
        return (None, 0.0)

    dy = ZONE_Y[target_zone] - ZONE_Y[current_zone]
    direction = 'FORWARD' if dy > 0 else 'BACKWARD'
    return (direction, abs(dy))


@dataclass
class ParkingContext:
    """Parking-specific context data."""
    target_slot_id: int = -1          # Target parking slot
    detected_slot_id: int = -1        # Currently detected slot from side camera
    slot_verified: bool = False       # Slot zone verification passed
    marker_distance: float = 0.0      # Distance to detected marker
    marker_angle: float = 0.0         # Angle to detected marker
    recovery_direction: str = ''      # 'FORWARD' or 'BACKWARD'
    recovery_distance: float = 0.0    # Distance to move for recovery
    recovery_start_time: float = 0.0  # Recovery start time
    align_marker_done: bool = False   # Marker alignment complete
    align_rect_done: bool = False     # Rectangle alignment complete
    final_done: bool = False          # Final positioning complete


@dataclass
class LoaderContext:
    """Loader-specific context data."""
    is_loaded: bool = False           # Vehicle currently loaded
    load_complete: bool = False       # Load operation complete
    unload_complete: bool = False     # Unload operation complete
    loader_error: bool = False        # Loader error occurred


@dataclass
class RecognitionContext:
    """Vehicle recognition context data."""
    plate_number: str = ''            # Detected plate number
    plate_verified: bool = False      # Server verified the plate
    assigned_slot_id: int = -1        # Server-assigned parking slot
    assigned_waypoints: List[int] = None  # Waypoints to reach slot
    query_sent: bool = False          # Query sent to server
    response_received: bool = False   # Response received from server

    def __post_init__(self):
        if self.assigned_waypoints is None:
            self.assigned_waypoints = []


@dataclass
class MissionContext:
    """Shared context for state machine."""
    # Route
    waypoint_ids: List[int] = None
    final_goal_id: int = -1
    current_waypoint_idx: int = 0
    task_id: str = ''
    task_type: str = ''

    # State tracking
    current_marker_id: int = -1
    turn_target_rad: float = 0.0
    state_enter_time: float = 0.0
    error_message: str = ''

    # Flags
    marker_reached: bool = False
    turn_complete: bool = False
    align_complete: bool = False

    # Sub-contexts
    parking: ParkingContext = field(default_factory=ParkingContext)
    loader: LoaderContext = field(default_factory=LoaderContext)
    recognition: RecognitionContext = field(default_factory=RecognitionContext)

    # Full mission mode
    is_full_mission: bool = False     # Full WAIT->RECOGNIZE->LOAD->...->UNLOAD->RETURN flow

    def __post_init__(self):
        if self.waypoint_ids is None:
            self.waypoint_ids = []

    def reset(self):
        """Reset context for new mission."""
        self.waypoint_ids = []
        self.final_goal_id = -1
        self.current_waypoint_idx = 0
        self.task_id = ''
        self.task_type = ''
        self.current_marker_id = -1
        self.turn_target_rad = 0.0
        self.state_enter_time = 0.0
        self.error_message = ''
        self.marker_reached = False
        self.turn_complete = False
        self.align_complete = False
        self.parking = ParkingContext()
        self.loader = LoaderContext()
        self.recognition = RecognitionContext()
        self.is_full_mission = False

    @property
    def current_target_marker(self) -> int:
        """Get current target marker ID."""
        if self.current_waypoint_idx < len(self.waypoint_ids):
            return self.waypoint_ids[self.current_waypoint_idx]
        return self.final_goal_id

    @property
    def is_last_waypoint(self) -> bool:
        """Check if at final waypoint."""
        return self.current_waypoint_idx >= len(self.waypoint_ids)

    def advance_waypoint(self) -> bool:
        """Move to next waypoint. Returns True if more waypoints exist."""
        self.current_waypoint_idx += 1
        self.marker_reached = False
        return not self.is_last_waypoint


class StateMachine:
    """Finite State Machine for mission execution."""

    def __init__(self, timeouts: Dict[str, float] = None, delays: Dict[str, float] = None):
        self._state = MissionState.IDLE
        self._context = MissionContext()
        self._transitions: Dict[MissionState, Callable[[], Optional[MissionState]]] = {}

        # Default timeout configuration (테스트용 - 타임아웃 비활성화)
        default_timeouts = {
            MissionState.STOP_AT_MARKER: 2.0,
            MissionState.ADVANCE_TO_CENTER: 0.5,
            # MissionState.ALIGN_TO_MARKER: 5.0,  # 비활성화
            MissionState.STOP_BUMP: 0.5,
            # MissionState.TURNING: 30.0,  # 비활성화
            # MissionState.PARK: 30.0,  # 비활성화
            # Parking sub-state timeouts - 비활성화
            # MissionState.PARK_DETECT: 10.0,
            # MissionState.PARK_RECOVERY: 15.0,
            # MissionState.PARK_ALIGN_MARKER: 15.0,
            # MissionState.PARK_ALIGN_RECT: 10.0,
            # MissionState.PARK_FINAL: 10.0,
            # Full mission timeouts
            MissionState.RECOGNIZE: 30.0,
            MissionState.LOAD: 30.0,
            MissionState.UNLOAD: 30.0,
            # WAIT_VEHICLE and RETURN_HOME have no timeout
        }

        # Apply custom timeouts if provided
        if timeouts:
            for key, value in timeouts.items():
                try:
                    state = MissionState[key]
                    default_timeouts[state] = value
                except KeyError:
                    pass

        self._state_timeouts = default_timeouts

        # State transition delays
        self._delays = {
            'stop_at_marker': 0.5,
            'stop_bump': 0.3,
        }
        if delays:
            self._delays.update(delays)

        # Callbacks
        self._on_state_change: Optional[Callable[[MissionState, MissionState], None]] = None

        self._setup_transitions()

    def _setup_transitions(self):
        """Setup state transition handlers."""
        self._transitions = {
            MissionState.IDLE: self._idle_transition,
            # Full mission states
            MissionState.WAIT_VEHICLE: self._wait_vehicle_transition,
            MissionState.RECOGNIZE: self._recognize_transition,
            MissionState.LOAD: self._load_transition,
            MissionState.UNLOAD: self._unload_transition,
            MissionState.RETURN_HOME: self._return_home_transition,
            # Navigation states
            MissionState.DRIVE: self._drive_transition,
            MissionState.STOP_AT_MARKER: self._stop_at_marker_transition,
            MissionState.ADVANCE_TO_CENTER: self._advance_transition,
            MissionState.ALIGN_TO_MARKER: self._align_transition,
            MissionState.STOP_BUMP: self._stop_bump_transition,
            MissionState.TURNING: self._turning_transition,
            MissionState.PARK: self._park_legacy_transition,
            # Parking sub-states
            MissionState.PARK_DETECT: self._park_detect_transition,
            MissionState.PARK_RECOVERY: self._park_recovery_transition,
            MissionState.PARK_ALIGN_MARKER: self._park_align_marker_transition,
            MissionState.PARK_ALIGN_RECT: self._park_align_rect_transition,
            MissionState.PARK_FINAL: self._park_final_transition,
            MissionState.FINISH: lambda: None,
            MissionState.ERROR: lambda: None,
        }

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def context(self) -> MissionContext:
        return self._context

    def set_state_change_callback(self, callback: Callable[[MissionState, MissionState], None]):
        """Set callback for state changes."""
        self._on_state_change = callback

    def start_waiting(self):
        """Start waiting for vehicle at home position."""
        self._context.reset()
        self._context.is_full_mission = True
        self._change_state(MissionState.WAIT_VEHICLE)

    def start_mission(self, waypoint_ids: List[int], final_goal_id: int,
                      task_id: str = '', task_type: str = ''):
        """Start a navigation mission (from server command or manual)."""
        self._context.reset()
        self._context.waypoint_ids = waypoint_ids
        self._context.final_goal_id = final_goal_id
        self._context.task_id = task_id
        self._context.task_type = task_type

        # If final goal is a parking slot, store it and set task_type
        if get_slot_zone(final_goal_id) is not None:
            self._context.parking.target_slot_id = final_goal_id
            if not task_type:
                self._context.task_type = 'PARK'

        self._change_state(MissionState.DRIVE)

    def start_full_mission(self, plate_number: str, assigned_slot_id: int,
                           waypoint_ids: List[int], task_id: str = ''):
        """Start full mission after plate verification (called after RECOGNIZE)."""
        self._context.waypoint_ids = waypoint_ids
        self._context.final_goal_id = assigned_slot_id
        self._context.parking.target_slot_id = assigned_slot_id
        self._context.task_id = task_id
        self._context.task_type = 'PARK'
        self._context.is_full_mission = True

        # Move to LOAD state
        self._change_state(MissionState.LOAD)

    def cancel_mission(self):
        """Cancel current mission."""
        self._context.error_message = 'Mission cancelled'
        self._change_state(MissionState.IDLE)

    def update(self):
        """Update state machine. Should be called periodically."""
        if self._state in (MissionState.IDLE, MissionState.FINISH, MissionState.ERROR):
            return

        # Check timeout (some states have no timeout)
        timeout = self._state_timeouts.get(self._state)
        if timeout:
            elapsed = time.time() - self._context.state_enter_time
            if elapsed > timeout:
                self._handle_timeout()
                return

        # Run transition handler
        handler = self._transitions.get(self._state)
        if handler:
            next_state = handler()
            if next_state:
                self._change_state(next_state)

    # Navigation notifications
    def notify_marker_reached(self, marker_id: int):
        """Notify that a marker has been reached."""
        self._context.marker_reached = True
        self._context.current_marker_id = marker_id

    def notify_turn_complete(self):
        """Notify that turn is complete."""
        self._context.turn_complete = True

    def notify_align_complete(self):
        """Notify that alignment is complete."""
        self._context.align_complete = True

    def set_turn_target(self, angle_rad: float):
        """Set turn target angle."""
        self._context.turn_target_rad = angle_rad

    # Recognition notifications
    def notify_plate_detected(self, plate_number: str):
        """Notify that a plate was detected by ANPR."""
        self._context.recognition.plate_number = plate_number

    def notify_plate_query_sent(self):
        """Notify that plate query was sent to server."""
        self._context.recognition.query_sent = True

    def notify_plate_response(self, verified: bool, slot_id: int, waypoints: List[int], message: str = ''):
        """Notify plate verification response from server."""
        self._context.recognition.response_received = True
        self._context.recognition.plate_verified = verified
        self._context.recognition.assigned_slot_id = slot_id
        self._context.recognition.assigned_waypoints = waypoints
        if not verified:
            self._context.error_message = message if message else 'Plate not verified'

    # Loader notifications
    def notify_load_complete(self):
        """Notify that vehicle loading is complete."""
        self._context.loader.load_complete = True
        self._context.loader.is_loaded = True

    def notify_unload_complete(self):
        """Notify that vehicle unloading is complete."""
        self._context.loader.unload_complete = True
        self._context.loader.is_loaded = False

    def notify_loader_error(self):
        """Notify loader error."""
        self._context.loader.loader_error = True

    # Parking notifications
    def notify_park_detect_done(self, detected_id: int, distance: float, angle: float):
        """Notify slot marker detected from side camera."""
        self._context.parking.detected_slot_id = detected_id
        self._context.parking.marker_distance = distance
        self._context.parking.marker_angle = angle

    def notify_park_align_marker_done(self):
        """Notify marker alignment complete."""
        self._context.parking.align_marker_done = True

    def notify_park_align_rect_done(self):
        """Notify rectangle alignment complete."""
        self._context.parking.align_rect_done = True

    def notify_park_final_done(self):
        """Notify final positioning complete."""
        self._context.parking.final_done = True

    def _change_state(self, new_state: MissionState):
        """Change to a new state."""
        old_state = self._state
        self._state = new_state
        self._context.state_enter_time = time.time()

        # Reset state-specific flags
        if new_state == MissionState.TURNING:
            self._context.turn_complete = False
        elif new_state == MissionState.ALIGN_TO_MARKER:
            self._context.align_complete = False
        elif new_state == MissionState.ADVANCE_TO_CENTER:
            self._context.align_complete = False
        # Full mission resets
        elif new_state == MissionState.WAIT_VEHICLE:
            self._context.recognition = RecognitionContext()
        elif new_state == MissionState.RECOGNIZE:
            self._context.recognition.query_sent = False
            self._context.recognition.response_received = False
        elif new_state == MissionState.LOAD:
            self._context.loader.load_complete = False
        elif new_state == MissionState.UNLOAD:
            self._context.loader.unload_complete = False
        elif new_state == MissionState.RETURN_HOME:
            # Setup return waypoints (just go to marker 0)
            self._context.waypoint_ids = []
            self._context.final_goal_id = HOME_MARKER_ID
            self._context.current_waypoint_idx = 0
            self._context.marker_reached = False
        # Parking state resets
        elif new_state == MissionState.PARK_DETECT:
            self._context.parking.detected_slot_id = -1
            self._context.parking.slot_verified = False
        elif new_state == MissionState.PARK_RECOVERY:
            self._context.parking.recovery_start_time = time.time()
        elif new_state == MissionState.PARK_ALIGN_MARKER:
            self._context.parking.align_marker_done = False
        elif new_state == MissionState.PARK_ALIGN_RECT:
            self._context.parking.align_rect_done = False
        elif new_state == MissionState.PARK_FINAL:
            self._context.parking.final_done = False

        if self._on_state_change:
            self._on_state_change(old_state, new_state)

    def _handle_timeout(self):
        """Handle state timeout."""
        # TURNING timeout: force to align state
        if self._state == MissionState.TURNING:
            self._context.turn_complete = True
            self._context.advance_waypoint()
            self._change_state(MissionState.ALIGN_TO_MARKER)
            return

        # ALIGN_TO_MARKER timeout: proceed to DRIVE
        if self._state == MissionState.ALIGN_TO_MARKER:
            self._context.align_complete = True
            self._change_state(MissionState.DRIVE)
            return

        # Recognition timeout
        if self._state == MissionState.RECOGNIZE:
            self._context.error_message = 'Plate verification timeout'
            self._change_state(MissionState.ERROR)
            return

        # Loader timeouts
        if self._state == MissionState.LOAD:
            self._context.error_message = 'Load operation timeout'
            self._change_state(MissionState.ERROR)
            return

        if self._state == MissionState.UNLOAD:
            self._context.error_message = 'Unload operation timeout'
            self._change_state(MissionState.ERROR)
            return

        # Parking timeouts - try to proceed
        if self._state == MissionState.PARK_DETECT:
            self._context.error_message = 'Parking slot marker not detected'
            self._change_state(MissionState.ERROR)
            return

        if self._state == MissionState.PARK_RECOVERY:
            self._change_state(MissionState.PARK_DETECT)
            return

        if self._state == MissionState.PARK_ALIGN_MARKER:
            self._context.parking.align_marker_done = True
            self._change_state(MissionState.PARK_ALIGN_RECT)
            return

        if self._state == MissionState.PARK_ALIGN_RECT:
            self._context.parking.align_rect_done = True
            self._change_state(MissionState.PARK_FINAL)
            return

        if self._state == MissionState.PARK_FINAL:
            self._context.parking.final_done = True
            # In full mission, go to UNLOAD; otherwise FINISH
            if self._context.is_full_mission:
                self._change_state(MissionState.UNLOAD)
            else:
                self._change_state(MissionState.FINISH)
            return

        self._context.error_message = f'Timeout in state {self._state.name}'
        self._change_state(MissionState.ERROR)

    # Transition handlers
    def _idle_transition(self) -> Optional[MissionState]:
        return None

    def _wait_vehicle_transition(self) -> Optional[MissionState]:
        """Wait for plate detection."""
        if self._context.recognition.plate_number:
            return MissionState.RECOGNIZE
        return None

    def _recognize_transition(self) -> Optional[MissionState]:
        """Wait for server response."""
        ctx = self._context.recognition

        if ctx.response_received:
            if ctx.plate_verified and ctx.assigned_slot_id >= 0:
                # Server verified - start full mission
                self._context.waypoint_ids = ctx.assigned_waypoints
                self._context.final_goal_id = ctx.assigned_slot_id
                self._context.parking.target_slot_id = ctx.assigned_slot_id
                self._context.task_type = 'PARK'
                return MissionState.LOAD
            else:
                # Not verified - back to waiting
                self._context.recognition = RecognitionContext()
                return MissionState.WAIT_VEHICLE

        return None

    def _load_transition(self) -> Optional[MissionState]:
        """Wait for load complete."""
        if self._context.loader.loader_error:
            self._context.error_message = 'Loader error during load'
            return MissionState.ERROR

        if self._context.loader.load_complete:
            return MissionState.DRIVE

        return None

    def _unload_transition(self) -> Optional[MissionState]:
        """Wait for unload complete."""
        if self._context.loader.loader_error:
            self._context.error_message = 'Loader error during unload'
            return MissionState.ERROR

        if self._context.loader.unload_complete:
            if self._context.is_full_mission:
                return MissionState.RETURN_HOME
            else:
                return MissionState.FINISH

        return None

    def _return_home_transition(self) -> Optional[MissionState]:
        """Wait to reach home marker."""
        if self._context.marker_reached and self._context.current_marker_id == HOME_MARKER_ID:
            # Back to waiting for next vehicle
            return MissionState.WAIT_VEHICLE
        return None

    def _drive_transition(self) -> Optional[MissionState]:
        if self._context.marker_reached:
            return MissionState.STOP_AT_MARKER
        return None

    def _stop_at_marker_transition(self) -> Optional[MissionState]:
        elapsed = time.time() - self._context.state_enter_time
        if elapsed > self._delays['stop_at_marker']:
            return MissionState.ADVANCE_TO_CENTER
        return None

    def _advance_transition(self) -> Optional[MissionState]:
        if self._context.align_complete:
            return MissionState.STOP_BUMP
        return None

    def _align_transition(self) -> Optional[MissionState]:
        if self._context.align_complete:
            # Check if in RETURN_HOME mode
            if self._state == MissionState.RETURN_HOME:
                return MissionState.DRIVE
            return MissionState.DRIVE
        return None

    def _stop_bump_transition(self) -> Optional[MissionState]:
        elapsed = time.time() - self._context.state_enter_time
        if elapsed > self._delays['stop_bump']:
            if self._context.is_last_waypoint:
                # Check if this is return home
                if self._context.final_goal_id == HOME_MARKER_ID:
                    return MissionState.WAIT_VEHICLE
                elif self._context.task_type in ('PARK', 'DROPOFF'):
                    return MissionState.PARK_DETECT
                else:
                    return MissionState.FINISH
            else:
                return MissionState.TURNING
        return None

    def _turning_transition(self) -> Optional[MissionState]:
        if self._context.turn_complete:
            self._context.advance_waypoint()
            # If next target is a parking slot, go directly to PARK_DETECT
            if self._context.is_last_waypoint and self._context.task_type in ('PARK', 'DROPOFF'):
                return MissionState.PARK_DETECT
            return MissionState.ALIGN_TO_MARKER
        return None

    def _park_legacy_transition(self) -> Optional[MissionState]:
        """Legacy PARK state - redirect to PARK_DETECT."""
        return MissionState.PARK_DETECT

    # Parking sub-state transitions
    def _park_detect_transition(self) -> Optional[MissionState]:
        """Wait for slot marker detection."""
        ctx = self._context.parking

        if ctx.detected_slot_id < 0:
            return None  # Still searching

        target_zone = get_slot_zone(ctx.target_slot_id)
        detected_zone = get_slot_zone(ctx.detected_slot_id)

        if target_zone is None or detected_zone is None:
            self._context.error_message = f'Invalid slot ID: target={ctx.target_slot_id}, detected={ctx.detected_slot_id}'
            return MissionState.ERROR

        if detected_zone != target_zone:
            direction, distance = get_recovery_info(ctx.detected_slot_id, ctx.target_slot_id)
            ctx.recovery_direction = direction
            ctx.recovery_distance = distance
            ctx.slot_verified = False
            return MissionState.PARK_RECOVERY

        ctx.slot_verified = True
        return MissionState.PARK_ALIGN_MARKER

    def _park_recovery_transition(self) -> Optional[MissionState]:
        """Move to correct zone based on recovery info."""
        ctx = self._context.parking
        elapsed = time.time() - ctx.recovery_start_time
        expected_time = ctx.recovery_distance / 0.01

        if elapsed > expected_time:
            ctx.detected_slot_id = -1
            return MissionState.PARK_DETECT

        return None

    def _park_align_marker_transition(self) -> Optional[MissionState]:
        """Align front/back using side marker angle."""
        if self._context.parking.align_marker_done:
            return MissionState.PARK_ALIGN_RECT
        return None

    def _park_align_rect_transition(self) -> Optional[MissionState]:
        """Align left/right using yellow rectangle offset."""
        if self._context.parking.align_rect_done:
            return MissionState.PARK_FINAL
        return None

    def _park_final_transition(self) -> Optional[MissionState]:
        """Final distance adjustment to slot center."""
        if self._context.parking.final_done:
            # In full mission, proceed to UNLOAD
            if self._context.is_full_mission:
                return MissionState.UNLOAD
            return MissionState.FINISH
        return None

    def get_progress(self) -> float:
        """Get mission progress (0-1)."""
        if self._state == MissionState.FINISH:
            return 1.0
        elif self._state in (MissionState.IDLE, MissionState.WAIT_VEHICLE):
            return 0.0

        # Full mission progress
        if self._context.is_full_mission:
            state_progress = {
                MissionState.RECOGNIZE: 0.05,
                MissionState.LOAD: 0.1,
                MissionState.DRIVE: 0.3,
                MissionState.PARK_DETECT: 0.5,
                MissionState.PARK_ALIGN_MARKER: 0.55,
                MissionState.PARK_ALIGN_RECT: 0.6,
                MissionState.PARK_FINAL: 0.65,
                MissionState.UNLOAD: 0.7,
                MissionState.RETURN_HOME: 0.85,
            }
            base = state_progress.get(self._state, 0.3)

            # Add waypoint progress for DRIVE and RETURN_HOME
            if self._state == MissionState.DRIVE:
                total = len(self._context.waypoint_ids) + 1
                if total > 0:
                    wp_progress = self._context.current_waypoint_idx / total
                    base = 0.1 + wp_progress * 0.35

            return min(1.0, base)

        # Simple mission progress
        total = len(self._context.waypoint_ids) + 1
        current = self._context.current_waypoint_idx

        if self._state in (MissionState.PARK_DETECT, MissionState.PARK_RECOVERY):
            return min(1.0, (current + 0.6) / total)
        elif self._state == MissionState.PARK_ALIGN_MARKER:
            return min(1.0, (current + 0.7) / total)
        elif self._state == MissionState.PARK_ALIGN_RECT:
            return min(1.0, (current + 0.8) / total)
        elif self._state == MissionState.PARK_FINAL:
            return min(1.0, (current + 0.9) / total)

        return min(1.0, current / total)
