#!/usr/bin/env python3
"""
state_machine.py - Mission State Machine

FSM states for autonomous mission execution:
  IDLE -> DRIVE -> STOP_AT_MARKER -> ALIGN -> TURN -> DRIVE -> ... -> FINISH
"""

from enum import Enum, auto
from typing import Optional, Callable, Dict, List
from dataclasses import dataclass
import time


class MissionState(Enum):
    """Mission FSM states."""
    IDLE = auto()
    DRIVE = auto()               # Lane following + marker approach
    STOP_AT_MARKER = auto()      # Stop at marker
    ADVANCE_TO_CENTER = auto()   # Move to marker center
    ALIGN_TO_MARKER = auto()     # Precise alignment
    STOP_BUMP = auto()           # Brief stop for stability
    TURNING = auto()             # In-place rotation
    PARK = auto()                # Parking maneuver
    FINISH = auto()              # Mission complete
    ERROR = auto()               # Error state


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

        # Default timeout configuration
        default_timeouts = {
            MissionState.STOP_AT_MARKER: 2.0,
            MissionState.ADVANCE_TO_CENTER: 0.5,  # 마커까지 전진 시간
            MissionState.ALIGN_TO_MARKER: 5.0,
            MissionState.STOP_BUMP: 0.5,
            MissionState.TURNING: 30.0,  # 회전 시간
            MissionState.PARK: 30.0,
        }

        # Apply custom timeouts if provided
        if timeouts:
            for key, value in timeouts.items():
                try:
                    state = MissionState[key]
                    default_timeouts[state] = value
                except KeyError:
                    pass  # Ignore invalid state names

        self._state_timeouts = default_timeouts

        # State transition delays
        self._delays = {
            'stop_at_marker': 0.5,  # STOP_AT_MARKER 후 대기 시간
            'stop_bump': 0.3,       # STOP_BUMP 후 대기 시간
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
            MissionState.DRIVE: self._drive_transition,
            MissionState.STOP_AT_MARKER: self._stop_at_marker_transition,
            MissionState.ADVANCE_TO_CENTER: self._advance_transition,
            MissionState.ALIGN_TO_MARKER: self._align_transition,
            MissionState.STOP_BUMP: self._stop_bump_transition,
            MissionState.TURNING: self._turning_transition,
            MissionState.PARK: self._park_transition,
            MissionState.FINISH: lambda: None,  # Terminal state
            MissionState.ERROR: lambda: None,   # Terminal state
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

    def start_mission(self, waypoint_ids: List[int], final_goal_id: int,
                      task_id: str = '', task_type: str = ''):
        """Start a new mission."""
        self._context.reset()
        self._context.waypoint_ids = waypoint_ids
        self._context.final_goal_id = final_goal_id
        self._context.task_id = task_id
        self._context.task_type = task_type

        self._change_state(MissionState.DRIVE)

    def cancel_mission(self):
        """Cancel current mission."""
        self._context.error_message = 'Mission cancelled'
        self._change_state(MissionState.IDLE)

    def update(self):
        """Update state machine. Should be called periodically."""
        if self._state in (MissionState.IDLE, MissionState.FINISH, MissionState.ERROR):
            return

        # Check timeout
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

        if self._on_state_change:
            self._on_state_change(old_state, new_state)

    def _handle_timeout(self):
        """Handle state timeout."""
        # TURNING timeout: 마커 정렬 상태로 전환 (ERROR 대신)
        if self._state == MissionState.TURNING:
            self._context.turn_complete = True  # 강제 완료 처리
            self._context.advance_waypoint()
            self._change_state(MissionState.ALIGN_TO_MARKER)
            return

        # ALIGN_TO_MARKER timeout: 그냥 DRIVE로 전환
        if self._state == MissionState.ALIGN_TO_MARKER:
            self._context.align_complete = True
            self._change_state(MissionState.DRIVE)
            return

        self._context.error_message = f'Timeout in state {self._state.name}'
        self._change_state(MissionState.ERROR)

    # Transition handlers
    def _idle_transition(self) -> Optional[MissionState]:
        return None  # Wait for start_mission()

    def _drive_transition(self) -> Optional[MissionState]:
        if self._context.marker_reached:
            return MissionState.STOP_AT_MARKER
        return None

    def _stop_at_marker_transition(self) -> Optional[MissionState]:
        # Wait briefly then advance
        elapsed = time.time() - self._context.state_enter_time
        if elapsed > self._delays['stop_at_marker']:
            return MissionState.ADVANCE_TO_CENTER
        return None

    def _advance_transition(self) -> Optional[MissionState]:
        # Move to marker center, wait for align_done signal
        if self._context.align_complete:
            return MissionState.STOP_BUMP
        return None

    def _align_transition(self) -> Optional[MissionState]:
        if self._context.align_complete:
            return MissionState.DRIVE  # STOP_BUMP 대신 바로 DRIVE
        return None

    def _stop_bump_transition(self) -> Optional[MissionState]:
        elapsed = time.time() - self._context.state_enter_time
        if elapsed > self._delays['stop_bump']:
            # Decide next action
            if self._context.is_last_waypoint:
                # At final goal
                if self._context.task_type in ('PARK', 'DROPOFF'):
                    return MissionState.PARK
                else:
                    return MissionState.FINISH
            else:
                # More waypoints - need to turn
                return MissionState.TURNING
        return None

    def _turning_transition(self) -> Optional[MissionState]:
        if self._context.turn_complete:
            self._context.advance_waypoint()
            return MissionState.DRIVE
        return None

    def _park_transition(self) -> Optional[MissionState]:
        # Parking is handled externally
        # Will be set to FINISH when parking complete
        return None

    def get_progress(self) -> float:
        """Get mission progress (0-1)."""
        total = len(self._context.waypoint_ids) + 1
        current = self._context.current_waypoint_idx

        if self._state == MissionState.FINISH:
            return 1.0
        elif self._state == MissionState.IDLE:
            return 0.0
        else:
            return min(1.0, current / total)
