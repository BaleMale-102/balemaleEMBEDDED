#!/usr/bin/env python3
"""
pid_controller.py - Simple PID Controller

Standard PID controller with anti-windup and derivative filtering.
"""

import time
from typing import Optional, Tuple


class PIDController:
    """PID Controller with anti-windup."""

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        setpoint: float = 0.0,
        output_limits: Tuple[float, float] = (-1.0, 1.0),
        integral_limits: Optional[Tuple[float, float]] = None,
        derivative_filter: float = 0.1
    ):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Target value
            output_limits: (min, max) output limits
            integral_limits: (min, max) integral term limits (anti-windup)
            derivative_filter: Derivative low-pass filter coefficient (0-1)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.integral_limits = integral_limits or output_limits
        self.derivative_filter = derivative_filter

        # State
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._last_time = None

    def update(self, measurement: float, dt: Optional[float] = None) -> float:
        """
        Compute PID output.

        Args:
            measurement: Current measured value
            dt: Time step (auto-calculated if None)

        Returns:
            Control output
        """
        current_time = time.time()

        if dt is None:
            if self._last_time is None:
                dt = 0.05  # Default 20Hz
            else:
                dt = current_time - self._last_time

        self._last_time = current_time

        # Ensure dt is positive
        dt = max(dt, 0.001)

        # Calculate error
        error = self.setpoint - measurement

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = self._clamp(
            self._integral,
            self.integral_limits[0] / max(self.ki, 0.001),
            self.integral_limits[1] / max(self.ki, 0.001)
        )
        i_term = self.ki * self._integral

        # Derivative term with filtering
        derivative = (error - self._last_error) / dt
        self._last_derivative = (
            self.derivative_filter * derivative +
            (1 - self.derivative_filter) * self._last_derivative
        )
        d_term = self.kd * self._last_derivative

        self._last_error = error

        # Calculate output
        output = p_term + i_term + d_term

        # Apply output limits
        output = self._clamp(output, self.output_limits[0], self.output_limits[1])

        return output

    def reset(self):
        """Reset controller state."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._last_time = None

    def set_setpoint(self, setpoint: float):
        """Set new setpoint."""
        self.setpoint = setpoint

    def set_gains(self, kp: float = None, ki: float = None, kd: float = None):
        """Update gains."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))


class RateLimiter:
    """Rate limiter for smooth acceleration/deceleration."""

    def __init__(
        self,
        rate_up: float = 1.0,
        rate_down: float = 1.0,
        initial_value: float = 0.0
    ):
        """
        Initialize rate limiter.

        Args:
            rate_up: Maximum rate of increase per second
            rate_down: Maximum rate of decrease per second
            initial_value: Starting value
        """
        self.rate_up = rate_up
        self.rate_down = rate_down
        self._value = initial_value
        self._last_time = None

    def update(self, target: float, dt: Optional[float] = None) -> float:
        """
        Apply rate limiting.

        Args:
            target: Target value
            dt: Time step

        Returns:
            Rate-limited value
        """
        current_time = time.time()

        if dt is None:
            if self._last_time is None:
                dt = 0.05
            else:
                dt = current_time - self._last_time

        self._last_time = current_time

        diff = target - self._value

        if diff > 0:
            max_change = self.rate_up * dt
            self._value += min(diff, max_change)
        else:
            max_change = self.rate_down * dt
            self._value += max(diff, -max_change)

        return self._value

    def reset(self, value: float = 0.0):
        """Reset to initial value."""
        self._value = value
        self._last_time = None

    @property
    def value(self) -> float:
        return self._value
