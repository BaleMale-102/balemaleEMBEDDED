#!/usr/bin/env python3
"""
mecanum_kinematics.py - Mecanum Wheel Kinematics

Converts body velocities (vx, vy, wz) to individual wheel speeds.

Wheel arrangement (top view, X = forward):
    Front
  FL     FR
    \   /
     \ /
     / \
    /   \
  RL     RR
    Rear

Standard mecanum wheel equations:
  FL = vx - vy - (lx + ly) * wz
  FR = vx + vy + (lx + ly) * wz
  RL = vx + vy - (lx + ly) * wz
  RR = vx - vy + (lx + ly) * wz

where:
  vx = forward velocity (m/s)
  vy = left velocity (m/s)
  wz = counter-clockwise angular velocity (rad/s)
  lx = distance from center to wheel along x-axis (m)
  ly = distance from center to wheel along y-axis (m)
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class WheelSpeeds:
    """Individual wheel speeds in rad/s."""
    front_left: float = 0.0
    front_right: float = 0.0
    rear_left: float = 0.0
    rear_right: float = 0.0

    def to_list(self) -> list:
        return [self.front_left, self.front_right, self.rear_left, self.rear_right]

    def scale(self, factor: float) -> 'WheelSpeeds':
        return WheelSpeeds(
            front_left=self.front_left * factor,
            front_right=self.front_right * factor,
            rear_left=self.rear_left * factor,
            rear_right=self.rear_right * factor
        )

    def clamp(self, max_speed: float) -> 'WheelSpeeds':
        """Clamp all wheel speeds to max_speed, preserving ratio."""
        max_abs = max(
            abs(self.front_left), abs(self.front_right),
            abs(self.rear_left), abs(self.rear_right)
        )
        if max_abs > max_speed and max_abs > 0:
            ratio = max_speed / max_abs
            return self.scale(ratio)
        return self


@dataclass
class MotorPWMs:
    """Motor PWM values (-max to +max)."""
    front_left: int = 0
    front_right: int = 0
    rear_left: int = 0
    rear_right: int = 0

    def to_list(self) -> list:
        return [self.front_left, self.front_right, self.rear_left, self.rear_right]


class MecanumKinematics:
    """Mecanum wheel inverse/forward kinematics."""

    def __init__(
        self,
        wheel_radius: float = 0.03,      # 30mm wheel radius
        wheel_base_x: float = 0.08,      # 80mm from center to wheel (x-axis)
        wheel_base_y: float = 0.06,      # 60mm from center to wheel (y-axis)
        max_wheel_speed: float = 10.0,   # rad/s
        max_pwm: int = 3000,
        pwm_deadzone: int = 100
    ):
        """
        Initialize mecanum kinematics.

        Args:
            wheel_radius: Wheel radius in meters
            wheel_base_x: Distance from center to wheel along x-axis (m)
            wheel_base_y: Distance from center to wheel along y-axis (m)
            max_wheel_speed: Maximum wheel angular speed (rad/s)
            max_pwm: Maximum PWM value for motors
            pwm_deadzone: PWM deadzone (values below this produce no motion)
        """
        self.wheel_radius = wheel_radius
        self.wheel_base_x = wheel_base_x
        self.wheel_base_y = wheel_base_y
        self.max_wheel_speed = max_wheel_speed
        self.max_pwm = max_pwm
        self.pwm_deadzone = pwm_deadzone

        # Kinematic constant
        self._k = wheel_base_x + wheel_base_y

    def inverse(self, vx: float, vy: float, wz: float) -> WheelSpeeds:
        """
        Inverse kinematics: body velocity → wheel speeds.

        Args:
            vx: Forward velocity (m/s, positive = forward)
            vy: Lateral velocity (m/s, positive = left)
            wz: Angular velocity (rad/s, positive = counter-clockwise)

        Returns:
            Individual wheel angular speeds (rad/s)
        """
        # Convert linear velocity to wheel angular velocity
        # v_wheel = v_body / wheel_radius

        # Mecanum inverse kinematics
        fl = (vx - vy - self._k * wz) / self.wheel_radius
        fr = (vx + vy + self._k * wz) / self.wheel_radius
        rl = (vx + vy - self._k * wz) / self.wheel_radius
        rr = (vx - vy + self._k * wz) / self.wheel_radius

        speeds = WheelSpeeds(
            front_left=fl,
            front_right=fr,
            rear_left=rl,
            rear_right=rr
        )

        # Clamp to max speed
        return speeds.clamp(self.max_wheel_speed)

    def forward(self, wheel_speeds: WheelSpeeds) -> Tuple[float, float, float]:
        """
        Forward kinematics: wheel speeds → body velocity.

        Args:
            wheel_speeds: Individual wheel speeds (rad/s)

        Returns:
            (vx, vy, wz) body velocities
        """
        fl = wheel_speeds.front_left
        fr = wheel_speeds.front_right
        rl = wheel_speeds.rear_left
        rr = wheel_speeds.rear_right

        # Mecanum forward kinematics
        vx = (fl + fr + rl + rr) * self.wheel_radius / 4.0
        vy = (-fl + fr + rl - rr) * self.wheel_radius / 4.0
        wz = (-fl + fr - rl + rr) * self.wheel_radius / (4.0 * self._k)

        return vx, vy, wz

    def speeds_to_pwm(self, wheel_speeds: WheelSpeeds) -> MotorPWMs:
        """
        Convert wheel speeds to PWM values.

        Args:
            wheel_speeds: Wheel angular speeds (rad/s)

        Returns:
            Motor PWM values
        """
        def speed_to_pwm(speed: float) -> int:
            if abs(speed) < 0.01:  # Near zero
                return 0

            # Linear mapping: speed → PWM
            ratio = speed / self.max_wheel_speed
            pwm = int(ratio * self.max_pwm)

            # Apply deadzone
            if abs(pwm) < self.pwm_deadzone:
                pwm = 0
            elif pwm > 0:
                pwm = min(pwm + self.pwm_deadzone, self.max_pwm)
            else:
                pwm = max(pwm - self.pwm_deadzone, -self.max_pwm)

            return pwm

        return MotorPWMs(
            front_left=speed_to_pwm(wheel_speeds.front_left),
            front_right=speed_to_pwm(wheel_speeds.front_right),
            rear_left=speed_to_pwm(wheel_speeds.rear_left),
            rear_right=speed_to_pwm(wheel_speeds.rear_right)
        )

    def velocity_to_pwm(self, vx: float, vy: float, wz: float) -> MotorPWMs:
        """
        Direct conversion: body velocity → PWM.

        Args:
            vx, vy, wz: Body velocities

        Returns:
            Motor PWM values
        """
        wheel_speeds = self.inverse(vx, vy, wz)
        return self.speeds_to_pwm(wheel_speeds)
