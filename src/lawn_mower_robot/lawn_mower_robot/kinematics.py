"""Skid-steer kinematic model (paper Sec. 3.2), written against config symbols.

A skid-steer vehicle of track width B turns like a differential-drive vehicle
of width chi * B, where chi >= 1 is the ICR expansion factor identified for the
terrain (Yu et al., 2009). Longitudinal slip s scales the
wheel rim speed to ground speed:

    v     = r (1 - s) (w_R + w_L) / 2
    omega = r (1 - s) (w_R - w_L) / (chi B)

All symbols come from the robot config; nothing here is robot specific.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SkidSteerKinematics:
    track_width: float          # B [m]
    wheel_radius: float         # r [m]
    icr_expansion_factor: float  # chi [-]
    longitudinal_slip: float    # s [-]
    max_linear_speed: float     # [m/s]
    max_angular_speed: float    # [rad/s]
    wheel_max_speed: float      # [rad/s]

    @classmethod
    def from_config(cls, cfg):
        return cls(
            track_width=cfg.geometry.track_width,
            wheel_radius=cfg.geometry.wheel_radius,
            icr_expansion_factor=cfg.terrain.icr_expansion_factor,
            longitudinal_slip=cfg.terrain.longitudinal_slip,
            max_linear_speed=cfg.limits.max_linear_speed,
            max_angular_speed=cfg.limits.max_angular_speed,
            wheel_max_speed=cfg.drive.wheel_max_speed,
        )

    @property
    def effective_track_width(self):
        """chi * B: track width of the equivalent differential-drive vehicle."""
        return self.icr_expansion_factor * self.track_width

    @property
    def effective_wheel_radius(self):
        """r (1 - s): ground speed per unit wheel speed."""
        return self.wheel_radius * (1 - self.longitudinal_slip)

    def forward(self, wheel_left, wheel_right):
        """Wheel speeds [rad/s] -> body (v [m/s], omega [rad/s])."""
        r = self.effective_wheel_radius
        v = r * (wheel_right + wheel_left) / 2
        omega = r * (wheel_right - wheel_left) / self.effective_track_width
        return v, omega

    def inverse(self, v, omega):
        """Body (v, omega) -> wheel speeds (left, right) [rad/s]."""
        r = self.effective_wheel_radius
        half = omega * self.effective_track_width / 2
        return (v - half) / r, (v + half) / r

    def side_ground_speeds(self, v, omega):
        """Ground speed of the left and right wheel sides [m/s]."""
        half = omega * self.effective_track_width / 2
        return v - half, v + half

    def saturate(self, v, omega):
        """Scale (v, omega) into the speed and wheel limits, keeping the curvature."""
        scale = 1.0
        if abs(v) > self.max_linear_speed:
            scale = min(scale, self.max_linear_speed / abs(v))
        if abs(omega) > self.max_angular_speed:
            scale = min(scale, self.max_angular_speed / abs(omega))
        fastest = max(abs(w) for w in self.inverse(v, omega))
        if fastest > self.wheel_max_speed:
            scale = min(scale, self.wheel_max_speed / fastest)
        return v * scale, omega * scale

    def integrate(self, pose, v, omega, dt):
        """Advance pose (x, y, theta) by body velocities over dt (exact arc)."""
        x, y, theta = pose
        if abs(omega) < 1e-12:
            return x + v * dt * math.cos(theta), y + v * dt * math.sin(theta), theta
        radius = v / omega
        new_theta = theta + omega * dt
        return (x + radius * (math.sin(new_theta) - math.sin(theta)),
                y - radius * (math.cos(new_theta) - math.cos(theta)),
                new_theta)
