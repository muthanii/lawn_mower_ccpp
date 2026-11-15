"""L-faster dual-channel brushed motor driver, as a behavioural model.

Each side takes an RC pulse. Inside the deadband around pulse_mid the output
is off. Outside it the wheel speed target is proportional to the stick
position, up to the no-load wheel speed, and the speed follows the target
with a first-order lag. The driver stops when it has no power (breaker or
emergency stop open) or no pulse for signal_timeout.
"""

import math

from .rc import pulse_to_unit


class MotorDriver:

    def __init__(self, electronics, wheel_max_speed):
        self.rc = electronics.rc
        self.cfg = electronics.motor_driver
        self.wheel_max_speed = wheel_max_speed
        self.pulses = None
        self.pulse_time = None
        self.speeds = (0.0, 0.0)  # (left, right) wheel speed [rad/s]
        self.time = None

    def set_pulses(self, t, left, right):
        self.pulses = (left, right)
        self.pulse_time = t

    def target(self, pulse):
        """Wheel speed target [rad/s] for one pulse."""
        if abs(pulse - self.rc.pulse_mid) <= self.cfg.deadband:
            return 0.0
        return pulse_to_unit(pulse, self.rc) * self.wheel_max_speed

    def step(self, t, powered=True):
        """Advance to time t [s]. Returns (left, right) wheel speeds [rad/s]."""
        dt = 0.0 if self.time is None else max(t - self.time, 0.0)
        self.time = t
        live = (powered and self.pulse_time is not None
                and t - self.pulse_time <= self.cfg.signal_timeout)
        targets = tuple(self.target(p) for p in self.pulses) if live else (0.0, 0.0)
        if not powered:
            # No supply: the motors coast down through the gearbox; model it as the same lag.
            targets = (0.0, 0.0)
        blend = 1.0 - math.exp(-dt / self.cfg.time_constant)
        self.speeds = tuple(s + (g - s) * blend for s, g in zip(self.speeds, targets))
        return self.speeds
