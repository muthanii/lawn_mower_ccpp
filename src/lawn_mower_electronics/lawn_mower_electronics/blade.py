"""Blade relay and blade motor, as a behavioural model.

The relay chain of the circuit diagram (12 V relay -> 70 A relay -> 40 A fuse)
is one switch here. The motor speed follows the switch with a first-order
lag. The blade cuts once it is above running_fraction of the rated speed.
Electrical power scales with speed: idle_power_fraction of the cutting power
when the blade spins free, and all of it when the blade is in grass.
"""

import math


class Blade:

    def __init__(self, electronics, cutting_power):
        self.cfg = electronics.blade
        self.cutting_power = cutting_power or 0.0  # W, electrical, while cutting
        self.fraction = 0.0                        # speed / rated speed
        self.time = None

    @property
    def speed(self):
        """Blade speed [rpm]."""
        return self.fraction * self.cfg.rated_speed

    @property
    def running(self):
        return self.fraction >= self.cfg.running_fraction

    def power(self, cutting=True):
        """Electrical power [W] at the present speed."""
        share = 1.0 if cutting else self.cfg.idle_power_fraction
        return self.cutting_power * share * self.fraction

    def step(self, t, relay, powered=True):
        """Advance to time t [s] with the relay closed or open. Returns the speed fraction."""
        dt = 0.0 if self.time is None else max(t - self.time, 0.0)
        self.time = t
        goal = 1.0 if relay and powered else 0.0
        self.fraction += (goal - self.fraction) * (1.0 - math.exp(-dt / self.cfg.time_constant))
        return self.fraction
