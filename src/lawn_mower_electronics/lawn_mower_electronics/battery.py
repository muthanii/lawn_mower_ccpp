"""Battery pack and the power monitor that reads it.

The pack is a voltage source behind an internal resistance. The monitor
counts the energy of each consumer, so the simulation reports traction, blade
and auxiliary energy apart, and the state of charge from the total.
"""

import math
from dataclasses import dataclass, field

SECONDS_PER_HOUR = 3600.0


@dataclass
class Battery:
    voltage: float               # V, nominal open-circuit voltage
    capacity: float              # A*h
    internal_resistance: float   # ohm
    cutoff_voltage: float        # V
    energy: dict = field(default_factory=dict)  # J drawn, by consumer
    time: float = None
    current: float = 0.0         # A, last step
    terminal_voltage: float = None

    @classmethod
    def from_configs(cls, robot, electronics):
        return cls(robot.battery.voltage, robot.battery.capacity,
                   electronics.battery.internal_resistance, electronics.battery.cutoff_voltage)

    @property
    def stored(self):
        """Energy of the full pack [J]."""
        return self.voltage * self.capacity * SECONDS_PER_HOUR

    @property
    def drawn(self):
        return sum(self.energy.values())

    @property
    def state_of_charge(self):
        return max(0.0, 1.0 - self.drawn / self.stored)

    @property
    def empty(self):
        return self.state_of_charge <= 0.0 or (self.terminal_voltage is not None
                                               and self.terminal_voltage <= self.cutoff_voltage)

    def _current(self, power):
        """Current [A] that delivers `power` [W] at the terminals: V I - R I^2 = P."""
        disc = self.voltage ** 2 - 4 * self.internal_resistance * power
        if self.internal_resistance <= 0 or disc < 0:
            return power / self.voltage
        return (self.voltage - math.sqrt(disc)) / (2 * self.internal_resistance)

    def step(self, t, powers):
        """Advance to time t [s] with `powers` {consumer: W}. Returns the terminal voltage."""
        dt = 0.0 if self.time is None else max(t - self.time, 0.0)
        self.time = t
        for name, power in powers.items():
            self.energy[name] = self.energy.get(name, 0.0) + power * dt
        self.current = self._current(sum(powers.values()))
        self.terminal_voltage = self.voltage - self.internal_resistance * self.current
        return self.terminal_voltage


class PowerMonitor:
    """Turns measured body velocities and the blade power into battery load.

    Traction power is the energy model of lawn_mower_robot (paper Sec. 3.3)
    at the measured velocity, with the kinetic energy gained since the last
    sample. Braking energy is not recovered.
    """

    TRACTION = 'traction'
    BLADE = 'blade'
    AUXILIARY = 'auxiliary'

    def __init__(self, model, battery):
        self.model = model
        self.battery = battery
        self._last = None

    def traction_power(self, t, v, omega):
        """Battery power for traction [W] at a velocity sample taken at time t [s]."""
        m = self.model
        mechanical = m.rolling_power(v, omega) + m.turning_power(omega)
        if self._last is not None and t > self._last[0]:
            gain = m.kinetic_energy(v, omega) - m.kinetic_energy(*self._last[1:])
            mechanical += max(gain, 0.0) / (t - self._last[0])
        self._last = (t, v, omega)
        return mechanical / m.drivetrain_efficiency

    def update(self, t, v, omega, blade_power):
        """Add a sample. Returns the battery terminal voltage [V]."""
        return self.battery.step(t, {
            self.TRACTION: self.traction_power(t, v, omega),
            self.BLADE: blade_power,
            self.AUXILIARY: self.model.auxiliary_power,
        })
