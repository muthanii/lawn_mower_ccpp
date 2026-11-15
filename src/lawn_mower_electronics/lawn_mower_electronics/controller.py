"""The real-time controller between the radio receiver and the actuators.

It replaces the direct wires from the receiver to the motor driver and to the
blade relay. On every cycle it reads the receiver, the velocity command from
the companion computer and the emergency stop, and writes one pulse for each
side of the motor driver and the state of the blade relay.

States, from the highest priority down:

    ESTOP     the emergency stop is pressed or latched: drive neutral, blade off.
              It stays latched until the stop is released and the mode switch
              is put back to manual.
    FAILSAFE  the receiver lost the transmitter: drive neutral, blade off. The
              robot does not drive itself without an operator in radio range.
    MANUAL    the mode switch is low: the sticks go straight to the driver and
              the blade follows the blade switch, as before the retrofit.
    AUTO      the mode switch is high: the velocity command sets the drive.
              The blade runs when the companion asks for it and the operator's
              blade switch allows it. A command older than command_timeout
              stops the drive and the blade.
"""

from dataclasses import dataclass

from .rc import Switch, channel, unit_to_pulse

ESTOP = 'ESTOP'
FAILSAFE = 'FAILSAFE'
MANUAL = 'MANUAL'
AUTO = 'AUTO'


@dataclass(frozen=True)
class Outputs:
    left: int      # us, pulse to the left side of the motor driver
    right: int     # us
    blade: bool    # blade relay closed
    state: str


class Controller:

    def __init__(self, electronics, kinematics):
        self.rc = electronics.rc
        self.cfg = electronics.controller
        self.kin = kinematics
        self.mode = Switch(self.cfg.switch_low, self.cfg.switch_high)
        self.blade = Switch(self.cfg.switch_low, self.cfg.switch_high)
        self.latched = False
        self.command = None  # (time, v, omega)
        self.blade_request = False

    def set_command(self, t, v, omega):
        """Velocity command from the companion computer, received at time t."""
        self.command = (t, v, omega)

    def set_blade_request(self, on):
        self.blade_request = bool(on)

    def _neutral(self, state):
        return Outputs(self.rc.pulse_mid, self.rc.pulse_mid, False, state)

    def twist_to_pulses(self, v, omega):
        """Body velocity -> (left, right) pulses, full pulse = no-load wheel speed."""
        v, omega = self.kin.saturate(v, omega)
        left, right = self.kin.inverse(v, omega)
        top = self.kin.wheel_max_speed
        return unit_to_pulse(left / top, self.rc), unit_to_pulse(right / top, self.rc)

    def step(self, t, channels, failsafe, estop):
        """One control cycle at time t [s]. Returns the Outputs."""
        ch = self.rc.channels
        # A lost link is not a switch position: keep the switches where they were.
        if not failsafe:
            auto = self.mode.update(channel(channels, ch.mode))
            blade_allowed = self.blade.update(channel(channels, ch.blade))
        else:
            auto, blade_allowed = self.mode.on, self.blade.on

        if estop:
            self.latched = True
        elif self.latched and not failsafe and not auto:
            self.latched = False
        if self.latched:
            return self._neutral(ESTOP)
        if failsafe:
            return self._neutral(FAILSAFE)
        if not auto:
            return Outputs(channel(channels, ch.left), channel(channels, ch.right),
                           blade_allowed, MANUAL)

        fresh = self.command is not None and t - self.command[0] <= self.cfg.command_timeout
        if not fresh:
            return self._neutral(AUTO)
        left, right = self.twist_to_pulses(self.command[1], self.command[2])
        return Outputs(left, right, blade_allowed and self.blade_request, AUTO)
