"""Skid-steer traction and turning energy model (paper Sec. 3.3).

Steady-state mechanical power at body velocities (v, omega), with the weight
m g shared equally by the four wheels at x = +-L/2:

    P_roll = mu_r (m g / 2) (|v_L| + |v_R|)      rolling resistance of each side
    P_turn = mu_l (m g L / 2) |omega|            Coulomb lateral sliding moment
    P_el   = (P_roll + P_turn + max(dE_k/dt, 0)) / eta + P_aux

where v_L, v_R = v -+ omega chi B / 2 are the side ground speeds of the
kinematic model and E_k = (m v^2 + I_z omega^2) / 2. Braking energy is not
recovered. The lateral moment is the Coulomb form of Wong's turning
resistance; Yu et al. (2010) show it falls as the turn radius grows, so this
over-estimates wide turns.

Energies are in joules. All symbols come from the robot config.
"""

from dataclasses import dataclass

from .kinematics import SkidSteerKinematics

STANDARD_GRAVITY = 9.80665  # m/s^2
JOULES_PER_WH = 3600.0


@dataclass(frozen=True)
class EnergyBreakdown:
    rolling: float = 0.0     # J, mechanical
    turning: float = 0.0     # J, mechanical
    kinetic: float = 0.0     # J, mechanical, acceleration only
    auxiliary: float = 0.0   # J, electrical
    efficiency: float = 1.0

    @property
    def mechanical(self):
        return self.rolling + self.turning + self.kinetic

    @property
    def electrical(self):
        """Energy drawn from the battery [J]."""
        return self.mechanical / self.efficiency + self.auxiliary

    @property
    def electrical_wh(self):
        return self.electrical / JOULES_PER_WH

    def __add__(self, other):
        return EnergyBreakdown(self.rolling + other.rolling, self.turning + other.turning,
                               self.kinetic + other.kinetic, self.auxiliary + other.auxiliary,
                               self.efficiency)


@dataclass(frozen=True)
class SkidSteerEnergyModel:
    kinematics: SkidSteerKinematics
    mass: float                  # m [kg]
    yaw_inertia: float           # I_z [kg m^2]
    wheelbase: float             # L [m]
    rolling_resistance: float    # mu_r [-]
    lateral_friction: float      # mu_l [-]
    drivetrain_efficiency: float  # eta [-]
    auxiliary_power: float       # P_aux [W]
    gravity: float = STANDARD_GRAVITY

    @classmethod
    def from_config(cls, cfg):
        return cls(
            kinematics=SkidSteerKinematics.from_config(cfg),
            mass=cfg.inertia.mass,
            yaw_inertia=cfg.inertia.yaw_inertia,
            wheelbase=cfg.geometry.wheelbase,
            rolling_resistance=cfg.terrain.rolling_resistance,
            lateral_friction=cfg.terrain.lateral_friction,
            drivetrain_efficiency=cfg.drive.drivetrain_efficiency,
            auxiliary_power=cfg.electrical.auxiliary_power,
        )

    @property
    def weight(self):
        return self.mass * self.gravity

    @property
    def turning_moment(self):
        """Lateral sliding resistance moment M_r = mu_l m g L / 2 [N m]."""
        return self.lateral_friction * self.weight * self.wheelbase / 2

    def rolling_power(self, v, omega):
        v_left, v_right = self.kinematics.side_ground_speeds(v, omega)
        return self.rolling_resistance * self.weight / 2 * (abs(v_left) + abs(v_right))

    def turning_power(self, omega):
        return self.turning_moment * abs(omega)

    def kinetic_energy(self, v, omega):
        return (self.mass * v ** 2 + self.yaw_inertia * omega ** 2) / 2

    def electrical_power(self, v, omega):
        """Steady-state battery power at (v, omega) [W]."""
        mechanical = self.rolling_power(v, omega) + self.turning_power(omega)
        return mechanical / self.drivetrain_efficiency + self.auxiliary_power

    def steady(self, v, omega, duration):
        """Energy of holding (v, omega) for duration, without the speed-up."""
        return EnergyBreakdown(
            rolling=self.rolling_power(v, omega) * duration,
            turning=self.turning_power(omega) * duration,
            auxiliary=self.auxiliary_power * duration,
            efficiency=self.drivetrain_efficiency,
        )

    def straight(self, length, speed):
        """Straight segment of length [m] at speed [m/s], starting from rest."""
        speed = abs(speed)
        e = self.steady(speed, 0.0, length / speed)
        return e + EnergyBreakdown(kinetic=self.kinetic_energy(speed, 0.0),
                                   efficiency=self.drivetrain_efficiency)

    def turn(self, angle, angular_speed, radius=0.0):
        """Arc of |angle| [rad] at |angular_speed| [rad/s] and radius [m] (0 = in place)."""
        omega = abs(angular_speed)
        v = omega * radius
        e = self.steady(v, omega, abs(angle) / omega)
        return e + EnergyBreakdown(kinetic=self.kinetic_energy(v, omega),
                                   efficiency=self.drivetrain_efficiency)


class EnergyIntegrator:
    """Integrates the model over measured body velocities, e.g. from odometry."""

    def __init__(self, model):
        self.model = model
        self.reset()

    def reset(self):
        self.energy = EnergyBreakdown(efficiency=self.model.drivetrain_efficiency)
        self._last = None

    def add(self, t, v, omega):
        """Add a sample at time t [s]; uses the trapezoid rule between samples."""
        m = self.model
        if self._last is not None:
            t0, v0, w0 = self._last
            dt = t - t0
            if dt > 0:
                gain = m.kinetic_energy(v, omega) - m.kinetic_energy(v0, w0)
                self.energy = self.energy + EnergyBreakdown(
                    rolling=(m.rolling_power(v0, w0) + m.rolling_power(v, omega)) / 2 * dt,
                    turning=(m.turning_power(w0) + m.turning_power(omega)) / 2 * dt,
                    kinetic=max(gain, 0.0),
                    auxiliary=m.auxiliary_power * dt,
                    efficiency=m.drivetrain_efficiency,
                )
        self._last = (t, v, omega)
