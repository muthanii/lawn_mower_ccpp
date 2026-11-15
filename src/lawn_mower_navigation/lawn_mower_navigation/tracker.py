"""Follows the coverage path the way the planner costs it: stop, turn, go.

At each vertex the robot turns in place to the heading of the next segment,
then drives the segment. While it drives, a fuzzy controller steers on the
cross-track error and the heading error, and the speed ramps down before the
end. The blade request follows the segment: on along the lanes, off on long
moves between cells.

A skid-steer robot does not turn at the rate it is told to: the ratio depends
on the ground. YawGain learns that ratio from the gyro while the robot
drives, and the tracker divides its turn commands by it.
"""

import math
from dataclasses import dataclass

from .fuzzy import FuzzyTable
from .localization import wrap

ALIGN = 'align'
DRIVE = 'drive'
DONE = 'done'

# Fuzzy steering sets: negative big, negative small, zero, positive small, positive big.
CROSS_TRACK_CENTRES = (-0.30, -0.15, 0.0, 0.15, 0.30)   # m, positive = robot left of the line
HEADING_CENTRES = (-0.40, -0.20, 0.0, 0.20, 0.40)       # rad, positive = robot points left
STEER_LEVELS = (-0.8, -0.4, 0.0, 0.4, 0.8)              # rad/s per unit of turn rate limit


def steering_rules():
    """Steer against the sum of both errors, saturating at the big sets."""
    n = len(STEER_LEVELS)
    mid = n // 2
    return [[STEER_LEVELS[min(max(mid - (i - mid) - (j - mid), 0), n - 1)]
             for j in range(n)] for i in range(n)]


class YawGain:
    """Measured yaw rate per commanded yaw rate, learned while the robot turns.

    Both rates go through the same low-pass filter, so the lag of the drive
    cancels in their ratio. The estimate moves only while the robot is told
    to turn, and stays inside `limits`.
    """

    def __init__(self, time_constant=1.0, min_rate=0.1, limits=(0.4, 2.5)):
        self.time_constant = time_constant
        self.min_rate = min_rate     # rad/s, filtered command below which nothing is learned
        self.limits = limits
        self.value = 1.0
        self._commanded = 0.0
        self._measured = 0.0

    def update(self, commanded, measured, dt):
        """Add the last command and the yaw rate measured after it, dt [s] apart."""
        if dt <= 0:
            return self.value
        blend = min(1.0, dt / self.time_constant)
        self._commanded += (commanded - self._commanded) * blend
        self._measured += (measured - self._measured) * blend
        if abs(self._commanded) >= self.min_rate:
            ratio = self._measured / self._commanded
            self.value = min(max(ratio, self.limits[0]), self.limits[1])
        return self.value


@dataclass(frozen=True)
class Segment:
    start: tuple     # (x, y) [m]
    end: tuple
    blade: bool      # blade on while driving this segment
    lane: bool = False  # part of a cell's coverage route (not a move between cells)

    @property
    def length(self):
        return math.dist(self.start, self.end)

    @property
    def heading(self):
        return math.atan2(self.end[1] - self.start[1], self.end[0] - self.start[0])


@dataclass(frozen=True)
class TrackerParams:
    cruise_speed: float            # m/s on a segment
    turn_rate: float               # rad/s in place, and the steering authority
    linear_deceleration: float     # m/s^2, ramp before the end of a segment
    angular_deceleration: float    # rad/s^2, ramp before the end of a turn
    min_speed: float = 0.08        # m/s, slowest commanded creep
    min_turn_rate: float = 0.12    # rad/s
    position_tolerance: float = 0.03   # m along the segment
    heading_tolerance: float = math.radians(1.5)
    still_rate: float = 0.05           # rad/s, below this the robot counts as not turning
    reaction_time: float = 0.25        # s from a command to the motion changing
    realign_angle: float = math.radians(30)  # heading error that stops and turns in place
    slow_angle: float = math.radians(20)     # heading error with the speed cut to slow_factor
    slow_factor: float = 0.4


@dataclass(frozen=True)
class Command:
    v: float
    omega: float
    blade: bool
    done: bool = False


class PathTracker:

    def __init__(self, segments, params):
        self.segments = [s for s in segments if s.length > params.position_tolerance]
        self.params = params
        self.index = 0
        self.phase = ALIGN if self.segments else DONE
        self.steer = FuzzyTable(CROSS_TRACK_CENTRES, HEADING_CENTRES, steering_rules())
        self.yaw_gain = YawGain()
        self._last_omega = 0.0
        self._last_v = 0.0

    @property
    def done(self):
        return self.phase == DONE

    @property
    def segment(self):
        return None if self.done else self.segments[self.index]

    def hold(self):
        """The robot was stopped or driven by hand: turn onto the segment again."""
        if not self.done:
            self.phase = ALIGN

    def errors(self, pose):
        """(remaining distance, cross-track error, heading error) on the current segment."""
        seg = self.segment
        x, y, heading = pose
        c, s = math.cos(seg.heading), math.sin(seg.heading)
        dx, dy = x - seg.start[0], y - seg.start[1]
        return seg.length - (dx * c + dy * s), -dx * s + dy * c, wrap(heading - seg.heading)

    def update(self, pose, blade_running=True, yaw_rate=None, dt=0.0):
        """Command for the pose (x, y, heading).

        blade_running gates the mowing segments. yaw_rate [rad/s] is the gyro
        reading and dt [s] the time since the last call; with them the turn
        commands are corrected by the learned yaw gain.
        """
        if yaw_rate is not None:
            self.yaw_gain.update(self._last_omega, yaw_rate, dt)
        cmd = self._command(pose, blade_running, yaw_rate or 0.0)
        cmd = Command(cmd.v, cmd.omega / self.yaw_gain.value, cmd.blade, cmd.done)
        self._last_omega, self._last_v = cmd.omega, cmd.v
        return cmd

    def _turn_in_place(self, heading_error, yaw_rate):
        """Yaw rate that brings the heading error to zero without overshoot."""
        p = self.params
        closing = -yaw_rate * math.copysign(1.0, heading_error)   # rate towards the target
        stopping = (max(closing, 0.0) ** 2 / (2 * p.angular_deceleration)
                    + max(closing, 0.0) * p.reaction_time)
        if abs(heading_error) <= stopping:
            return 0.0   # already fast enough to coast onto the heading
        rate = min(p.turn_rate, math.sqrt(2 * p.angular_deceleration * abs(heading_error)))
        return -math.copysign(max(rate, p.min_turn_rate), heading_error)

    def _command(self, pose, blade_running, yaw_rate):
        p = self.params
        while not self.done:
            seg = self.segment
            remaining, cross, heading_error = self.errors(pose)
            if remaining <= p.position_tolerance:
                self.index += 1
                self.phase = ALIGN if self.index < len(self.segments) else DONE
                continue
            if self.phase == ALIGN:
                if abs(heading_error) > p.heading_tolerance:
                    return Command(0.0, self._turn_in_place(heading_error, yaw_rate), seg.blade)
                if abs(yaw_rate) > p.still_rate:
                    return Command(0.0, 0.0, seg.blade)   # let the turn die out
                if seg.blade and not blade_running:
                    return Command(0.0, 0.0, True)   # wait for the blade to spin up
                self.phase = DRIVE
            if abs(heading_error) > p.realign_angle:
                self.phase = ALIGN
                continue
            # Brake early by the distance covered before a command takes effect.
            braking = max(remaining - self._last_v * p.reaction_time, 0.0)
            speed = min(p.cruise_speed, math.sqrt(2 * p.linear_deceleration * braking))
            if abs(heading_error) > p.slow_angle:
                speed *= p.slow_factor
            speed = max(speed, p.min_speed)
            return Command(speed, p.turn_rate * self.steer(cross, heading_error), seg.blade)
        return Command(0.0, 0.0, False, done=True)
