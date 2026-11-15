"""Follows the coverage path the way the planner costs it: stop, turn, go.

At each vertex the robot turns in place to the heading of the next segment,
then drives the segment. While it drives, a fuzzy controller steers on the
cross-track error and the heading error, and the speed ramps down before the
end. The blade request follows the segment: on along the lanes, off on long
moves between cells.
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

    def update(self, pose, blade_running=True):
        """Command for the pose (x, y, heading). blade_running gates the mowing segments."""
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
                    rate = min(p.turn_rate,
                               math.sqrt(2 * p.angular_deceleration * abs(heading_error)))
                    rate = max(rate, p.min_turn_rate)
                    return Command(0.0, -math.copysign(rate, heading_error), seg.blade)
                if seg.blade and not blade_running:
                    return Command(0.0, 0.0, True)   # wait for the blade to spin up
                self.phase = DRIVE
            if abs(heading_error) > p.realign_angle:
                self.phase = ALIGN
                continue
            speed = min(p.cruise_speed, math.sqrt(2 * p.linear_deceleration * remaining))
            if abs(heading_error) > p.slow_angle:
                speed *= p.slow_factor
            speed = max(speed, p.min_speed)
            return Command(speed, p.turn_rate * self.steer(cross, heading_error), seg.blade)
        return Command(0.0, 0.0, False, done=True)
