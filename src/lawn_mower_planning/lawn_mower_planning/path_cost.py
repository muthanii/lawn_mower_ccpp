"""Metrics of a polyline driven with point turns: length, turn count and energy.

The robot drives each straight segment from rest and turns in place at each
vertex (stop-turn-go). One turn is one vertex where the heading changes by
more than TURN_TOLERANCE, so a lane change of two 90 degree turns counts 2.
The same rule counts turns inside cells, on A* transitions and at the
junctions between them, so the turn metric is the same for every method.

Energy comes from the skid-steer model in lawn_mower_robot (paper Sec. 3.3).
"""

import math
from dataclasses import dataclass

TURN_TOLERANCE = math.radians(1.0)  # smaller heading changes are not turns
MIN_SEGMENT = 1e-9                  # m, shorter segments are dropped


def heading(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def heading_change(a, b):
    """|b - a| wrapped to [0, pi]."""
    return abs((b - a + math.pi) % (2 * math.pi) - math.pi)


def clean(points):
    """Drop repeated points."""
    out = [tuple(points[0])]
    for p in points[1:]:
        if math.dist(out[-1], p) > MIN_SEGMENT:
            out.append(tuple(p))
    return out


@dataclass(frozen=True)
class PathCost:
    """Turns a polyline into metrics.

    energy_model is a lawn_mower_robot.energy.SkidSteerEnergyModel, or None to
    leave energy out. speed [m/s] and turn_rate [rad/s] are the operating
    speeds; from_config takes the robot's limits.
    """

    energy_model: object = None
    speed: float = None
    turn_rate: float = None

    @classmethod
    def from_config(cls, cfg):
        from lawn_mower_robot.energy import SkidSteerEnergyModel
        return cls(SkidSteerEnergyModel.from_config(cfg), cfg.limits.max_linear_speed,
                   cfg.limits.max_angular_speed)

    @property
    def metrics(self):
        return ('length', 'turns') if self.energy_model is None else ('length', 'turns', 'energy')

    def segments(self, points, start_heading=None, end_heading=None):
        """Straight lengths and turn angles of a polyline.

        start_heading and end_heading are the robot heading before the first
        point and the heading it must have after the last point (None = free).
        """
        points = clean(points)
        lengths = [math.dist(p, q) for p, q in zip(points, points[1:])]
        headings = [heading(p, q) for p, q in zip(points, points[1:])]
        if start_heading is not None:
            headings.insert(0, start_heading)
        if end_heading is not None:
            headings.append(end_heading)
        turns = [d for d in (heading_change(a, b) for a, b in zip(headings, headings[1:]))
                 if d > TURN_TOLERANCE]
        return lengths, turns

    def __call__(self, points, start_heading=None, end_heading=None):
        lengths, turns = self.segments(points, start_heading, end_heading)
        out = {'length': sum(lengths), 'turns': float(len(turns))}
        if self.energy_model is not None:
            m = self.energy_model
            out['energy'] = (sum(m.straight(d, self.speed).electrical for d in lengths)
                             + sum(m.turn(a, self.turn_rate).electrical for a in turns))
        return out
