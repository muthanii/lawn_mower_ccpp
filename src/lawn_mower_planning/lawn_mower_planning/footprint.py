"""Does the robot's body fit? Checks for a pivot and for a straight run.

The body is a rectangle around the robot centre. A straight run sweeps a
longer rectangle. A pivot (a turn in place) sweeps the corners along arcs, so
a pivot of a few degrees needs much less room than a full turn. The checks
take the area that the body may occupy: the lawn less a margin to its edge
and to each obstacle (body_area).
"""

import math
from dataclasses import dataclass

from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.prepared import prep

from .decomposition import polygons

PIVOT_STEP = math.radians(2.0)  # largest heading step between the checked poses of a pivot
TOLERANCE = 1e-6                # m, so a body that just touches the limit still fits


def heading(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def turn_angle(start, end):
    """Signed shortest turn from heading start to heading end [rad], in (-pi, pi]."""
    return math.atan2(math.sin(end - start), math.cos(end - start))


def body_area(lawn, edge_margin, obstacle_margin):
    """The lawn less edge_margin [m] at its outer edge and obstacle_margin [m] round each hole."""
    parts = []
    for poly in polygons(lawn):
        inner = Polygon(poly.exterior).buffer(-edge_margin, join_style='mitre')
        holes = [Polygon(ring).buffer(obstacle_margin, join_style='mitre')
                 for ring in poly.interiors]
        parts.append(inner.difference(unary_union(holes)) if holes else inner)
    return unary_union(parts)


@dataclass(frozen=True)
class Footprint:
    length: float   # m, along the driving direction
    width: float    # m

    @property
    def turning_radius(self):
        """Radius of the circle the body sweeps in a full turn in place [m]."""
        return math.hypot(self.length, self.width) / 2

    def at(self, point, direction):
        """The body rectangle with its centre at point, heading in direction [rad]."""
        return self._box(point, direction, -self.length / 2, self.length / 2)

    def _box(self, point, direction, back, front):
        c, s = math.cos(direction), math.sin(direction)
        half = self.width / 2
        return Polygon([(point[0] + c * x - s * y, point[1] + s * x + c * y)
                        for x, y in ((front, half), (back, half), (back, -half), (front, -half))])

    def run(self, start, end):
        """The area the body sweeps on a straight run from start to end."""
        return self._box(start, heading(start, end), -self.length / 2,
                         math.dist(start, end) + self.length / 2)


class BodyCheck:
    """Checks runs and pivots of a footprint against the area the body may occupy."""

    def __init__(self, footprint, area):
        self.footprint = footprint
        self.area = area
        self._inside = prep(area.buffer(TOLERANCE))

    def run(self, start, end):
        """The body stays inside on a straight run from start to end."""
        if math.dist(start, end) == 0:
            return True
        return self._inside.covers(self.footprint.run(start, end))

    def pivot(self, point, start, end):
        """The body stays inside on a turn in place at point from heading start to end [rad]."""
        angle = turn_angle(start, end)
        steps = max(1, math.ceil(abs(angle) / PIVOT_STEP))
        return all(self._inside.covers(self.footprint.at(point, start + angle * i / steps))
                   for i in range(steps + 1))

    def path(self, points, start_heading=None):
        """Every run and every pivot of a polyline stays inside.

        Returns the list of problems, empty when the path is fine.
        """
        problems = []
        last = start_heading
        for i, (p, q) in enumerate(zip(points, points[1:])):
            if math.dist(p, q) == 0:
                continue
            h = heading(p, q)
            if last is not None and not self.pivot(p, last, h):
                problems.append(f'pivot at point {i} ({p[0]:.2f}, {p[1]:.2f})')
            if not self.run(p, q):
                problems.append(f'run from point {i} ({p[0]:.2f}, {p[1]:.2f})')
            last = h
        return problems
