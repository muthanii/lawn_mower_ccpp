"""Score a simulated mission from the ground truth of the simulator.

    coverage   mowed area inside the lawn / lawn area, where the mowed area is
               the driven path with the blade cutting, widened by the cutting
               width piece by piece. It is the planner's coverage measure on
               the real path.
    overlap    area mowed more than once / mowed area
    footprint  how far the body left the lawn or entered an obstacle
    tracking   distance of the robot centre from the planned path
"""

import math
from dataclasses import dataclass

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

MIN_STEP = 0.02       # m between trajectory samples that are kept
OVERLAP_STEP = 0.10   # m, length of the pieces whose swaths are compared for overlap
MIN_TURN = math.radians(2.0)  # heading change between samples that are kept
FOOTPRINT_STEP = 0.10  # m between poses whose footprint is checked


@dataclass(frozen=True)
class Sample:
    t: float        # s
    x: float        # m
    y: float
    heading: float  # rad
    cutting: bool   # the blade was fast enough to cut


def thin(samples, step=MIN_STEP, keep_turns=True):
    """Drop samples closer than `step` to the last kept one.

    A sample is also kept when the blade state changes, at the end, and, with
    keep_turns, when the heading has changed by MIN_TURN, so that a turn in
    place is not lost.
    """
    kept = []
    for s in samples:
        turned = keep_turns and kept and abs(math.atan2(
            math.sin(s.heading - kept[-1].heading),
            math.cos(s.heading - kept[-1].heading))) >= MIN_TURN
        if (not kept or s.cutting != kept[-1].cutting or turned
                or math.hypot(s.x - kept[-1].x, s.y - kept[-1].y) >= step):
            kept.append(s)
    if samples and kept[-1] is not samples[-1]:
        kept.append(samples[-1])
    return kept


def path_length(samples):
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(samples, samples[1:]))


def piece_swaths(samples, width):
    """The swath of each piece between two samples driven with the blade cutting."""
    return [LineString([(a.x, a.y), (b.x, b.y)]).buffer(width / 2, cap_style='flat')
            for a, b in zip(samples, samples[1:])
            if a.cutting and b.cutting and math.hypot(b.x - a.x, b.y - a.y) > 0]


def coverage(samples, lawn, width):
    """Coverage and overlap [%] of the lawn polygon by the driven path.

    Both use the swaths of the short pieces of the path driven with the blade
    on, as the planner does for its segments. The mowed area is their union;
    the overlap is the area they share, so a stretch that crosses or retraces
    itself counts.
    """
    pieces = piece_swaths(thin(samples), width)
    if not pieces:
        return {'coverage': 0.0, 'overlap': 0.0, 'mowed_area': 0.0}
    mowed = unary_union(pieces)
    # Longer pieces for the overlap, so that the jitter of a turn in place does not count.
    coarse = piece_swaths(thin(samples, OVERLAP_STEP, keep_turns=False), width) or pieces
    swept = unary_union(coarse)
    repeated = sum(p.area for p in coarse) - swept.area
    return {'coverage': 100 * mowed.intersection(lawn).area / lawn.area,
            'overlap': 100 * repeated / swept.area,
            'mowed_area': mowed.intersection(lawn).area}


def footprint(sample, length, width):
    """The body rectangle at a pose."""
    c, s = math.cos(sample.heading), math.sin(sample.heading)
    corners = [(sx * length / 2, sy * width / 2) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    return Polygon([(sample.x + c * px - s * py, sample.y + s * px + c * py) for px, py in corners])


def footprint_excursion(samples, lawn, length, width):
    """Largest distance [m] that a body corner was outside the lawn or inside an obstacle."""
    worst = 0.0
    for s in thin(samples, FOOTPRINT_STEP):
        body = footprint(s, length, width)
        if lawn.contains(body):
            continue
        outside = body.difference(lawn)
        worst = max(worst, max(lawn.boundary.distance(Point(p))
                               for g in getattr(outside, 'geoms', [outside])
                               for p in g.exterior.coords))
    return worst


def tracking_error(samples, path_points):
    """(mean, max) distance [m] of the robot centre from the planned path."""
    if len(path_points) < 2 or not samples:
        return 0.0, 0.0
    path = LineString(path_points)
    errors = [path.distance(Point(s.x, s.y)) for s in thin(samples)]
    return sum(errors) / len(errors), max(errors)
