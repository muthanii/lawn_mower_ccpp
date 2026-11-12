"""Energy-chosen decomposition direction.

The BCD sweep direction fixes the cells and the lane heading that the cells
start from, and through the number of lanes it fixes most of the turns. On a
skid-steer robot the turns are a large part of the energy, so the direction
matters more than the cell order does. This module chooses it by energy:

1. Candidates. The lane headings parallel to an edge of the lawn boundary or
   of an obstacle (Huang 2001: a sweep that minimises the number of lanes of
   a polygon is parallel to one of its edges). The candidates are ranked by
   the total length of the edges they are parallel to, and the first `limit`
   are kept. The default heading (along y) is always a candidate.
2. Score. Each candidate is decomposed and scored with the energy of the
   best plan of a short GA run (energy_ga_order with `screen` evaluations and
   a fixed seed).
3. Choice. The candidate with the least energy; a tie keeps the earlier one.

The choice is deterministic for a lawn, and it costs `screen` evaluated orders
per candidate. A method that uses it must take these from its budget
(DirectionChoice.evaluations). Every ordering method can plan on the chosen
problem, so the gain from the direction and the gain from the ordering can be
reported separately.
"""

import math
import time
from dataclasses import dataclass

from .energy_order import EnergyGaParams, energy_ga_order
from .problem import RIGHT_ANGLE, build_problem

CANDIDATE_LIMIT = 8
SCREEN_EVALUATIONS = 500  # evaluated orders to score one candidate
SCREEN_SEED = 0
ANGLE_DIGITS = 6      # headings closer than 1e-6 rad are one direction
MIN_EDGE = 1e-9       # m, shorter edges have no direction


@dataclass(frozen=True)
class DirectionChoice:
    problem: object      # the CoverageProblem of the chosen direction
    lane_angle: float    # its world lane heading [rad], in [0, pi)
    scores: tuple        # ((lane_angle, screening energy or inf), ...) per candidate, in order
    evaluations: int     # orders evaluated to make the choice
    seconds: float       # time to decompose and score the candidates


def candidate_lane_angles(lawn, limit=CANDIDATE_LIMIT):
    """Lane headings [rad, in [0, pi)] parallel to the lawn's edges, longest first.

    The default heading pi / 2 is always first. limit caps the number of
    candidates, the default included.
    """
    if limit < 1:
        raise ValueError('limit must be >= 1')
    weight = {}
    for ring in (lawn.exterior, *lawn.interiors):
        coords = list(ring.coords)
        for p, q in zip(coords, coords[1:]):
            length = math.dist(p, q)
            if length <= MIN_EDGE:
                continue
            angle = round(math.atan2(q[1] - p[1], q[0] - p[0]) % math.pi, ANGLE_DIGITS)
            if angle >= round(math.pi, ANGLE_DIGITS):
                angle = 0.0
            weight[angle] = weight.get(angle, 0.0) + length
    default = round(RIGHT_ANGLE, ANGLE_DIGITS)
    weight.pop(default, None)
    ranked = sorted(weight, key=lambda a: (-weight[a], a))
    return [RIGHT_ANGLE, *ranked][:limit]


def choose_direction(lawn, width, path_cost, start=None, sweep_angles=(0.0,), clearance=None,
                     limit=CANDIDATE_LIMIT, screen=EnergyGaParams(evaluations=SCREEN_EVALUATIONS),
                     default=None):
    """Decompose the lawn along each candidate heading and keep the least-energy one.

    screen   GA parameters of the scoring run; its weights are the objective
             (energy by default) and its evaluations the cost per candidate
    default  the already built problem of the default heading (pi / 2), to
             save building it again
    The other arguments are those of build_problem. Returns a DirectionChoice.
    """
    t0 = time.perf_counter()
    best, scores, used = None, [], 0
    for angle in candidate_lane_angles(lawn, limit):
        if angle == RIGHT_ANGLE and default is not None:
            problem = default
        else:
            try:
                problem = build_problem(lawn, width, path_cost, clearance, angle, start,
                                        sweep_angles)
            except ValueError:  # the turned decomposition has slivers
                scores.append((angle, math.inf))
                continue
        result = energy_ga_order(problem.table, screen, SCREEN_SEED)
        used += result.evaluations
        scores.append((angle, result.plan.cost))
        if result.plan.feasible and (best is None or result.plan.cost < best[0]):
            best = (result.plan.cost, angle, problem)
    if best is None:
        raise ValueError('no candidate direction gives a feasible plan')
    return DirectionChoice(best[2], best[1], tuple(scores), used, time.perf_counter() - t0)
