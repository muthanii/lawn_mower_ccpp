"""Plan a coverage mission: lawn and robot in, tracker segments out.

Calls the pipeline of lawn_mower_planning (cells, lanes, transitions, cell
ordering) with the robot's cutting width, and a clearance that keeps the whole
body inside the lawn, also while it turns in place.

Two patterns:

    lanes   boustrophedon lanes in the free area only.
    hybrid  the same lanes, then contour passes along the lawn edge and round
            the obstacles (lawn_mower_planning.contour): a ring on the
            boundary of the free area, and a closer ring where the body fits
            only while it drives along the edge.
"""

import math
import time
from dataclasses import dataclass, replace

from shapely import affinity
from shapely.geometry import LineString
from shapely.ops import unary_union

from lawn_mower_planning.contour import order_passes, side_ring_passes, turn_ring_passes
from lawn_mower_planning.direction import choose_direction
from lawn_mower_planning.experiment import (CHOSEN_DIRECTION, METHODS, SWEEP_ANGLES_DEG,
                                            Settings)
from lawn_mower_planning.footprint import BodyCheck, Footprint, body_area, heading
from lawn_mower_planning.path_cost import PathCost, clean
from lawn_mower_planning.problem import RIGHT_ANGLE, build_problem
from lawn_mower_planning.roadmap import Roadmap
from lawn_mower_robot.energy import SkidSteerEnergyModel

from .tracker import Segment

DEFAULT_METHOD = 'energy_ga'
DEFAULT_MARGIN = 0.10        # m between the turning body and the lawn edge or an obstacle
DEFAULT_BLADE_OFF_MOVE = 3.0  # m, the blade stays on over shorter moves between cells
PATTERNS = ('hybrid', 'lanes')
DEFAULT_PATTERN = PATTERNS[0]


@dataclass(frozen=True)
class Margins:
    """Room the body keeps [m]. The lawn edge is a line on the ground; an
    obstacle is solid and the bumper does not cover the wheels, so it gets more."""

    run_edge: float = 0.15        # while driving straight, to the lawn edge
    run_obstacle: float = 0.25    # while driving straight, to an obstacle
    pivot_edge: float = 0.05      # at the worst moment of a turn in place, to the lawn edge
    pivot_obstacle: float = 0.10  # at the worst moment of a turn in place, to an obstacle


def turning_radius(robot):
    """Radius of the circle the body sweeps when it turns in place [m]."""
    return math.hypot(robot.geometry.body_length, robot.geometry.body_width) / 2


def default_clearance(robot, margin=DEFAULT_MARGIN):
    """Distance the robot centre keeps from the lawn edge and from obstacles [m]."""
    return turning_radius(robot) + margin


@dataclass(frozen=True)
class Mission:
    lawn: object          # lawn_mower_planning.maps.Lawn
    problem: object       # CoverageProblem
    plan: object          # Plan: cell order and variants
    segments: list        # tracker Segments, world frame
    width: float          # cutting width [m]
    clearance: float      # [m]
    method: str
    planning_seconds: float
    pattern: str = 'lanes'
    predicted: dict = None       # planned length [m], turns and energy [J] of the whole path
    planned_coverage: dict = None  # coverage and overlap [%] if the robot follows the path exactly
    run_check: object = None     # BodyCheck for straight runs
    pivot_check: object = None   # BodyCheck for turns in place

    @property
    def free(self):
        """World-frame area the robot centre may use: the lawn less the clearance."""
        return self.lawn.area.buffer(-self.clearance, join_style='mitre')

    @property
    def points(self):
        """World-frame waypoints of the path."""
        return [self.segments[0].start] + [s.end for s in self.segments] if self.segments else []

    def way_back(self, pose, target, final_heading):
        """The body fits when the robot turns at pose (x, y, heading), drives
        straight to target and turns there to final_heading."""
        line = heading(pose[:2], target)
        return (self.pivot_check.pivot(pose[:2], pose[2], line)
                and self.run_check.run(pose[:2], target)
                and self.pivot_check.pivot(target, line, final_heading))


def _to_world(problem, points):
    line = affinity.rotate(LineString(points), -(RIGHT_ANGLE - problem.lane_angle),
                           origin=(0, 0), use_radians=True)
    return list(line.coords)


def plan_segments(problem, plan, blade_off_move=DEFAULT_BLADE_OFF_MOVE):
    """Tracker segments of a plan, in the world frame.

    Lane segments mow. A move between cells, or from the start, keeps the
    blade on when it is shorter than blade_off_move, so that the blade motor
    is not switched for every short gap. The first move, from the start pose,
    always has the blade off.
    """
    pieces = []   # (sweep-frame points, lane, blade)
    prev = (None, None)
    for c, a in zip(plan.order, plan.variants):
        link = problem.links.get((*prev, c, a))
        if link is None and prev != (None, None):
            raise ValueError(f'cell {c} cannot be reached from cell {prev[0]}')
        if link:
            length = LineString(link).length if len(link) > 1 else 0.0
            pieces.append((link, False, prev != (None, None) and length < blade_off_move))
        pieces.append((problem.routes[c][a], True, True))
        prev = (c, a)

    segments = []
    last = None
    for points, lane, blade in pieces:
        world = _to_world(problem, points) if len(points) > 1 else []
        if last is not None and world:
            world = [last] + world   # joins the pieces; clean() drops it when it repeats
        world = clean(world) if world else []
        for p, q in zip(world, world[1:]):
            segments.append(Segment(tuple(p), tuple(q), blade, lane))
        if world:
            last = world[-1]
    return segments


def segments_coverage(segments, lawn, width):
    """Coverage and overlap [%] of the lawn by the segments driven with the blade on."""
    strips = [LineString([s.start, s.end]).buffer(width / 2, cap_style='flat')
              for s in segments if s.blade]
    if not strips:
        return {'coverage': 0.0, 'overlap': 0.0}
    swath = unary_union(strips)
    return {'coverage': 100 * swath.intersection(lawn).area / lawn.area,
            'overlap': 100 * (sum(s.area for s in strips) - swath.area) / swath.area}


def contour_segments(lawn, free, footprint, margins, start, blade_off_move):
    """Tracker segments of the contour passes, from `start` (where the lanes end)."""
    run_check = BodyCheck(footprint, body_area(lawn, margins.run_edge, margins.run_obstacle))
    pivot_check = BodyCheck(footprint,
                            body_area(lawn, margins.pivot_edge, margins.pivot_obstacle))
    side = run_check.area.buffer(-footprint.width / 2, join_style='mitre')
    passes = turn_ring_passes(free) + side_ring_passes(side, free, run_check, pivot_check)
    segments = []
    for link, route in order_passes(passes, start, Roadmap(free)):
        link = clean(link)
        length = LineString(link).length if len(link) > 1 else 0.0
        segments += [Segment(p, q, length < blade_off_move) for p, q in zip(link, link[1:])]
        segments += [Segment(p, q, True, True) for p, q in zip(route, route[1:])]
    return segments


def plan_mission(lawn, robot, method=DEFAULT_METHOD, seed=0, cruise_speed=None, turn_rate=None,
                 clearance=None, budget=None, sweep_angles_deg=SWEEP_ANGLES_DEG,
                 blade_off_move=DEFAULT_BLADE_OFF_MOVE, pattern=DEFAULT_PATTERN,
                 margins=Margins()):
    """Plan the coverage of `lawn` (a maps.Lawn) with `robot` (a RobotConfig).

    cruise_speed [m/s] and turn_rate [rad/s] are the speeds the tracker will
    drive at; the energy of the plan is predicted at these speeds. They default
    to the robot's limits.
    """
    if method not in METHODS:
        raise ValueError(f'unknown method {method!r}; have {sorted(METHODS)}')
    if pattern not in PATTERNS:
        raise ValueError(f'unknown pattern {pattern!r}; have {PATTERNS}')
    width = robot.mower.cutting_width
    clearance = default_clearance(robot) if clearance is None else clearance
    path_cost = PathCost(SkidSteerEnergyModel.from_config(robot),
                         cruise_speed or robot.limits.max_linear_speed,
                         turn_rate or robot.limits.max_angular_speed)
    settings = Settings() if budget is None else Settings(budget)
    angles = tuple(math.radians(a) for a in sweep_angles_deg)

    t0 = time.perf_counter()
    if method in CHOSEN_DIRECTION:
        screen = replace(settings.ga, evaluations=settings.screen_evaluations)
        choice = choose_direction(lawn.area, width, path_cost, lawn.start, angles,
                                  clearance=clearance, limit=settings.directions, screen=screen)
        problem = choice.problem
        spent = choice.evaluations if CHOSEN_DIRECTION[method] else 0
        if spent >= settings.budget:
            raise ValueError(f'the direction choice of {method} uses the whole budget '
                             f'({spent} of {settings.budget} evaluations); use a larger budget')
        result = METHODS[method](problem, settings, seed, spent) if spent \
            else METHODS[method](problem, settings, seed)
    else:
        problem = build_problem(lawn.area, width, path_cost, clearance=clearance,
                                start=lawn.start, sweep_angles=angles)
        result = METHODS[method](problem, settings, seed)
    if isinstance(result, str):
        raise ValueError(f'{method} cannot plan this lawn: {result}')
    if not result.plan.feasible:
        raise ValueError(f'{method} found no feasible plan for {lawn.name}')
    plan = result.plan
    segments = plan_segments(problem, plan, blade_off_move)
    predicted = dict(plan.metrics)
    coverage = problem.coverage(plan)
    footprint = Footprint(robot.geometry.body_length, robot.geometry.body_width)
    area = lawn.area
    if pattern == 'hybrid' and segments:
        free = area.buffer(-clearance, join_style='mitre')
        extra = contour_segments(area, free, footprint, margins, segments[-1].end,
                                 blade_off_move)
        if extra:
            points = [extra[0].start] + [s.end for s in extra]
            for name, value in path_cost(points, segments[-1].heading).items():
                predicted[name] += value
            segments = segments + extra
            coverage = segments_coverage(segments, area, width)
    return Mission(lawn, problem, plan, segments, width, clearance, method,
                   time.perf_counter() - t0, pattern, predicted, coverage,
                   BodyCheck(footprint, body_area(area, margins.run_edge, margins.run_obstacle)),
                   BodyCheck(footprint,
                             body_area(area, margins.pivot_edge, margins.pivot_obstacle)))
