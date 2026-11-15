"""Plan a coverage mission: lawn and robot in, tracker segments out.

Calls the pipeline of lawn_mower_planning (cells, lanes, transitions, cell
ordering) with the robot's cutting width, and a clearance that keeps the whole
body inside the lawn, also while it turns in place.
"""

import math
import time
from dataclasses import dataclass, replace

from shapely import affinity
from shapely.geometry import LineString

from lawn_mower_planning.direction import choose_direction
from lawn_mower_planning.experiment import (CHOSEN_DIRECTION, METHODS, SWEEP_ANGLES_DEG,
                                            Settings)
from lawn_mower_planning.path_cost import PathCost, clean
from lawn_mower_planning.problem import RIGHT_ANGLE, build_problem
from lawn_mower_robot.energy import SkidSteerEnergyModel

from .tracker import Segment

DEFAULT_METHOD = 'energy_ga'
DEFAULT_MARGIN = 0.10        # m between the turning body and the lawn edge or an obstacle
DEFAULT_BLADE_OFF_MOVE = 3.0  # m, the blade stays on over shorter moves between cells


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

    @property
    def points(self):
        """World-frame waypoints of the path."""
        return [self.segments[0].start] + [s.end for s in self.segments] if self.segments else []

    @property
    def predicted(self):
        """The planner's metrics of the plan (length, turns, energy [J])."""
        return dict(self.plan.metrics)

    @property
    def planned_coverage(self):
        """Coverage and overlap [%] if the robot follows the path exactly."""
        return self.problem.coverage(self.plan)


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


def plan_mission(lawn, robot, method=DEFAULT_METHOD, seed=0, cruise_speed=None, turn_rate=None,
                 clearance=None, budget=None, sweep_angles_deg=SWEEP_ANGLES_DEG,
                 blade_off_move=DEFAULT_BLADE_OFF_MOVE):
    """Plan the coverage of `lawn` (a maps.Lawn) with `robot` (a RobotConfig).

    cruise_speed [m/s] and turn_rate [rad/s] are the speeds the tracker will
    drive at; the energy of the plan is predicted at these speeds. They default
    to the robot's limits.
    """
    if method not in METHODS:
        raise ValueError(f'unknown method {method!r}; have {sorted(METHODS)}')
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
    return Mission(lawn, problem, result.plan,
                   plan_segments(problem, result.plan, blade_off_move), width, clearance,
                   method, time.perf_counter() - t0)
