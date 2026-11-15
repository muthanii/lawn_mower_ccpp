import math

import pytest
from shapely.geometry import LineString

from lawn_mower_navigation.lawns import DEMO, find_lawn
from lawn_mower_planning.experiment import CHOSEN_DIRECTION, METHODS
from lawn_mower_navigation.plan import (default_clearance, plan_mission, plan_segments,
                                        segments_coverage, turning_radius)
from lawn_mower_planning.footprint import heading
from lawn_mower_planning.maps import shared_lawns

BUDGET = 300  # evaluated orders; the demo lawn has few cells


@pytest.fixture(scope='module')
def mission(robot):
    return plan_mission(DEMO, robot, budget=BUDGET, cruise_speed=0.6, turn_rate=0.6,
                        pattern='lanes')


def test_find_lawn():
    assert find_lawn('demo') is DEMO
    assert find_lawn('two_beds').name == 'two_beds'
    with pytest.raises(KeyError):
        find_lawn('nowhere')


def test_clearance_keeps_the_turning_body_inside(robot):
    g = robot.geometry
    assert turning_radius(robot) == pytest.approx(math.hypot(g.body_length, g.body_width) / 2)
    assert default_clearance(robot) > turning_radius(robot)


def test_segments_are_one_connected_path_from_the_start(mission):
    segs = mission.segments
    assert segs[0].start == pytest.approx(DEMO.start[:2])
    for a, b in zip(segs, segs[1:]):
        assert a.end == pytest.approx(b.start)


def test_segments_match_the_planner_path(mission):
    planned = LineString(mission.problem.path(mission.plan))
    assert LineString(mission.points).length == pytest.approx(planned.length, rel=1e-6)
    assert mission.predicted['length'] == pytest.approx(planned.length, rel=1e-6)


def test_path_keeps_the_clearance(mission, robot):
    path = LineString(mission.points)
    assert DEMO.area.boundary.distance(path) >= mission.clearance - 1e-6
    assert DEMO.area.contains(path)


def test_lanes_mow_and_the_first_move_does_not(mission):
    assert not mission.segments[0].blade and not mission.segments[0].lane
    assert all(s.blade for s in mission.segments if s.lane)
    lanes = sum(s.length for s in mission.segments if s.lane)
    assert lanes > 0.7 * mission.predicted['length']


def test_blade_stays_on_over_short_moves_only(mission):
    always = plan_segments(mission.problem, mission.plan, blade_off_move=math.inf)
    never = plan_segments(mission.problem, mission.plan, blade_off_move=0.0)
    assert all(s.blade for s in always[1:] if not s.lane) or len(mission.plan.order) == 1
    assert not any(s.blade for s in never if not s.lane)


def test_width_is_the_cutting_width(mission, robot):
    assert mission.width == robot.mower.cutting_width
    assert 0 < mission.planned_coverage['coverage'] <= 100


def test_unknown_method(robot):
    with pytest.raises(ValueError, match='unknown method'):
        plan_mission(DEMO, robot, method='magic')


@pytest.mark.parametrize('method', sorted(METHODS))
def test_every_method_plans_the_demo_lawn(robot, method):
    # The methods that pay for the direction choice need more than it costs.
    budget = None if CHOSEN_DIRECTION.get(method) else BUDGET
    mission = plan_mission(DEMO, robot, method, budget=budget, pattern='lanes')
    assert mission.method == method and mission.segments
    assert mission.planned_coverage['coverage'] > 50


def test_a_budget_below_the_direction_choice_is_refused(robot):
    with pytest.raises(ValueError, match='larger budget'):
        plan_mission(DEMO, robot, 'energy_direction_ga', budget=BUDGET)


@pytest.fixture(scope='module')
def hybrid(robot):
    return plan_mission(DEMO, robot, budget=BUDGET, cruise_speed=0.6, turn_rate=0.6)


def body_problems(mission, start_heading):
    """Pivots and runs of the mission where the body does not fit."""
    problems = []
    last = start_heading
    for i, s in enumerate(mission.segments):
        h = heading(s.start, s.end)
        if not mission.pivot_check.pivot(s.start, last, h):
            problems.append(f'pivot before segment {i} at {s.start}')
        # A run along an edge keeps the full margin; a manoeuvre the smaller one.
        if not mission.run_check.run(s.start, s.end) and not mission.pivot_check.run(s.start, s.end):
            problems.append(f'run of segment {i} from {s.start}')
        last = h
    return problems


def test_hybrid_is_the_default_and_starts_with_the_lanes(mission, hybrid):
    assert hybrid.pattern == 'hybrid' and mission.pattern == 'lanes'
    assert hybrid.segments[:len(mission.segments)] == mission.segments
    assert len(hybrid.segments) > len(mission.segments)


def test_hybrid_covers_more_than_the_lanes(mission, hybrid):
    assert hybrid.planned_coverage['coverage'] > mission.planned_coverage['coverage'] + 10
    assert hybrid.predicted['length'] > mission.predicted['length']
    assert hybrid.predicted['energy'] > mission.predicted['energy']
    assert hybrid.predicted['length'] == pytest.approx(
        sum(s.length for s in hybrid.segments), rel=1e-6)


def test_hybrid_path_is_connected_and_the_body_fits(hybrid):
    for a, b in zip(hybrid.segments, hybrid.segments[1:]):
        assert a.end == pytest.approx(b.start)
    assert body_problems(hybrid, DEMO.start[2]) == []


def test_contour_runs_are_closer_to_the_edge_than_the_free_area(hybrid, robot):
    edge = DEMO.area.exterior
    nearest = min(edge.distance(LineString([s.start, s.end])) for s in hybrid.segments if s.lane)
    assert nearest == pytest.approx(robot.geometry.body_width / 2 + 0.15, abs=1e-6)
    assert nearest < hybrid.clearance


def test_segments_coverage_matches_the_planner_on_lanes_only(mission):
    lanes = [s for s in mission.segments if s.lane]
    ours = segments_coverage(lanes, DEMO.area, mission.width)['coverage']
    assert ours <= mission.planned_coverage['coverage'] + 1e-6
    assert ours > mission.planned_coverage['coverage'] - 5


def test_way_back_checks_the_body(hybrid):
    assert hybrid.way_back((3.0, 4.0, 0.0), (2.0, 4.0), 1.57)
    assert not hybrid.way_back((3.0, 4.0, 0.0), (5.9, 4.0), 0.0)     # into the bed
    assert not hybrid.way_back((3.0, 4.0, 0.0), (0.66, 4.0), 0.0)    # a turn too close to the edge


@pytest.mark.parametrize('lawn', shared_lawns(), ids=lambda lawn: lawn.name)
def test_hybrid_fits_and_gains_on_every_shared_lawn(robot, lawn):
    lanes = plan_mission(lawn, robot, budget=BUDGET, pattern='lanes')
    hybrid = plan_mission(lawn, robot, budget=BUDGET)
    assert body_problems(hybrid, lawn.start[2]) == []
    for a, b in zip(hybrid.segments, hybrid.segments[1:]):
        assert a.end == pytest.approx(b.start)
    assert hybrid.planned_coverage['coverage'] > lanes.planned_coverage['coverage'] + 2


def test_unknown_pattern(robot):
    with pytest.raises(ValueError, match='unknown pattern'):
        plan_mission(DEMO, robot, pattern='spiral')
