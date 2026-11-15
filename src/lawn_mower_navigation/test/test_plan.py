import math

import pytest
from shapely.geometry import LineString

from lawn_mower_navigation.lawns import DEMO, find_lawn
from lawn_mower_navigation.plan import (default_clearance, plan_mission, plan_segments,
                                        turning_radius)

BUDGET = 300  # evaluated orders; the demo lawn has few cells


@pytest.fixture(scope='module')
def mission(robot):
    return plan_mission(DEMO, robot, budget=BUDGET, cruise_speed=0.6, turn_rate=0.6)


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
