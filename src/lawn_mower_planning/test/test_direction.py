import math

import pytest
from shapely import affinity
from shapely.geometry import LineString, box

from lawn_mower_planning.decomposition import boustrophedon_cells
from lawn_mower_planning.direction import candidate_lane_angles, choose_direction
from lawn_mower_planning.energy_order import EnergyGaParams
from lawn_mower_planning.lanes import monotone
from lawn_mower_planning.maps import DEFAULT_WIDTH, shared_lawns
from lawn_mower_planning.path_cost import PathCost
from lawn_mower_planning.problem import RIGHT_ANGLE, build_problem

SCREEN = EnergyGaParams(evaluations=60, population=20)


@pytest.fixture(scope='module')
def energy_cost():
    from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
    return PathCost.from_config(load_robot_config(resolve_robot_config('pioneer3at')))


def test_candidates_are_the_edge_directions_with_the_default_first():
    wide = box(0, 0, 20, 10).difference(box(5, 3, 8, 6))
    assert candidate_lane_angles(wide) == [RIGHT_ANGLE, 0.0]
    assert candidate_lane_angles(wide, limit=1) == [RIGHT_ANGLE]
    turned = affinity.rotate(box(0, 0, 20, 10), 30, origin=(0, 0))
    # The long edges (at 30 deg) come before the short ones (at 120 deg).
    assert candidate_lane_angles(turned) == pytest.approx(
        [RIGHT_ANGLE, math.radians(30), math.radians(120)], abs=1e-5)
    with pytest.raises(ValueError, match='limit'):
        candidate_lane_angles(wide, limit=0)


def test_candidates_are_ranked_by_edge_length():
    lawn = box(0, 0, 30, 20).difference(affinity.rotate(box(10, 8, 14, 9), 20))
    angles = candidate_lane_angles(lawn)
    assert angles[:2] == [RIGHT_ANGLE, 0.0]  # the boundary outweighs the small bed
    assert angles[2:] == pytest.approx([math.radians(20), math.radians(110)], abs=1e-5)
    assert len(candidate_lane_angles(lawn, limit=3)) == 3


def test_a_wide_lawn_gets_lanes_along_its_long_side(energy_cost):
    lawn = box(0, 0, 20, 8).difference(box(8, 3, 10, 5))
    default = build_problem(lawn, DEFAULT_WIDTH, energy_cost, start=(1, 1, 0))
    choice = choose_direction(lawn, DEFAULT_WIDTH, energy_cost, start=(1, 1, 0), screen=SCREEN,
                              default=default)
    assert choice.lane_angle == 0.0 and choice.problem.lane_angle == 0.0
    assert [a for a, _ in choice.scores] == [RIGHT_ANGLE, 0.0]
    by_angle = dict(choice.scores)
    assert by_angle[0.0] < by_angle[RIGHT_ANGLE]
    assert choice.evaluations == 2 * SCREEN.evaluations
    # The same call gives the same choice, and limit=1 keeps the default problem.
    again = choose_direction(lawn, DEFAULT_WIDTH, energy_cost, start=(1, 1, 0), screen=SCREEN)
    assert again.scores == choice.scores
    only = choose_direction(lawn, DEFAULT_WIDTH, energy_cost, start=(1, 1, 0), screen=SCREEN,
                            limit=1, default=default)
    assert only.problem is default and only.evaluations == SCREEN.evaluations


def test_a_tall_lawn_keeps_the_default_direction(energy_cost):
    lawn = box(0, 0, 8, 20).difference(box(3, 8, 5, 10))
    choice = choose_direction(lawn, DEFAULT_WIDTH, energy_cost, screen=SCREEN)
    assert choice.lane_angle == RIGHT_ANGLE


def test_the_chosen_plan_stays_on_the_lawn_and_covers_it(energy_cost):
    from lawn_mower_planning.energy_order import energy_ga_order
    lawn = affinity.rotate(box(0, 0, 20, 8).difference(box(8, 3, 10, 5)), 25, origin=(0, 0))
    choice = choose_direction(lawn, DEFAULT_WIDTH, energy_cost, screen=SCREEN)
    assert choice.lane_angle == pytest.approx(math.radians(25), abs=1e-5)
    problem = choice.problem
    plan = energy_ga_order(problem.table, SCREEN, seed=0).plan
    free = lawn.buffer(-DEFAULT_WIDTH / 2, join_style='mitre')
    assert free.buffer(1e-6).covers(LineString(problem.path(plan)))
    assert problem.coverage(plan)['coverage'] > 90


@pytest.mark.parametrize('lawn', shared_lawns(), ids=lambda lawn: lawn.name)
def test_every_candidate_direction_gives_sound_cells(lawn):
    """Cells are monotone, have no holes and tile the free area, in every direction."""
    for angle in candidate_lane_angles(lawn.area):
        sweep = affinity.rotate(lawn.area, RIGHT_ANGLE - angle, origin=(0, 0), use_radians=True)
        free = sweep.buffer(-DEFAULT_WIDTH / 2, join_style='mitre')
        cells = boustrophedon_cells(free)
        assert all(monotone(c) and not c.interiors for c in cells), math.degrees(angle)
        assert sum(c.area for c in cells) == pytest.approx(free.area)
