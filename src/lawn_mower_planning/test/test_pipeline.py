import math
import os

import numpy as np
import pytest
from shapely import affinity
from shapely.geometry import LineString, Polygon, box

from lawn_mower_planning.brute_force import brute_force_order
from lawn_mower_planning.decomposition import boustrophedon_cells
from lawn_mower_planning.lanes import cell_route, lane_segments
from lawn_mower_planning.path_cost import PathCost
from lawn_mower_planning.problem import build_problem
from lawn_mower_planning.roadmap import Roadmap
from lawn_mower_planning.xie_ga_aco import XieGaAcoParams, xie_ga_aco_order

DIAMOND = Polygon([(12, 2), (15, 5), (13, 8), (11, 5)])
LAWN = box(0, 0, 20, 10).difference(box(5, 3, 8, 6)).difference(DIAMOND)
ROBOT_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'lawn_mower_robot', 'config', 'robot', 'pioneer3at.yaml')


def test_bcd_cells():
    assert len(boustrophedon_cells(box(0, 0, 4, 3))) == 1
    cells = boustrophedon_cells(box(0, 0, 10, 10).difference(box(4, 4, 6, 6)))
    assert len(cells) == 4
    cells = boustrophedon_cells(LAWN)
    # Square: left, below, above, right. Diamond: below, above, right
    # (its left part shares the cell between the obstacles).
    assert len(cells) == 7
    assert sum(c.area for c in cells) == pytest.approx(LAWN.area)
    for c in cells:
        xmin, ymin, xmax, ymax = c.bounds
        for x in np.linspace(xmin, xmax, 23)[1:-1]:
            cut = c.intersection(LineString([(x, ymin - 1), (x, ymax + 1)]))
            assert len(getattr(cut, 'geoms', [cut])) == 1, 'a vertical line meets a cell once'


def test_lanes_cover_the_cell_and_stay_inside():
    below_diamond = next(c for c in boustrophedon_cells(LAWN) if c.bounds[0] == 11)
    lanes = lane_segments(box(0, 0, 3, 5), 0.5)
    assert [x for x, _, _ in lanes] == pytest.approx([0.25, 0.75, 1.25, 1.75, 2.25, 2.75])
    for cell in (box(0, 0, 3, 5), below_diamond):
        for variant in range(4):
            route = cell_route(cell, 0.5, variant)
            assert cell.buffer(1e-6).covers(LineString(route))
    up_from_left = cell_route(box(0, 0, 3, 5), 0.5, 0)
    assert up_from_left[:3] == [(0.25, 0), (0.25, 5), (0.75, 5)]
    assert cell_route(box(0, 0, 3, 5), 0.5, 3)[0] == (2.75, 5)


def test_astar_goes_around_obstacles():
    area = box(0, 0, 10, 10).difference(box(4, 0, 6, 8))
    path = Roadmap(area).path((1, 1), (9, 1))
    assert LineString(path).length == pytest.approx(2 * math.hypot(3, 7) + 2)
    split = box(0, 0, 4, 4).union(box(6, 0, 10, 4))
    assert Roadmap(split).path((1, 1), (9, 1)) is None


def test_turn_count_and_energy(pioneer_cost):
    cost = PathCost()
    u_turn = [(0, 0), (0, 5), (1, 5), (1, 0)]
    assert cost(u_turn) == {'length': 11, 'turns': 2}
    assert cost(u_turn, start_heading=0.0, end_heading=-math.pi / 2)['turns'] == 3
    assert cost([(2, 2), (2, 2)], math.pi / 2, -math.pi / 2)['turns'] == 1
    m = pioneer_cost.energy_model
    expected = (m.straight(5, pioneer_cost.speed).electrical
                + m.straight(1, pioneer_cost.speed).electrical
                + 2 * m.turn(math.pi / 2, pioneer_cost.turn_rate).electrical)
    assert pioneer_cost([(0, 0), (0, 5), (1, 5)], end_heading=-math.pi / 2)['energy'] \
        == pytest.approx(expected)


@pytest.fixture(scope='module')
def pioneer_cost():
    from lawn_mower_robot.robot_config import load_robot_config
    return PathCost.from_config(load_robot_config(ROBOT_CONFIG))


@pytest.fixture(scope='module')
def problem(pioneer_cost):
    return build_problem(LAWN, 0.5, pioneer_cost, start=(1, 1, 0))


def test_table_and_path_agree(problem):
    table = problem.table
    assert table.metrics == ('energy', 'length', 'turns')
    assert (table.n_cells, table.n_variants) == (7, 4)
    plan = brute_force_order(table, {'length': 1, 'turns': 1})
    path = problem.path(plan)
    assert path[0] == pytest.approx((1, 1))
    assert LineString(path).length == pytest.approx(plan.metrics['length'])
    assert PathCost()(path, start_heading=0.0)['turns'] == plan.metrics['turns']
    assert LAWN.buffer(-0.25 + 1e-6).covers(LineString(path))
    stats = problem.coverage(plan)
    assert stats['coverage'] > 90 and 0 <= stats['overlap'] < 20


def test_ga_aco_matches_brute_force_on_the_pipeline(problem):
    params = XieGaAcoParams(evaluations=1000)
    assert xie_ga_aco_order(problem.table, params, seed=0).plan.cost == pytest.approx(
        brute_force_order(problem.table, params.weights).cost)


def test_lane_angle_follows_the_lawn():
    turned = affinity.rotate(LAWN, 0.4, origin=(0, 0), use_radians=True)
    a = build_problem(LAWN, 0.5).table
    b = build_problem(turned, 0.5, lane_angle=math.pi / 2 + 0.4).table
    for m in a.metrics:
        np.testing.assert_allclose(a.cell[m], b.cell[m], atol=1e-6)
        np.testing.assert_allclose(a.transition[m], b.transition[m], atol=1e-6)


def test_rejects_a_start_in_the_clearance():
    with pytest.raises(ValueError, match='outside the free area'):
        build_problem(LAWN, 0.5, start=(0, 0, 0))
