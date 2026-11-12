import math
import os

import numpy as np
import pytest
from shapely import affinity
from shapely.geometry import LineString, Polygon, box

from lawn_mower_planning.brute_force import brute_force_order
from lawn_mower_planning.decomposition import boustrophedon_cells
from lawn_mower_planning.lanes import cell_route, cell_routes, lane_segments, monotone
from lawn_mower_planning.path_cost import PathCost
from lawn_mower_planning.problem import build_problem
from lawn_mower_planning.roadmap import Roadmap
from lawn_mower_planning.xie_ga_aco import XieGaAcoParams, length_turns_order, xie2024_order

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


def test_bcd_splits_at_an_edge_whose_x_is_not_a_round_number():
    """An obstacle edge a rounding error from a slab boundary still starts new cells.

    The beds are from a random lawn on which the cells above and below a bed
    were joined into one cell, so the lanes left a part of the lawn out.
    """
    beds = [box(12.671770707560109, 4.2150303563032026, 15.282136429588556, 7.001970377628751),
            box(18.0369169981593, 4.922419305336799, 22.97912859756301, 9.80739088632816),
            box(15.693188728276859, 13.318568633162727, 19.278956518056923, 16.69657628181894)]
    lawn = box(0, 0, 30, 20)
    for bed in beds:
        lawn = lawn.difference(bed)
    free = lawn.buffer(-0.25, join_style='mitre')
    cells = boustrophedon_cells(free)
    assert len(cells) == 10  # 3 per bed and the cell on the left
    assert all(monotone(c) and not c.interiors for c in cells)
    assert sum(c.area for c in cells) == pytest.approx(free.area)


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


def test_xie2024_on_the_pipeline(problem):
    result = xie2024_order(problem, XieGaAcoParams(evaluations=2000, population=50, ants=25))
    assert sorted(result.plan.order) == list(range(7)) and result.plan.feasible
    assert set(result.plan.metrics) == {'energy', 'length', 'turns'}
    assert problem.path(result.plan)[0] == pytest.approx((1, 1))
    # Its objective ignores energy, so it cannot beat the energy optimum.
    assert result.plan.metrics['energy'] >= brute_force_order(problem.table,
                                                              {'energy': 1}).metrics['energy']


def test_length_turns_ablation_matches_brute_force_on_the_pipeline(problem):
    params = XieGaAcoParams(evaluations=2000, population=50, ants=25)
    best = brute_force_order(problem.table, {'length': 1, 'turns': params.turn_weight}).cost
    assert length_turns_order(problem.table, params, seed=0).plan.cost <= 1.01 * best


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


U_CELL = Polygon([(0, 0), (3, 0), (3, 3), (2, 3), (2, 1), (1, 1), (1, 3), (0, 3)])


def test_turned_lanes_cover_the_cell_and_stay_inside():
    flat = box(0, 0, 5, 1)
    routes = cell_routes(flat, 0.5, (0.0, math.pi / 2, math.pi / 4))
    assert len(routes) == 12
    for route in routes:
        assert flat.buffer(1e-6).covers(LineString(route))
    cost = PathCost()
    vertical, horizontal = cost(routes[0]), cost(routes[4])
    assert vertical['turns'] == 18      # 10 lanes across the 5 m side
    assert horizontal['turns'] == 2     # 2 lanes along it
    assert LineString(routes[4][:2]).length == pytest.approx(5)  # first lane runs along x


def test_angles_where_the_cell_is_not_monotone_are_unusable():
    assert monotone(U_CELL)
    assert not monotone(affinity.rotate(U_CELL, -math.pi / 2, origin=(0, 0), use_radians=True))
    routes = cell_routes(U_CELL, 0.5, (0.0, math.pi / 2))
    assert all(r is not None for r in routes[:4])
    assert routes[4:] == [None] * 4


@pytest.fixture(scope='module')
def angled(pioneer_cost):
    return build_problem(LAWN, 0.5, pioneer_cost, start=(1, 1, 0),
                         sweep_angles=(0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4))


def test_sweep_angles_widen_the_variants(problem, angled):
    assert (angled.table.n_cells, angled.table.n_variants) == (7, 16)
    for m in problem.table.metrics:  # angle 0 is the same as the fixed lanes
        np.testing.assert_allclose(angled.table.cell[m][:, :4], problem.table.cell[m])
        np.testing.assert_allclose(angled.table.transition[m][:, :4, :, :4],
                                   problem.table.transition[m])
    fixed = brute_force_order(problem.table, {'energy': 1})
    free = brute_force_order(angled.table, {'energy': 1})
    assert free.cost <= fixed.cost
    assert any(angled.variant_angle(a) != 0 for a in free.variants), 'some cell turns its lanes'


def test_angled_table_and_path_agree(angled):
    plan = brute_force_order(angled.table, {'length': 1, 'turns': 1})
    path = angled.path(plan)
    assert path[0] == pytest.approx((1, 1))
    assert LineString(path).length == pytest.approx(plan.metrics['length'])
    assert PathCost()(path, start_heading=0.0)['turns'] == plan.metrics['turns']
    assert LAWN.buffer(-0.25 + 1e-6).covers(LineString(path))
    assert angled.coverage(plan)['coverage'] > 90


def test_xie2024_takes_each_cells_fewest_turns_angle(angled):
    result = xie2024_order(angled, XieGaAcoParams(evaluations=2000, population=50, ants=25))
    assert sorted(result.plan.order) == list(range(7)) and result.plan.feasible
    for c, a in zip(result.plan.order, result.plan.variants):
        assert a in angled.fewest_turns_angle(c)
        turns = angled.table.cell['turns'][c].reshape(4, 4)  # (angle, corner)
        assert turns[a // 4].min() == turns.min()


def test_rejects_sweep_angles_without_the_bcd_direction():
    with pytest.raises(ValueError, match='must include 0'):
        build_problem(LAWN, 0.5, sweep_angles=(math.pi / 2,))
