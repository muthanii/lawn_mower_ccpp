import math

import pytest
from shapely.geometry import LineString, Point, box

from lawn_mower_planning.contour import (order_passes, rings, side_ring_passes,
                                         turn_ring_passes)
from lawn_mower_planning.footprint import BodyCheck, Footprint, body_area, heading
from lawn_mower_planning.roadmap import Roadmap

BODY = Footprint(length=1.2, width=1.0)
CLEARANCE = BODY.turning_radius + 0.1
RUN_MARGIN, PIVOT_MARGIN = 0.15, 0.05
LAWN = box(0, 0, 12, 9).difference(box(5, 3.5, 7, 5.5))


def setup(lawn):
    free = lawn.buffer(-CLEARANCE, join_style='mitre')
    run = BodyCheck(BODY, body_area(lawn, RUN_MARGIN, RUN_MARGIN))
    pivot = BodyCheck(BODY, body_area(lawn, PIVOT_MARGIN, PIVOT_MARGIN))
    side = run.area.buffer(-BODY.width / 2, join_style='mitre')
    return free, side, run, pivot


def fits(points, run, pivot):
    """Every pivot fits, and every run fits as a run or as a manoeuvre."""
    last = None
    for p, q in zip(points, points[1:]):
        h = heading(p, q)
        if last is not None and not pivot.pivot(p, last, h):
            return False
        if not run.run(p, q) and not pivot.run(p, q):
            return False
        last = h
    return True


def test_rings_drop_collinear_vertices():
    square = box(0, 0, 4, 4).union(box(4, 0, 8, 4))   # one rectangle with extra vertices
    assert len(rings(square)) == 1 and len(rings(square)[0]) == 4


def test_turn_ring_passes_are_closed_loops_on_the_free_boundary():
    free, _, run, pivot = setup(LAWN)
    passes = turn_ring_passes(free)
    assert len(passes) == 2                            # lawn edge and the bed
    for points in passes:
        assert points[0] == points[-1]
        assert all(free.boundary.distance(Point(p)) < 1e-9 for p in points)
        assert fits(points, run, pivot)


def test_side_passes_start_and_end_in_the_free_area_and_fit():
    free, side, run, pivot = setup(LAWN)
    passes = side_ring_passes(side, free, run, pivot)
    assert len(passes) == 2
    for points in passes:
        assert free.distance(Point(points[0])) < 1e-6
        assert free.distance(Point(points[-1])) < 1e-6
        assert fits(points, run, pivot)


def test_side_pass_runs_at_half_the_width_plus_the_margin():
    free, side, run, pivot = setup(LAWN)
    outer = max(side_ring_passes(side, free, run, pivot), key=lambda p: LineString(p).length)
    edge = LAWN.exterior
    closest = min(edge.distance(Point(p)) for p in outer)
    assert closest == pytest.approx(BODY.width / 2 + RUN_MARGIN)
    # All four edges are visited.
    near = {side for p, q in zip(outer, outer[1:])
            for side, value in (('left', 0.65), ('right', 11.35), ('bottom', 0.65), ('top', 8.35))
            if (side in ('left', 'right') and p[0] == pytest.approx(value)
                and q[0] == pytest.approx(value))
            or (side in ('bottom', 'top') and p[1] == pytest.approx(value)
                and q[1] == pytest.approx(value))}
    assert near == {'left', 'right', 'bottom', 'top'}


def test_a_square_corner_is_left_on_a_shallow_diagonal():
    free, side, run, pivot = setup(LAWN)
    outer = max(side_ring_passes(side, free, run, pivot), key=lambda p: LineString(p).length)
    turns = [abs(math.degrees(math.atan2(math.sin(b - a), math.cos(b - a))))
             for a, b in zip([heading(p, q) for p, q in zip(outer, outer[1:])],
                             [heading(p, q) for p, q in zip(outer[1:], outer[2:])])]
    on_edge = [t for t, p in zip(turns, outer[1:]) if free.distance(Point(p)) > 1e-6]
    assert on_edge and max(on_edge) < 15              # small pivots next to the edge
    assert max(turns) > 60                            # the large ones are in the free area


def test_the_ring_round_an_obstacle_is_a_loop_with_a_spur():
    free, side, run, pivot = setup(LAWN)
    inner = min(side_ring_passes(side, free, run, pivot), key=lambda p: LineString(p).length)
    assert inner[0] == pytest.approx(inner[-1])
    assert LAWN.interiors[0].distance(LineString(inner)) == pytest.approx(
        BODY.width / 2 + RUN_MARGIN)
    assert free.distance(Point(inner[1])) > 0.1        # the loop itself is outside the free area


def test_a_lawn_too_narrow_for_the_body_gives_no_passes():
    lawn = box(0, 0, 10, 1.1)
    free, side, run, pivot = setup(lawn)
    assert free.is_empty or side_ring_passes(side, free, run, pivot) == []


def test_order_passes_goes_to_the_nearest_first_and_links_through_the_free_area():
    free, side, run, pivot = setup(LAWN)
    passes = turn_ring_passes(free) + side_ring_passes(side, free, run, pivot)
    start = (CLEARANCE, CLEARANCE)
    ordered = order_passes(passes, start, Roadmap(free))
    assert len(ordered) == len(passes)
    here = start
    for link, route in ordered:
        assert link[0] == pytest.approx(here) and link[-1] == pytest.approx(route[0])
        assert free.buffer(1e-6).covers(LineString(link)) or len(link) < 2 \
            or LineString(link).length < 1e-9
        here = route[-1]
    assert LineString(ordered[0][0]).length < 1e-9     # the first pass starts where the robot is


def test_a_closed_pass_is_entered_at_its_nearest_vertex():
    loop = [(0, 0), (4, 0), (4, 4), (0, 4), (0, 0)]
    (link, route), = order_passes([loop], (5, 5), Roadmap(box(-1, -1, 6, 6)))
    assert route[0] == (4, 4) and route[-1] == (4, 4) and len(route) == 5


def test_a_closed_pass_is_not_entered_outside_the_free_area():
    free, side, run, pivot = setup(LAWN)
    loop = min(side_ring_passes(side, free, run, pivot), key=lambda p: LineString(p).length)
    far_corner = max(loop, key=lambda p: math.dist(p, loop[0]))
    start = min(rings(free)[1], key=lambda p: math.dist(p, far_corner))   # nearest free point
    (link, route), = order_passes([loop], start, Roadmap(free))
    assert math.dist(start, far_corner) < math.dist(start, loop[0])
    assert route[0] == pytest.approx(loop[0])          # by the spur, the only way in
