import math

import pytest
from shapely.geometry import box

from lawn_mower_planning.footprint import BodyCheck, Footprint, body_area, turn_angle

BODY = Footprint(length=1.2, width=1.0)
LAWN = box(0, 0, 10, 6).difference(box(4, 2, 6, 4))


def test_turning_radius_is_half_the_diagonal():
    assert BODY.turning_radius == pytest.approx(math.hypot(1.2, 1.0) / 2)


def test_body_rectangle_follows_the_heading():
    along = BODY.at((5, 5), 0.0).bounds
    across = BODY.at((5, 5), math.pi / 2).bounds
    assert along == pytest.approx((4.4, 4.5, 5.6, 5.5))
    assert across == pytest.approx((4.5, 4.4, 5.5, 5.6))


def test_turn_angle_takes_the_short_way():
    assert turn_angle(math.radians(170), math.radians(-170)) == pytest.approx(math.radians(20))
    assert turn_angle(0.0, math.radians(-90)) == pytest.approx(math.radians(-90))


def test_body_area_has_separate_margins():
    area = body_area(LAWN, edge_margin=0.1, obstacle_margin=0.3)
    assert area.bounds == pytest.approx((0.1, 0.1, 9.9, 5.9))
    hole = area.interiors[0].bounds
    assert hole == pytest.approx((3.7, 1.7, 6.3, 4.3))


def test_run_along_an_edge_fits_where_a_run_across_does_not():
    check = BodyCheck(BODY, LAWN)
    assert check.run((1.0, 0.5), (9.0, 0.5))          # half the width from the edge
    assert not check.run((1.0, 0.45), (9.0, 0.45))
    assert not check.run((0.5, 1.0), (0.5, 0.5))      # nose over the edge
    assert check.run((2.0, 3.0), (3.4, 3.0))          # nose just short of the obstacle
    assert not check.run((2.0, 3.0), (3.5, 3.0))


def test_a_small_pivot_fits_where_a_full_turn_does_not():
    check = BodyCheck(BODY, LAWN)
    near = (5.0, 0.6)                                  # 0.1 m of room at the side
    assert check.pivot(near, 0.0, math.radians(5))
    assert not check.pivot(near, 0.0, math.radians(30))
    assert not check.pivot(near, 0.0, math.pi / 2)
    assert check.pivot((2.0, 3.0), 0.0, math.pi)       # open lawn: a full turn fits


def test_a_pivot_checks_the_sweep_and_not_only_its_ends():
    check = BodyCheck(BODY, LAWN)
    # Along the edge before and after, across it on the way.
    assert check.run((4.0, 0.5), (6.0, 0.5))
    assert not check.pivot((5.0, 0.5), 0.0, math.pi)


def test_path_lists_what_does_not_fit():
    check = BodyCheck(BODY, LAWN)
    assert check.path([(1, 1), (3, 1), (3, 5)], start_heading=0.0) == []
    problems = check.path([(1, 0.5), (9, 0.5), (9, 3)], start_heading=0.0)
    assert problems == ['pivot at point 1 (9.00, 0.50)', 'run from point 1 (9.00, 0.50)']
    assert check.path([(1, 1), (5, 3)]) == ['run from point 0 (1.00, 1.00)']
