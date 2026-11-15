import math

import pytest
from shapely.geometry import box

from lawn_mower_navigation.evaluation import (Sample, coverage, footprint_excursion,
                                              path_length, thin, tracking_error)

LAWN = box(0, 0, 10, 4)
WIDTH = 0.5


def line(y, x0, x1, cutting=True, t0=0.0, step=0.05):
    n = int(abs(x1 - x0) / step)
    heading = 0.0 if x1 > x0 else math.pi
    return [Sample(t0 + i, x0 + (x1 - x0) * i / n, y, heading, cutting) for i in range(n + 1)]


def test_one_lane_covers_its_strip():
    result = coverage(line(2.0, 0.0, 10.0), LAWN, WIDTH)
    assert result['coverage'] == pytest.approx(100 * 10 * WIDTH / LAWN.area, rel=1e-3)
    assert result['overlap'] == pytest.approx(0.0, abs=1e-6)


def test_nothing_is_mowed_with_the_blade_off():
    assert coverage(line(2.0, 0.0, 10.0, cutting=False), LAWN, WIDTH)['coverage'] == 0.0


def test_two_lanes_half_a_width_apart_overlap_by_a_third():
    samples = line(2.0, 0.0, 10.0) + line(2.0, 10.0, 0.0, cutting=False)[:1] \
        + line(2.0 + WIDTH / 2, 10.0, 0.0)
    result = coverage(samples, LAWN, WIDTH)
    assert result['coverage'] == pytest.approx(100 * 10 * 1.5 * WIDTH / LAWN.area, rel=1e-2)
    assert result['overlap'] == pytest.approx(100 / 3, rel=2e-2)


def test_mowing_outside_the_lawn_does_not_count():
    result = coverage(line(2.0, -5.0, 15.0), LAWN, WIDTH)
    assert result['coverage'] == pytest.approx(100 * 10 * WIDTH / LAWN.area, rel=1e-3)


def test_footprint_inside_and_outside():
    inside = line(2.0, 1.0, 9.0)
    assert footprint_excursion(inside, LAWN, 1.2, 1.0) == 0.0
    # The centre stops 0.3 m from the edge; half the body length is 0.6 m.
    near = line(2.0, 1.0, 9.7)
    assert footprint_excursion(near, LAWN, 1.2, 1.0) == pytest.approx(0.3, abs=0.01)


def test_footprint_in_an_obstacle():
    lawn = LAWN.difference(box(4, 2.6, 6, 3.4))
    assert footprint_excursion(line(2.0, 1.0, 9.0), lawn, 1.2, 1.0) == 0.0     # passes 0.1 m below
    assert footprint_excursion(line(2.2, 1.0, 9.0), lawn, 1.2, 1.0) == pytest.approx(0.1, abs=0.01)


def test_tracking_error():
    mean, worst = tracking_error(line(2.1, 0.0, 10.0), [(0, 2), (10, 2)])
    assert mean == pytest.approx(0.1) and worst == pytest.approx(0.1)


def test_thin_and_length():
    samples = line(2.0, 0.0, 10.0, step=0.001)
    assert len(thin(samples)) < len(samples) / 10
    assert path_length(thin(samples)) == pytest.approx(10.0, abs=0.05)
