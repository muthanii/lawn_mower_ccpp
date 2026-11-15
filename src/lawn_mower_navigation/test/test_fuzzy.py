import numpy as np
import pytest

from lawn_mower_navigation.fuzzy import FuzzyTable, memberships
from lawn_mower_navigation.tracker import (CROSS_TRACK_CENTRES, HEADING_CENTRES, STEER_LEVELS,
                                           steering_rules)

CENTRES = (-1.0, 0.0, 1.0)


def test_memberships_add_up_to_one():
    for value in (-5, -1, -0.3, 0, 0.5, 1, 9):
        assert memberships(value, CENTRES).sum() == pytest.approx(1.0)


def test_memberships_at_and_between_centres():
    assert list(memberships(0.0, CENTRES)) == [0, 1, 0]
    assert list(memberships(0.25, CENTRES)) == [0, 0.75, 0.25]
    assert list(memberships(7.0, CENTRES)) == [0, 0, 1]


def test_table_interpolates_between_rules():
    table = FuzzyTable(CENTRES, CENTRES, np.add.outer([-1, 0, 1], [-1, 0, 1]))
    assert table(0, 0) == 0
    assert table(1, 1) == 2
    assert table(0.5, -0.5) == pytest.approx(0.0)
    assert table(0.5, 0) == pytest.approx(0.5)


def test_table_checks_its_shape_and_centres():
    with pytest.raises(ValueError):
        FuzzyTable(CENTRES, CENTRES, [[0, 0], [0, 0]])
    with pytest.raises(ValueError):
        FuzzyTable((0, 0, 1), CENTRES, np.zeros((3, 3)))


def test_steering_rules_steer_against_the_errors():
    steer = FuzzyTable(CROSS_TRACK_CENTRES, HEADING_CENTRES, steering_rules())
    assert steer(0, 0) == 0
    assert steer(0.2, 0) < 0 < steer(-0.2, 0)        # left of the line: turn right
    assert steer(0, 0.3) < 0 < steer(0, -0.3)        # pointing left: turn right
    assert steer(5, 5) == min(STEER_LEVELS)
    assert steer(0.15, -0.20) == pytest.approx(0)    # heading back to the line already


def test_steering_is_odd_and_monotone():
    steer = FuzzyTable(CROSS_TRACK_CENTRES, HEADING_CENTRES, steering_rules())
    grid = np.linspace(-0.5, 0.5, 21)
    for a in grid:
        for b in grid:
            assert steer(a, b) == pytest.approx(-steer(-a, -b))
        values = [steer(a, b) for b in grid]
        assert all(x >= y - 1e-12 for x, y in zip(values, values[1:]))
