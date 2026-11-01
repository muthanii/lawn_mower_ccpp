import pytest

from lawn_mower_planning.maps import CELL_RANGE, DEFAULT_WIDTH, lawn_by_name, random_lawn, \
    shared_lawns
from lawn_mower_planning.problem import build_problem

LAWNS = shared_lawns()


def test_names_are_unique():
    names = [lawn.name for lawn in LAWNS]
    assert len(names) == len(set(names)) == 10


@pytest.mark.parametrize('lawn', LAWNS, ids=lambda lawn: lawn.name)
def test_lawn_has_the_agreed_cell_count_and_a_usable_start(lawn):
    problem = build_problem(lawn.area, DEFAULT_WIDTH, start=lawn.start)  # raises on a bad start
    assert CELL_RANGE[0] <= problem.table.n_cells <= CELL_RANGE[1]
    assert lawn.area.is_valid


def test_random_lawns_are_reproducible():
    assert random_lawn(2).area.equals(random_lawn(2).area)
    assert not random_lawn(2).area.equals(random_lawn(3).area)


def test_lookup_by_name():
    assert lawn_by_name('garden').name == 'garden'
    with pytest.raises(KeyError, match='no lawn'):
        lawn_by_name('moon')
