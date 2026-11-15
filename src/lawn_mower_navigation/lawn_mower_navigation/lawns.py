"""Lawns for the simulation: the shared test lawns, and a small one for quick runs."""

from shapely.geometry import box

from lawn_mower_planning.maps import Lawn, lawn_by_name

# Small enough that a full coverage run takes a few minutes of simulated time.
DEMO = Lawn('demo', box(0, 0, 12, 9).difference(box(5, 3.5, 7, 5.5)), start=(1.5, 1.5, 0.0),
            note='small rectangle with one flower bed, for quick simulation runs')


def find_lawn(name):
    """The lawn called `name`: demo, or one of lawn_mower_planning.maps."""
    return DEMO if name == DEMO.name else lawn_by_name(name)
