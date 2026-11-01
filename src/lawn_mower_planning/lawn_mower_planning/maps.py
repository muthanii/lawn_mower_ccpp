"""Shared test lawns for the cell-ordering experiments (issue #21).

Every method runs on the same lawns. Each lawn is a polygon in metres with
obstacles (flower beds, trees, a shed, a pond) as holes, and a robot start
pose inside the free area. With the default 0.5 m cutting width the lawns
give 6 to 15 BCD cells (test/test_maps.py checks this).

The hand-made lawns cover the common shapes: rectangles with beds, an L, an
irregular boundary, round trees. The random lawns add seeded variety: the
same seed always gives the same lawn.
"""

from dataclasses import dataclass

import numpy as np
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

DEFAULT_WIDTH = 0.5        # m, cutting width and lane spacing
CELL_RANGE = (6, 15)       # BCD cells per lawn at the default width
START = (1.0, 1.0, 0.0)    # robot start pose (x, y, heading) on the lawns that allow it
TREE_SIDES = 2             # quarter-circle segments of a round obstacle (an octagon)


@dataclass(frozen=True)
class Lawn:
    name: str
    area: Polygon          # lawn with obstacle holes, world frame [m]
    start: tuple = START   # robot start pose (x, y, heading)
    note: str = ''


def tree(x, y, r):
    return Point(x, y).buffer(r, quad_segs=TREE_SIDES)


def ellipse(x, y, a, b, angle=0.0):
    """Ellipse with half-axes a, b [m], turned by angle [deg]."""
    return affinity.rotate(affinity.scale(tree(x, y, 1), a, b), angle)


def _cut(outline, *obstacles):
    return outline.difference(unary_union(obstacles))


def hand_made():
    return [
        Lawn('two_beds', _cut(box(0, 0, 20, 12), box(5, 4, 8, 8), box(13, 2, 16, 6)),
             note='small rectangle, two flower beds'),
        Lawn('backyard', _cut(box(0, 0, 25, 18), box(0, 12, 6, 18), box(10, 6, 14, 9),
                              tree(19, 13, 1.4)),
             note='shed in a corner, a bed and a tree'),
        Lawn('l_shape', _cut(Polygon([(0, 0), (30, 0), (30, 10), (12, 10), (12, 25), (0, 25)]),
                             box(4, 14, 8, 18), box(18, 3, 22, 6)),
             note='L-shaped lawn, two beds'),
        Lawn('three_beds', _cut(box(0, 0, 30, 15), box(5, 3, 9, 7), box(13, 8, 17, 12),
                                box(22, 4, 25, 10)),
             note='rectangle, three staggered beds'),
        Lawn('garden', _cut(Polygon([(0, 0), (28, 0), (34, 10), (30, 22), (6, 24), (-2, 12)]),
                            tree(8, 8, 1.2), Polygon([(16, 14), (21, 14), (18.5, 19)]),
                            ellipse(22, 6, 3, 1.5, 20)),
             start=(2.0, 1.0, 0.0), note='irregular boundary, tree, triangular bed, pond'),
        Lawn('park_trees', _cut(box(0, 0, 40, 25), tree(8, 8, 1.5), tree(16, 17, 1.5),
                                tree(26, 9, 1.5), tree(33, 18, 1.5)),
             note='large rectangle, four round trees'),
        Lawn('sports_edge', _cut(box(0, 0, 45, 30), box(6, 5, 10, 9), tree(15, 22, 1.5),
                                 box(21, 10, 25, 16), box(37, 18, 41, 24)),
             note='large rectangle, beds and a tree spread out'),
    ]


def random_lawn(seed, size=(30.0, 20.0), obstacles=(2, 5), width=DEFAULT_WIDTH):
    """Rectangle with seeded random beds and trees, kept apart by at least 2 m.

    Draws again (deterministically) until the lawn has CELL_RANGE cells.
    """
    from .decomposition import boustrophedon_cells
    rng = np.random.default_rng(seed)
    w, h = size
    for _ in range(100):
        shapes = []
        for _ in range(rng.integers(obstacles[0], obstacles[1] + 1)):
            for _ in range(50):
                x, y = rng.uniform(4, w - 4), rng.uniform(4, h - 4)
                if rng.random() < 0.5:
                    a, b = rng.uniform(1, 2.5, 2)
                    shape = box(x - a, y - b, x + a, y + b)
                else:
                    shape = tree(x, y, rng.uniform(0.8, 1.8))
                if all(shape.distance(s) > 2 for s in shapes):
                    shapes.append(shape)
                    break
        area = _cut(box(0, 0, w, h), *shapes)
        cells = len(boustrophedon_cells(area.buffer(-width / 2, join_style='mitre')))
        if CELL_RANGE[0] <= cells <= CELL_RANGE[1]:
            return Lawn(f'random_{seed}', area, note=f'{len(shapes)} random obstacles')
    raise ValueError(f'no lawn with {CELL_RANGE} cells for seed {seed}')


RANDOM_SEEDS = (1, 2, 3)


def shared_lawns():
    """All shared lawns, in a fixed order."""
    return hand_made() + [random_lawn(s) for s in RANDOM_SEEDS]


def lawn_by_name(name):
    for lawn in shared_lawns():
        if lawn.name == name:
            return lawn
    raise KeyError(f'no lawn {name!r}; have {[lawn.name for lawn in shared_lawns()]}')
