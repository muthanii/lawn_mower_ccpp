import math

import numpy as np
import pytest

from lawn_mower_planning.transitions import TransitionTable


def lawn_table(n_cells, seed, lane=0.5, with_start=True):
    """Synthetic table: n rectangular cells along a lawn, covered by vertical lanes.

    Variant = start corner (4). Straight-line distance stands in for A*, and
    'energy' weights turns more heavily than the length+turns objective does.
    """
    rng = np.random.default_rng(seed)
    x = np.cumsum(rng.uniform(1.0, 4.0, n_cells))
    rects = [(x0, y0, x0 + w, y0 + h) for x0, y0, w, h in zip(
        rng.permutation(x), rng.uniform(0, 6, n_cells), rng.uniform(1, 4, n_cells),
        rng.uniform(2, 8, n_cells))]
    k = 4
    entry = np.zeros((n_cells, k, 2))
    exit_ = np.zeros((n_cells, k, 2))
    cell_len, cell_turns = np.zeros((n_cells, k)), np.zeros((n_cells, k))
    for c, (x0, y0, x1, y1) in enumerate(rects):
        lanes = max(1, math.ceil((x1 - x0) / lane))
        for a, (left, bottom) in enumerate([(1, 1), (1, 0), (0, 1), (0, 0)]):
            sx, ex = (x0, x1) if left else (x1, x0)
            sy = y0 if bottom else y1
            ey = sy if lanes % 2 == 0 else (y1 if bottom else y0)
            entry[c, a], exit_[c, a] = (sx, sy), (ex, ey)
            cell_len[c, a] = lanes * (y1 - y0) + (lanes - 1) * lane
            cell_turns[c, a] = 2 * (lanes - 1)

    def link(p, q):
        d = q - p
        dist = np.linalg.norm(d, axis=-1)
        aligned = np.isclose(d[..., 0], 0) | np.isclose(d[..., 1], 0)
        return dist, np.where(dist < 1e-9, 0, np.where(aligned, 1, 2))

    t_len, t_turns = link(exit_[:, :, None, None, :], entry[None, None, :, :, :])
    s_len, s_turns = link(np.zeros(2), entry)

    def energy(length, turns):
        return 40.0 * length + 300.0 * turns

    cell = {'length': cell_len, 'turns': cell_turns, 'energy': energy(cell_len, cell_turns)}
    trans = {'length': t_len, 'turns': t_turns, 'energy': energy(t_len, t_turns)}
    start = {'length': s_len, 'turns': s_turns, 'energy': energy(s_len, s_turns)} \
        if with_start else None
    return TransitionTable(cell, trans, start)


@pytest.fixture
def table():
    return lawn_table(6, seed=1)
