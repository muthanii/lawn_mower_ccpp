"""Exact orderings by trying every permutation, for small maps (sanity checks)."""

import itertools
import math

from .transitions import best_variants, make_plan

MAX_CELLS = 9  # 9! = 362880 orders


def brute_force_order(table, weights):
    """Optimal Plan under the objective {metric: weight}. Raises for > MAX_CELLS cells."""
    n = table.n_cells
    if n > MAX_CELLS:
        raise ValueError(f'brute force is limited to {MAX_CELLS} cells, table has {n}')
    cost = table.combine(weights)
    best, best_order = math.inf, tuple(range(n))
    for order in itertools.permutations(range(n)):
        total, _ = best_variants(order, cost)
        if total < best:
            best, best_order = total, order
    return make_plan(table, cost, best_order)


def brute_force_tour(dist):
    """Shortest closed tour over a distance matrix: (length, order starting at 0)."""
    n = len(dist)
    if n > MAX_CELLS:
        raise ValueError(f'brute force is limited to {MAX_CELLS} cells, got {n}')
    best, best_order = math.inf, tuple(range(n))
    for rest in itertools.permutations(range(1, n)):
        order = (0, *rest)
        length = sum(dist[a][b] for a, b in zip(order, order[1:] + order[:1]))
        if length < best:
            best, best_order = length, order
    return best, best_order
