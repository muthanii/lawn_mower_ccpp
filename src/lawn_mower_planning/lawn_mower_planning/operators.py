"""Permutation operators shared by the GA-based orderings."""

import numpy as np


def pmx(p1, p2, i, j):
    """Partially mapped crossover: the child keeps p1[i:j], the rest comes from p2."""
    child = np.full(len(p1), -1)
    child[i:j] = p1[i:j]
    where = {int(v): k for k, v in enumerate(p2)}
    kept = set(p1[i:j].tolist())
    for k in range(i, j):
        v = int(p2[k])
        if v in kept:
            continue
        pos = k
        while i <= pos < j:
            pos = where[int(p1[pos])]
        child[pos] = v
    free = child < 0
    child[free] = p2[free]
    return child


def inversion_mutation(order, rng):
    """Reverse a random segment."""
    i, j = sorted(rng.choice(len(order) + 1, size=2, replace=False))
    out = order.copy()
    out[i:j] = out[i:j][::-1]
    return out


def ordered_crossover(p1, p2, rng):
    """OX: keep a random slice of p1, fill the rest in p2's order, starting after the slice."""
    n = len(p1)
    i, j = sorted(rng.choice(n + 1, size=2, replace=False))
    child = np.full(n, -1)
    child[i:j] = p1[i:j]
    kept = set(p1[i:j].tolist())
    fill = [c for c in np.roll(p2, -j) if c not in kept]
    child[np.roll(np.arange(n), -j)[:len(fill)]] = fill
    return child
