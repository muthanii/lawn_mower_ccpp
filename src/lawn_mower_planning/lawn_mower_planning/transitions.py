"""Cell transition table shared by every cell-ordering method.

The pipeline splits the lawn into cells (BCD), covers each cell with
back-and-forth lanes and joins the cells with A* paths. A cell can be covered
in k variants: the start corner and lane direction, which fix where the robot
enters and leaves the cell. The table holds, for each metric (path length,
turn count, modelled energy, ...):

    cell[c, a]              covering cell c in variant a
    transition[i, a, j, b]  A* path from the exit of (i, a) to the entry of (j, b)
    start[c, a]             from the robot start to the entry of (c, a)

An unusable variant has cost inf. All ordering methods read the same table,
so they differ only in their objective: a weighted sum of the metrics.

For a fixed cell order the best variant of each cell is found exactly by
dynamic programming over the order (best_variants), so the metaheuristics only
search over permutations.
"""

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class TransitionTable:
    cell: dict          # metric -> (n, k) array
    transition: dict    # metric -> (n, k, n, k) array
    start: dict = None  # metric -> (n, k) array, or None for a free start
    metrics: tuple = field(init=False)

    def __post_init__(self):
        metrics = tuple(sorted(self.cell))
        problems = []
        if not metrics:
            problems.append('no metrics')
        if set(self.transition) != set(metrics):
            problems.append(f'transition metrics {sorted(self.transition)} != cell metrics {list(metrics)}')
        if self.start is not None and set(self.start) != set(metrics):
            problems.append(f'start metrics {sorted(self.start)} != cell metrics {list(metrics)}')
        if problems:
            raise ValueError('; '.join(problems))
        n, k = np.shape(self.cell[metrics[0]])
        for m in metrics:
            for name, arrays, shape in (('cell', self.cell, (n, k)),
                                        ('transition', self.transition, (n, k, n, k)),
                                        ('start', self.start, (n, k))):
                if arrays is not None and np.shape(arrays[m]) != shape:
                    problems.append(f'{name}[{m!r}] has shape {np.shape(arrays[m])}, expected {shape}')
        if problems:
            raise ValueError('; '.join(problems))
        object.__setattr__(self, 'metrics', metrics)

    @property
    def n_cells(self):
        return np.shape(self.cell[self.metrics[0]])[0]

    @property
    def n_variants(self):
        return np.shape(self.cell[self.metrics[0]])[1]

    def combine(self, weights):
        """Scalar cost arrays (cell, transition, start) for {metric: weight}."""
        unknown = set(weights) - set(self.metrics)
        if unknown:
            raise ValueError(f'unknown metrics {sorted(unknown)}; table has {list(self.metrics)}')
        n, k = self.n_cells, self.n_variants

        def total(arrays, shape):
            out = np.zeros(shape)
            for m, w in weights.items():
                if w:
                    out = out + w * np.asarray(arrays[m], dtype=float)
            return out

        start = np.zeros((n, k)) if self.start is None else total(self.start, (n, k))
        return CombinedCost(total(self.cell, (n, k)), total(self.transition, (n, k, n, k)), start)

    def totals(self, order, variants):
        """Every metric summed along a plan (start, cells and transitions)."""
        out = {}
        for m in self.metrics:
            cell, trans = np.asarray(self.cell[m]), np.asarray(self.transition[m])
            value = 0.0 if self.start is None else float(self.start[m][order[0]][variants[0]])
            value += sum(float(cell[c, a]) for c, a in zip(order, variants))
            value += sum(float(trans[i, a, j, b]) for i, a, j, b in
                         zip(order, variants, order[1:], variants[1:]))
            out[m] = value
        return out


@dataclass(frozen=True)
class CombinedCost:
    """Scalar costs for one objective (see TransitionTable.combine)."""

    cell: np.ndarray        # (n, k)
    transition: np.ndarray  # (n, k, n, k)
    start: np.ndarray       # (n, k)

    def edge_lower_bounds(self):
        """Cheapest transition from cell i to cell j over all variants: (n, n)."""
        return self.transition.min(axis=(1, 3))

    def start_lower_bounds(self):
        """Cheapest way from the robot start to cell j: (n,)."""
        return self.start.min(axis=1)


def best_variants(order, cost):
    """Best variant of each cell for a fixed order. Returns (total cost, variants).

    Exact dynamic programming over the order, O(n k^2).
    """
    order = list(order)
    first = order[0]
    acc = cost.start[first] + cost.cell[first]
    back = []
    for prev, cur in zip(order, order[1:]):
        step = acc[:, None] + cost.transition[prev, :, cur, :]  # (a, b)
        choice = np.argmin(step, axis=0)
        back.append(choice)
        acc = step[choice, np.arange(step.shape[1])] + cost.cell[cur]
    last = int(np.argmin(acc))
    total = float(acc[last])
    variants = [last]
    for choice in reversed(back):
        variants.append(int(choice[variants[-1]]))
    variants.reverse()
    return total, variants


@dataclass(frozen=True)
class Plan:
    order: tuple     # cell indices in visiting order
    variants: tuple  # variant of each cell, aligned with order
    cost: float      # objective value
    metrics: dict    # every table metric summed along the plan

    @property
    def feasible(self):
        return math.isfinite(self.cost)


def make_plan(table, cost, order):
    total, variants = best_variants(order, cost)
    return Plan(tuple(int(c) for c in order), tuple(variants), total,
                table.totals(order, variants))
