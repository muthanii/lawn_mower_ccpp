"""Shared bookkeeping for the cell-ordering searches: budget and result."""

from dataclasses import dataclass, field

import numpy as np

from .transitions import Plan


@dataclass(frozen=True)
class RunResult:
    plan: Plan               # every table metric along the plan, energy included
    seconds: float           # wall-clock time of the search
    evaluations: int         # evaluated orders, all stages together
    ga_evaluations: int = 0  # evaluations of a GA stage, where there is one
    history: list = field(default_factory=list)  # (evaluation, best cost) at each improvement

    def row(self):
        """Flat record for the experiment log."""
        out = {'cost': self.plan.cost, 'seconds': self.seconds,
               'evaluations': self.evaluations, 'ga_evaluations': self.ga_evaluations,
               'order': ' '.join(map(str, self.plan.order)),
               'variants': ' '.join(map(str, self.plan.variants))}
        out.update(self.plan.metrics)
        return out


class Search:
    """Evaluates orders against a budget and keeps the best one.

    Every call to evaluate counts as one evaluation, so methods given the
    same budget do the same number of objective evaluations.
    """

    def __init__(self, cost, budget):
        self.cost = cost  # order -> float
        self.budget = budget
        self.used = 0
        self.best_cost = np.inf
        self.best_order = None
        self.history = []

    @property
    def left(self):
        return self.budget - self.used

    def evaluate(self, order):
        total = self.cost(order)
        self.used += 1
        if total < self.best_cost or self.best_order is None:
            self.best_cost, self.best_order = total, np.array(order)
            self.history.append((self.used, total))
        return total
