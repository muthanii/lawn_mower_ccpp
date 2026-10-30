import itertools

import numpy as np
import pytest

from conftest import lawn_table
from lawn_mower_planning.brute_force import brute_force_order
from lawn_mower_planning.transitions import TransitionTable, best_variants, make_plan


def test_rejects_inconsistent_tables(table):
    with pytest.raises(ValueError, match='transition metrics'):
        TransitionTable(table.cell, {'length': table.transition['length']})
    bad = dict(table.transition, turns=np.zeros((2, 2)))
    with pytest.raises(ValueError, match="transition\\['turns'\\] has shape"):
        TransitionTable(table.cell, bad, table.start)
    with pytest.raises(ValueError, match='unknown metrics'):
        table.combine({'time': 1.0})


@pytest.mark.parametrize('seed', range(5))
def test_variant_dp_matches_exhaustive_search(seed):
    table = lawn_table(5, seed)
    cost = table.combine({'length': 1.0, 'turns': 2.0})
    order = list(np.random.default_rng(seed).permutation(5))
    total, variants = best_variants(order, cost)
    best = min(
        (sum(cost.cell[c, a] for c, a in zip(order, vs)) + cost.start[order[0], vs[0]]
         + sum(cost.transition[i, a, j, b] for i, a, j, b in zip(order, vs, order[1:], vs[1:])), vs)
        for vs in itertools.product(range(table.n_variants), repeat=5))
    assert total == pytest.approx(best[0])
    assert tuple(variants) == best[1]


def test_plan_metrics_add_up_to_the_objective(table):
    plan = make_plan(table, table.combine({'length': 1.0, 'turns': 3.0}), [2, 0, 4, 1, 5, 3])
    assert plan.cost == pytest.approx(plan.metrics['length'] + 3.0 * plan.metrics['turns'])
    assert set(plan.metrics) == {'energy', 'length', 'turns'}


def test_unusable_variants_are_avoided(table):
    cell = {m: v.copy() for m, v in table.cell.items()}
    cell['length'][:, 1:] = np.inf
    limited = TransitionTable(cell, table.transition, table.start)
    plan = make_plan(limited, limited.combine({'length': 1.0}), range(6))
    assert plan.variants == (0,) * 6 and plan.feasible


def test_brute_force_is_optimal_and_limited():
    table = lawn_table(5, seed=3)
    weights = {'length': 1.0, 'turns': 1.0}
    plan = brute_force_order(table, weights)
    cost = table.combine(weights)
    assert all(plan.cost <= best_variants(o, cost)[0] + 1e-9
               for o in itertools.permutations(range(5)))
    with pytest.raises(ValueError, match='limited'):
        brute_force_order(lawn_table(10, seed=0), weights)
