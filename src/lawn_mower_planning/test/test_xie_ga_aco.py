import numpy as np
import pytest

from conftest import lawn_table
from lawn_mower_planning.brute_force import brute_force_order
from lawn_mower_planning.xie_ga_aco import (XieGaAcoParams, inversion_mutation,
                                            ordered_crossover, xie_ga_aco_order)


@pytest.mark.parametrize('seed', range(10))
def test_matches_brute_force_on_seven_cells(seed):
    # 1000 decoded orders is a fifth of the 7! = 5040 an exhaustive search needs.
    table = lawn_table(7, seed)
    params = XieGaAcoParams(evaluations=1000, turn_weight=2.0)
    assert xie_ga_aco_order(table, params, seed=seed).plan.cost == pytest.approx(
        brute_force_order(table, params.weights).cost)


@pytest.mark.parametrize('share', [0.0, 1.0])
def test_single_stage_variants_stay_close_to_optimal(share):
    # ga_share = 1 is the GA-only fallback of the issue; 0 is ACO alone.
    for seed in range(5):
        table = lawn_table(7, seed)
        params = XieGaAcoParams(evaluations=1000, ga_share=share, turn_weight=2.0)
        best = brute_force_order(table, params.weights).cost
        assert xie_ga_aco_order(table, params, seed=seed).plan.cost <= 1.02 * best


def test_same_seed_same_plan():
    table = lawn_table(12, seed=4)
    a, b = (xie_ga_aco_order(table, XieGaAcoParams(evaluations=800), seed=7) for _ in range(2))
    assert a.plan == b.plan and a.history == b.history


@pytest.mark.parametrize('budget, share', [(800, 0.5), (800, 1.0), (800, 0.0), (37, 0.5)])
def test_spends_exactly_the_budget(budget, share):
    result = xie_ga_aco_order(lawn_table(10, seed=2),
                              XieGaAcoParams(evaluations=budget, ga_share=share), seed=0)
    assert result.evaluations == budget
    assert result.ga_evaluations == round(budget * share)
    assert [e for e, _ in result.history] == sorted(e for e, _ in result.history)
    assert result.history[-1][1] == pytest.approx(result.plan.cost)


def test_turn_weight_trades_length_for_turns():
    table = lawn_table(9, seed=5)
    short = xie_ga_aco_order(table, XieGaAcoParams(evaluations=1500, turn_weight=0.0), seed=0)
    few_turns = xie_ga_aco_order(table, XieGaAcoParams(evaluations=1500, turn_weight=50.0), seed=0)
    assert short.plan.metrics['length'] <= few_turns.plan.metrics['length']
    assert few_turns.plan.metrics['turns'] <= short.plan.metrics['turns']


def test_logs_energy_and_free_start():
    table = lawn_table(6, seed=0, with_start=False)
    row = xie_ga_aco_order(table, XieGaAcoParams(evaluations=200), seed=0).row()
    assert {'length', 'turns', 'energy', 'cost', 'seconds', 'evaluations', 'order'} <= set(row)
    assert sorted(map(int, row['order'].split())) == list(range(6))


@pytest.mark.parametrize('n', [1, 2])
def test_tiny_maps(n):
    plan = xie_ga_aco_order(lawn_table(n, seed=0), XieGaAcoParams(evaluations=50), seed=0).plan
    assert sorted(plan.order) == list(range(n)) and plan.feasible


def test_operators_return_permutations():
    rng = np.random.default_rng(0)
    for _ in range(200):
        n = rng.integers(2, 12)
        a, b = rng.permutation(n), rng.permutation(n)
        for child in (ordered_crossover(a, b, rng), inversion_mutation(a, rng)):
            assert sorted(child) == list(range(n))


def test_rejects_bad_params():
    with pytest.raises(ValueError, match='ga_share'):
        XieGaAcoParams(ga_share=1.5)
    with pytest.raises(ValueError, match='evaporation'):
        XieGaAcoParams(evaporation=0.0)
