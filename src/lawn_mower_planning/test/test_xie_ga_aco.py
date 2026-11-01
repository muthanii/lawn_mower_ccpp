import math

import numpy as np
import pytest

from conftest import lawn_table
from lawn_mower_planning.brute_force import brute_force_order, brute_force_tour
from lawn_mower_planning.xie_ga_aco import (XieGaAcoParams, centroid_tour_length,
                                            inversion_mutation, length_turns_order, pmx,
                                            xie2024_order)

SMALL = dict(population=50, ants=25)  # the paper's 400 and 100 are slow for unit tests


class Cells:
    """Stand-in for a CoverageProblem: square cells at given centres, one lane each way."""

    def __init__(self, centres, start=None):
        from shapely.geometry import box
        self.cells = [box(x - 1, y - 1, x + 1, y + 1) for x, y in centres]
        self.routes = [[[(x - 1, y - 1), (x - 1, y + 1)], [(x - 1, y + 1), (x - 1, y - 1)],
                        [(x + 1, y - 1), (x + 1, y + 1)], [(x + 1, y + 1), (x + 1, y - 1)]]
                       for x, y in centres]
        self.start = start
        self.table = lawn_table(len(centres), seed=0)


def test_paper_parameters_are_the_defaults():
    p = XieGaAcoParams()
    assert (p.population, p.crossover_rate, p.mutation_rate) == (400, 0.5, 0.9)
    assert (p.ants, p.alpha, p.beta, p.deposit, p.mu) == (100, 8.0, 1.0, 100.0, 0.95)


def test_adaptive_alpha_beta_and_rho():
    p = XieGaAcoParams()
    early, late = p.alpha_beta(-6), p.alpha_beta(6)
    assert early[0] < late[0] and early[1] > late[1]       # pheromone gains weight
    assert late[0] == pytest.approx(p.alpha + p.mu * p.beta, abs=0.03)
    assert p.alpha_beta(0) == pytest.approx((p.alpha / 2 + p.mu * p.beta, p.beta / 2))
    assert p.rho(0) == pytest.approx(1 / math.sqrt(2 * math.pi) + 0.2)
    rhos = [p.rho(t) for t in range(0, 60, 5)]
    assert rhos == sorted(rhos, reverse=True) and rhos[-1] == pytest.approx(0.2)


@pytest.mark.parametrize('seed', range(8))
def test_finds_the_optimal_centre_tour(seed):
    centres = np.random.default_rng(seed).uniform(0, 50, (8, 2))
    dist = np.linalg.norm(centres[:, None] - centres[None, :], axis=-1)
    result = xie2024_order(Cells(centres), XieGaAcoParams(evaluations=4000, **SMALL), seed=seed)
    assert result.plan.cost == pytest.approx(brute_force_tour(dist)[0])
    assert centroid_tour_length(result.plan.order, dist) == pytest.approx(result.plan.cost)


def test_nearest_corner_bridging():
    # Two cells in a row; the robot starts left of the left one.
    result = xie2024_order(Cells([(10, 0), (2, 0)], start=(-5, -1)),
                           XieGaAcoParams(evaluations=50, **SMALL))
    assert result.plan.order == (1, 0)          # opened at the cell nearest the start
    assert result.plan.variants == (0, 1)       # bottom-left corner, then nearest top corner


@pytest.mark.parametrize('share', [0.0, 1.0])
def test_single_stage_variants_still_tour(share):
    centres = np.random.default_rng(3).uniform(0, 50, (8, 2))
    dist = np.linalg.norm(centres[:, None] - centres[None, :], axis=-1)
    result = xie2024_order(Cells(centres), XieGaAcoParams(evaluations=2000, ga_share=share,
                                                          **SMALL))
    assert sorted(result.plan.order) == list(range(8))
    assert result.plan.cost <= 1.15 * brute_force_tour(dist)[0]


@pytest.mark.parametrize('seed', range(5))
def test_length_turns_ablation_is_near_brute_force(seed):
    table = lawn_table(7, seed)
    params = XieGaAcoParams(evaluations=2000, turn_weight=2.0, **SMALL)
    best = brute_force_order(table, {'length': 1.0, 'turns': 2.0}).cost
    assert length_turns_order(table, params, seed=seed).plan.cost <= 1.01 * best


def test_same_seed_same_plan():
    table = lawn_table(12, seed=4)
    params = XieGaAcoParams(evaluations=800, **SMALL)
    a, b = (length_turns_order(table, params, seed=7) for _ in range(2))
    assert a.plan == b.plan and a.history == b.history


@pytest.mark.parametrize('budget, share', [(800, 0.5), (800, 1.0), (800, 0.0), (37, 0.5)])
def test_spends_exactly_the_budget(budget, share):
    result = length_turns_order(lawn_table(10, seed=2),
                                XieGaAcoParams(evaluations=budget, ga_share=share, **SMALL))
    assert result.evaluations == budget
    assert result.ga_evaluations == round(budget * share)
    assert result.history[-1][1] == pytest.approx(result.plan.cost)


def test_logs_energy():
    row = length_turns_order(lawn_table(6, seed=0, with_start=False),
                             XieGaAcoParams(evaluations=200, **SMALL)).row()
    assert {'length', 'turns', 'energy', 'cost', 'seconds', 'evaluations', 'order'} <= set(row)


@pytest.mark.parametrize('n', [1, 2])
def test_tiny_maps(n):
    plan = length_turns_order(lawn_table(n, seed=0), XieGaAcoParams(evaluations=50, **SMALL)).plan
    assert sorted(plan.order) == list(range(n)) and plan.feasible


def test_operators_return_permutations():
    rng = np.random.default_rng(0)
    for _ in range(300):
        n = int(rng.integers(2, 12))
        a, b = rng.permutation(n), rng.permutation(n)
        i, j = sorted(rng.choice(n + 1, size=2, replace=False))
        child = pmx(a, b, i, j)
        assert sorted(child) == list(range(n)) and (child[i:j] == a[i:j]).all()
        assert sorted(inversion_mutation(a, rng)) == list(range(n))


def test_rejects_bad_params():
    with pytest.raises(ValueError, match='ga_share'):
        XieGaAcoParams(ga_share=1.5)
    with pytest.raises(ValueError, match='mutation_rate'):
        XieGaAcoParams(mutation_rate=2.0)
