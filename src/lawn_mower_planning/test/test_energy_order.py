import pytest

from conftest import lawn_table
from lawn_mower_planning.brute_force import brute_force_order
from lawn_mower_planning.energy_order import (EnergyGaParams, EnergyPsoParams, energy_ga_order,
                                              energy_pso_order)

ENERGY = {'energy': 1.0}


@pytest.mark.parametrize('seed', range(5))
def test_ga_finds_the_energy_optimum(seed):
    # 2000 evaluations, against 7! = 5040 orders for brute force.
    table = lawn_table(7, seed)
    best = brute_force_order(table, ENERGY).cost
    assert energy_ga_order(table, EnergyGaParams(evaluations=2000), seed=seed).plan.cost \
        == pytest.approx(best)


@pytest.mark.parametrize('seed', range(5))
def test_pso_is_near_the_energy_optimum(seed):
    table = lawn_table(7, seed)
    best = brute_force_order(table, ENERGY).cost
    assert energy_pso_order(table, EnergyPsoParams(evaluations=2000), seed=seed).plan.cost \
        <= 1.01 * best


@pytest.mark.parametrize('order_fn, params', [
    (energy_ga_order, EnergyGaParams(evaluations=500)),
    (energy_pso_order, EnergyPsoParams(evaluations=500)),
    (energy_pso_order, EnergyPsoParams(evaluations=500, neighbours=0, restart=0)),
])
def test_budget_seed_and_metrics(order_fn, params):
    table = lawn_table(10, seed=2)
    a, b = order_fn(table, params, seed=3), order_fn(table, params, seed=3)
    assert a.evaluations == params.evaluations
    assert a.plan == b.plan and a.history == b.history
    assert a.plan.cost == pytest.approx(a.plan.metrics['energy'])
    assert sorted(a.plan.order) == list(range(10))


def test_other_objectives():
    table = lawn_table(6, seed=1)
    weights = (('length', 1.0), ('turns', 2.0))
    plan = energy_ga_order(table, EnergyGaParams(evaluations=1500, weights=weights)).plan
    assert plan.cost == pytest.approx(brute_force_order(table, dict(weights)).cost)


@pytest.mark.parametrize('n', [1, 2])
@pytest.mark.parametrize('order_fn, params', [(energy_ga_order, EnergyGaParams(evaluations=30)),
                                              (energy_pso_order, EnergyPsoParams(evaluations=30))])
def test_tiny_maps(n, order_fn, params):
    plan = order_fn(lawn_table(n, seed=0), params).plan
    assert sorted(plan.order) == list(range(n)) and plan.feasible


def test_rejects_bad_params():
    with pytest.raises(ValueError, match='elites'):
        EnergyGaParams(population=10, elites=10)
    with pytest.raises(ValueError, match='particles'):
        EnergyPsoParams(particles=1)
