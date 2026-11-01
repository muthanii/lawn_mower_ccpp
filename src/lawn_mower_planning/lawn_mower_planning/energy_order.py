"""Our method: order the cells by modelled skid-steer energy, with a GA or a PSO.

Both searches run on the shared TransitionTable and minimise a weighted sum
of its metrics, by default energy alone ({'energy': 1}). Each candidate cell
order is decoded with the exact variant DP (transitions.best_variants), so
every order is scored with its best start corners, and every decode counts
against the budget `evaluations`, the same budget the baselines get.

    energy_ga_order    permutation GA: tournament selection, ordered
                       crossover (OX), inversion mutation, elitism, no
                       duplicate orders in a generation
    energy_pso_order   random-key PSO: a particle is a vector of n keys in
                       [0, 1] and the cell order is argsort(keys); ring
                       topology, Clerc's constriction coefficients, and a
                       particle that stops improving is drawn again

Against the Xie et al. baseline only the ordering changes: the cells, lanes,
A* paths and energy model are shared.
"""

import time
from dataclasses import dataclass

import numpy as np

from .operators import inversion_mutation, ordered_crossover
from .search import RunResult, Search
from .transitions import best_variants, make_plan

ENERGY = (('energy', 1.0),)
UNIQUE_TRIES = 10  # inversions tried to make a duplicate child new

# Clerc and Kennedy (2002) constriction: chi = 0.7298, chi * phi / 2 = 1.49618
CLERC_INERTIA = 0.7298
CLERC_ACCELERATION = 1.49618


@dataclass(frozen=True)
class EnergyGaParams:
    evaluations: int = 20000       # evaluated orders
    population: int = 50
    tournament: int = 3
    crossover_rate: float = 0.9
    mutation_rate: float = 0.3
    elites: int = 2                # best orders copied unchanged to the next generation
    unique: bool = True            # re-mutate a child that is already in the population
    weights: tuple = ENERGY        # objective as (metric, weight) pairs

    def __post_init__(self):
        problems = []
        if self.evaluations < 1:
            problems.append('evaluations must be >= 1')
        if self.population < 2 or self.tournament < 1:
            problems.append('population must be >= 2 and tournament >= 1')
        if not 0 <= self.elites < self.population:
            problems.append('elites must be in [0, population)')
        if not (0 <= self.crossover_rate <= 1 and 0 <= self.mutation_rate <= 1):
            problems.append('crossover_rate and mutation_rate must be in [0, 1]')
        if problems:
            raise ValueError('; '.join(problems))


@dataclass(frozen=True)
class EnergyPsoParams:
    evaluations: int = 20000       # evaluated orders
    particles: int = 60
    inertia: float = CLERC_INERTIA
    cognitive: float = CLERC_ACCELERATION   # pull to the particle's own best
    social: float = CLERC_ACCELERATION      # pull to the swarm's best
    max_velocity: float = 0.5      # per key, keys live in [0, 1]
    neighbours: int = 1            # ring topology: particles i - k .. i + k; 0 = global best
    restart: int = 20              # re-draw a particle whose own best has not improved for this
                                   # many of its moves; 0 = never
    weights: tuple = ENERGY        # objective as (metric, weight) pairs

    def __post_init__(self):
        problems = []
        if self.evaluations < 1:
            problems.append('evaluations must be >= 1')
        if self.particles < 2:
            problems.append('particles must be >= 2')
        if self.neighbours < 0 or self.restart < 0:
            problems.append('neighbours and restart must be >= 0')
        if self.max_velocity <= 0:
            problems.append('max_velocity must be > 0')
        if problems:
            raise ValueError('; '.join(problems))


def _table_search(table, weights, budget):
    cost = table.combine(dict(weights))
    return cost, Search(lambda order: best_variants(order, cost)[0], budget)


def _result(table, cost, search, t0):
    return RunResult(make_plan(table, cost, search.best_order), time.perf_counter() - t0,
                     search.used, history=search.history)


def _tournament(costs, size, rng):
    picks = rng.integers(len(costs), size=size)
    return picks[np.argmin(costs[picks])]


def energy_ga_order(table, params=EnergyGaParams(), seed=0):
    """Permutation GA on the table. Returns RunResult."""
    t0 = time.perf_counter()
    rng = np.random.default_rng(seed)
    n = table.n_cells
    cost, search = _table_search(table, params.weights, params.evaluations)
    if n == 1:
        search.evaluate(np.zeros(1, dtype=int))
        return _result(table, cost, search, t0)
    size = min(params.population, params.evaluations)
    pop = [rng.permutation(n) for _ in range(size)]
    costs = np.array([search.evaluate(p) for p in pop])
    while search.left > 0:
        rank = np.argsort(costs, kind='stable')
        new_pop = [pop[i] for i in rank[:params.elites]]
        new_costs = [costs[i] for i in rank[:params.elites]]
        while len(new_pop) < size and search.left > 0:
            a = pop[_tournament(costs, params.tournament, rng)]
            b = pop[_tournament(costs, params.tournament, rng)]
            child = ordered_crossover(a, b, rng) if rng.random() < params.crossover_rate \
                else a.copy()
            if rng.random() < params.mutation_rate:
                child = inversion_mutation(child, rng)
            if params.unique:
                seen = {tuple(p) for p in new_pop}
                for _ in range(UNIQUE_TRIES):
                    if tuple(child) not in seen:
                        break
                    child = inversion_mutation(child, rng)
            new_pop.append(child)
            new_costs.append(search.evaluate(child))
        pop, costs = new_pop, np.array(new_costs)
    return _result(table, cost, search, t0)


def energy_pso_order(table, params=EnergyPsoParams(), seed=0):
    """Random-key PSO on the table. Returns RunResult."""
    t0 = time.perf_counter()
    rng = np.random.default_rng(seed)
    n = table.n_cells
    cost, search = _table_search(table, params.weights, params.evaluations)
    if n == 1:
        search.evaluate(np.zeros(1, dtype=int))
        return _result(table, cost, search, t0)
    size = min(params.particles, params.evaluations)
    vmax = params.max_velocity
    keys = rng.random((size, n))
    velocity = rng.uniform(-vmax, vmax, (size, n))
    own_best = keys.copy()
    own_cost = np.array([search.evaluate(np.argsort(k, kind='stable')) for k in keys])
    stale = np.zeros(size, dtype=int)
    k = params.neighbours
    while search.left > 0:
        for i in range(size):
            if search.left == 0:
                break
            if k == 0 or 2 * k + 1 >= size:
                leader = own_best[np.argmin(own_cost)]
            else:
                ring = [(i + d) % size for d in range(-k, k + 1)]
                leader = own_best[ring[int(np.argmin(own_cost[ring]))]]
            r1, r2 = rng.random(n), rng.random(n)
            velocity[i] = (params.inertia * velocity[i]
                           + params.cognitive * r1 * (own_best[i] - keys[i])
                           + params.social * r2 * (leader - keys[i]))
            np.clip(velocity[i], -vmax, vmax, out=velocity[i])
            keys[i] = np.clip(keys[i] + velocity[i], 0.0, 1.0)
            c = search.evaluate(np.argsort(keys[i], kind='stable'))
            if c < own_cost[i]:
                own_best[i], own_cost[i], stale[i] = keys[i].copy(), c, 0
            else:
                stale[i] += 1
            if params.restart and stale[i] >= params.restart:
                keys[i] = rng.random(n)
                velocity[i] = rng.uniform(-vmax, vmax, n)
                stale[i] = 0
    return _result(table, cost, search, t0)
