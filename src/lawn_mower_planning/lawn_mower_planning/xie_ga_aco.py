"""Re-implementation of the Xie et al. (2024) GA-ACO cell ordering (baseline).

Xie, Yan, Zhang, Qin, Jin and Xu (2024), "Hybrid genetic ant colony
optimization algorithm for full-coverage path planning of gardening pruning
robots", Intelligent Service Robotics 17, 661-683,
https://doi.org/10.1007/s11370-024-00525-6

Xie et al. order the sub-areas with a hybrid genetic ant colony algorithm and
join them with A*. Their objective is path length and turn count; they do not
model energy. Here only the ordering changes against our method: the cells,
lanes and A* transitions come from the shared TransitionTable.

    objective  J = length + turn_weight * turns
    stage 1    GA over cell permutations: tournament selection, ordered
               crossover (OX), inversion mutation, elitism
    handover   the best GA orders lay the initial pheromone
    stage 2    max-min ant system: ants build orders with probability
               tau^alpha * eta^beta, eta = 1 / cheapest transition; global-best
               deposit, evaporation, pheromone kept in [tau_min, tau_max]

Each order is decoded with the exact variant DP (transitions.best_variants),
and every decode counts against the compute budget `evaluations`, so the
baseline and our method can be given the same budget.

The full text was not available when this was written, so the operators,
the handover and the parameter values are an interpretation (see the package
README). ga_share = 1 gives the GA-only fallback.

The GA operators are written here instead of taken from DEAP: DEAP draws from
the global `random` module, and the ROS rosdep index has no python3-deap key
for CI. A local NumPy generator keeps every run reproducible from its seed.
"""

import time
from dataclasses import dataclass, field

import numpy as np

from .transitions import best_variants, make_plan

# Floor for step costs in the ACO heuristic, as a share of the median step,
# so that a zero-length transition does not swamp the pheromone.
STEP_FLOOR = 0.01


@dataclass(frozen=True)
class XieGaAcoParams:
    turn_weight: float = 1.0       # w_T: objective cost of one turn, in metres of path
    evaluations: int = 5000        # compute budget: decoded orders, GA and ACO together
    ga_share: float = 0.5          # share of the budget for the GA; 1 = GA only, 0 = ACO only
    population: int = 50
    tournament: int = 3
    crossover_rate: float = 0.9
    mutation_rate: float = 0.2
    elites: int = 2                # best orders copied unchanged to the next generation
    handover_elites: int = 10      # best GA orders that lay the initial pheromone
    ants: int = 20
    alpha: float = 1.0             # pheromone exponent
    beta: float = 2.0              # heuristic exponent
    evaporation: float = 0.1       # rho

    def __post_init__(self):
        problems = []
        if self.evaluations < 1:
            problems.append('evaluations must be >= 1')
        if not 0 <= self.ga_share <= 1:
            problems.append('ga_share must be in [0, 1]')
        if self.population < 2 or self.ants < 1 or self.tournament < 1:
            problems.append('population must be >= 2, ants and tournament >= 1')
        if not 0 <= self.elites < self.population:
            problems.append('elites must be in [0, population)')
        if self.handover_elites < 1:
            problems.append('handover_elites must be >= 1')
        if not 0 < self.evaporation < 1:
            problems.append('evaporation must be in (0, 1)')
        if problems:
            raise ValueError('; '.join(problems))

    @property
    def weights(self):
        """Objective as table metric weights."""
        return {'length': 1.0, 'turns': self.turn_weight}


@dataclass(frozen=True)
class RunResult:
    plan: object             # transitions.Plan with every table metric, energy included
    seconds: float           # wall-clock time of the search
    evaluations: int         # decoded orders, GA and ACO together
    ga_evaluations: int
    history: list = field(default_factory=list)  # (evaluation, best cost) at each improvement

    def row(self):
        """Flat record for the experiment log."""
        out = {'cost': self.plan.cost, 'seconds': self.seconds,
               'evaluations': self.evaluations, 'ga_evaluations': self.ga_evaluations,
               'order': ' '.join(map(str, self.plan.order)),
               'variants': ' '.join(map(str, self.plan.variants))}
        out.update(self.plan.metrics)
        return out


class _Search:
    """Counts evaluations and keeps the best order found."""

    def __init__(self, cost, budget):
        self.cost = cost
        self.budget = budget
        self.used = 0
        self.best_cost = np.inf
        self.best_order = None
        self.history = []

    @property
    def left(self):
        return self.budget - self.used

    def evaluate(self, order):
        total, _ = best_variants(order, self.cost)
        self.used += 1
        if total < self.best_cost or self.best_order is None:
            self.best_cost, self.best_order = total, tuple(int(c) for c in order)
            self.history.append((self.used, total))
        return total


def ordered_crossover(p1, p2, rng):
    """OX: keep a slice of p1, fill the rest in p2's order, starting after the slice."""
    n = len(p1)
    i, j = sorted(rng.choice(n + 1, size=2, replace=False))
    child = np.full(n, -1)
    child[i:j] = p1[i:j]
    kept = set(p1[i:j].tolist())
    fill = [c for c in np.roll(p2, -j) if c not in kept]
    child[np.roll(np.arange(n), -j)[:len(fill)]] = fill
    return child


def inversion_mutation(order, rng):
    """Reverse a random segment (a 2-opt move on the cell order)."""
    n = len(order)
    i, j = sorted(rng.choice(n + 1, size=2, replace=False))
    out = order.copy()
    out[i:j] = out[i:j][::-1]
    return out


def _tournament(costs, size, rng):
    picks = rng.integers(len(costs), size=size)
    return picks[np.argmin(costs[picks])]


def _genetic_stage(search, n, params, budget, rng):
    """Return the GA population and costs, best first."""
    size = min(params.population, budget)
    pop = [rng.permutation(n) for _ in range(size)]
    costs = np.array([search.evaluate(p) for p in pop])
    used = size
    while used < budget and size >= 2:
        rank = np.argsort(costs, kind='stable')
        new_pop = [pop[i] for i in rank[:params.elites]]
        new_costs = [costs[i] for i in rank[:params.elites]]
        while len(new_pop) < size and used < budget:
            a = pop[_tournament(costs, params.tournament, rng)]
            b = pop[_tournament(costs, params.tournament, rng)]
            child = ordered_crossover(a, b, rng) if rng.random() < params.crossover_rate else a.copy()
            if rng.random() < params.mutation_rate:
                child = inversion_mutation(child, rng)
            new_pop.append(child)
            new_costs.append(search.evaluate(child))
            used += 1
        pop, costs = new_pop, np.array(new_costs)
    rank = np.argsort(costs, kind='stable')
    return [pop[i] for i in rank], costs[rank]


def _greedy_order(eta, n):
    """Nearest-neighbour order on the heuristic, used to size the pheromone without a GA."""
    order, row, free = [], n, set(range(n))
    while free:
        nxt = max(free, key=lambda c: eta[row, c])
        order.append(nxt)
        free.remove(nxt)
        row = nxt
    return np.array(order)


def _edges(order, n):
    """Pheromone entries (row, column) used by an order; row n is the start."""
    rows = np.concatenate(([n], order[:-1]))
    return rows, np.asarray(order)


def _pheromone_bounds(best_cost, n, params):
    tau_max = 1.0 / (params.evaporation * best_cost) if np.isfinite(best_cost) and best_cost > 0 \
        else 1.0
    return tau_max / (2 * n), tau_max


def _ant_stage(search, n, params, eta, seeds, rng):
    """Max-min ant system seeded with the GA orders (seeds: list of (order, cost))."""
    tau_min, tau_max = _pheromone_bounds(search.best_cost, n, params)
    if seeds:
        tau = np.full((n + 1, n), tau_min)
        for order, cost in seeds:
            if np.isfinite(cost) and cost > 0:
                tau[_edges(order, n)] += 1.0 / (params.evaporation * cost * len(seeds))
        tau = np.clip(tau, tau_min, tau_max)
    else:
        tau = np.full((n + 1, n), tau_max)
    weight = eta ** params.beta
    while search.left > 0:
        for _ in range(min(params.ants, search.left)):
            order, row = np.empty(n, dtype=int), n
            free = np.ones(n, dtype=bool)
            for step in range(n):
                p = np.where(free, tau[row] ** params.alpha * weight[row], 0.0)
                total = p.sum()
                p = p / total if total > 0 else free / free.sum()
                row = order[step] = rng.choice(n, p=p)
                free[row] = False
            search.evaluate(order)
        tau *= 1 - params.evaporation
        best = np.array(search.best_order)
        if np.isfinite(search.best_cost) and search.best_cost > 0:
            tau[_edges(best, n)] += 1.0 / search.best_cost
        tau_min, tau_max = _pheromone_bounds(search.best_cost, n, params)
        np.clip(tau, tau_min, tau_max, out=tau)


def _heuristic(cost, n):
    """eta[row, j] = 1 / cheapest transition to cell j; row n is the start.

    Covering a cell costs the same in every order (up to the variant), so only
    the transitions inform the next step.
    """
    steps = np.vstack((cost.edge_lower_bounds(), cost.start_lower_bounds()[None, :]))
    off_diagonal = ~np.vstack((np.eye(n, dtype=bool), np.zeros((1, n), dtype=bool)))
    usable = steps[off_diagonal & np.isfinite(steps) & (steps > 0)]
    floor = STEP_FLOOR * np.median(usable) if usable.size else 1.0
    eta = np.where(off_diagonal, 1.0 / np.maximum(steps, floor), 0.0)
    return eta


def xie_ga_aco_order(table, params=XieGaAcoParams(), seed=0):
    """Order the cells of table under Xie et al.'s objective. Returns RunResult."""
    t0 = time.perf_counter()
    rng = np.random.default_rng(seed)
    n = table.n_cells
    cost = table.combine(params.weights)
    search = _Search(cost, params.evaluations)
    ga_budget = 0
    if n == 1:
        search.evaluate(np.zeros(1, dtype=int))
    else:
        ga_budget = int(round(params.evaluations * params.ga_share))
        seeds = []
        if ga_budget > 0:
            pop, costs = _genetic_stage(search, n, params, ga_budget, rng)
            seeds = list(zip(pop, costs))[:params.handover_elites]
        eta = _heuristic(cost, n)
        if search.left > 0:
            if not seeds:
                search.evaluate(_greedy_order(eta, n))
            _ant_stage(search, n, params, eta, seeds, rng)
    plan = make_plan(table, cost, search.best_order)
    return RunResult(plan, time.perf_counter() - t0, search.used, min(ga_budget, search.used),
                     search.history)
