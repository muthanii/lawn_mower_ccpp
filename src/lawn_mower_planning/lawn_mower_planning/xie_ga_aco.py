"""Re-implementation of the Xie et al. (2024) HGACO cell ordering (baseline).

Xie, Yan, Zhang, Qin, Jin and Xu (2024), "Hybrid genetic ant colony
optimization algorithm for full-coverage path planning of gardening pruning
robots", Intelligent Service Robotics 17, 661-683,
https://doi.org/10.1007/s11370-024-00525-6

Xie et al. order the sub-regions as a closed travelling salesman tour over
the sub-region centre points, with Euclidean distances (their Eqs. 1-3).
Turns and coverage are only measured afterwards; they are not in the
objective. The tour is found by HGACO (their Sec. 3.2):

    GA     population N, PMX crossover (P_c), inversion mutation (P_m),
           fitness = 1 / tour length
    hand-  low initial pheromone everywhere, high on the edges of the GA's
    over   best tour
    ACO    ant system: m ants, p_ij ~ tau^alpha' eta^beta', eta = 1 / d_ij,
           every ant deposits Q / L_k (Eqs. 4-7), with
               alpha' = alpha sigmoid(x) + mu beta          (Eq. 9)
               beta'  = beta (1 - sigmoid(x))               (Eq. 10)
               rho(t) = N(t / 10; mean, sigma) + 0.2        (Eqs. 11-12)

Then each cell is entered at the corner nearest to where the robot is
(their Sec. 3.3), and the shared pipeline's A* paths join the cells.

xie2024_order does all of this on a CoverageProblem. length_turns_order runs
the same HGACO on the shared TransitionTable instead, minimising
length + turn_weight * turns with the exact variant DP: an ablation, not
Xie et al.'s method. Every evaluated order counts against the budget
`evaluations` in both, so all methods can get the same budget.

Where the paper leaves a value or step open, the choice made here is listed
in the package README. ga_share = 1 gives the GA-only fallback.

The GA operators are written here instead of taken from DEAP: DEAP draws from
the global `random` module, and the ROS rosdep index has no python3-deap key
for CI. A local NumPy generator keeps every run reproducible from its seed.
"""

import math
import time
from dataclasses import dataclass

import numpy as np

from .operators import inversion_mutation, pmx
from .search import RunResult, Search
from .transitions import Plan, best_variants, make_plan

# Floor for step costs in the ACO heuristic, as a share of the median step,
# so that a zero-length step does not swamp the pheromone.
STEP_FLOOR = 0.01


@dataclass(frozen=True)
class XieGaAcoParams:
    # Budget, shared with the other methods
    evaluations: int = 20000       # evaluated orders, GA and ACO together
    ga_share: float = 0.5          # share of the budget for the GA; 1 = GA only, 0 = ACO only
    # GA, Xie et al. Table 2 (G_max is replaced by the budget)
    population: int = 400          # N
    crossover_rate: float = 0.5    # P_c
    mutation_rate: float = 0.9     # P_m
    # ACO, Xie et al. Table 1 (the iteration count is replaced by the budget)
    ants: int = 100                # m
    alpha: float = 8.0
    beta: float = 1.0
    deposit: float = 100.0         # Q
    mu: float = 0.95
    # Not given in the paper (see README)
    sigmoid_span: float = 6.0      # x runs from -span to +span over the ACO iterations
    rho_mean: float = 0.0          # y in Eq. 11
    rho_sigma: float = 1.0         # sigma in Eq. 11
    tau_low: float = 0.5           # initial pheromone after a GA stage
    tau_ga: float = 2.0            # initial pheromone on the GA's best tour
    tau_plain: float = 1.0         # initial pheromone without a GA stage
    # Ablation only (length_turns_order)
    turn_weight: float = 1.0       # w_T: cost of one turn, in metres of path

    def __post_init__(self):
        problems = []
        if self.evaluations < 1:
            problems.append('evaluations must be >= 1')
        if not 0 <= self.ga_share <= 1:
            problems.append('ga_share must be in [0, 1]')
        if self.population < 2 or self.ants < 1:
            problems.append('population must be >= 2 and ants >= 1')
        if not (0 <= self.crossover_rate <= 1 and 0 <= self.mutation_rate <= 1):
            problems.append('crossover_rate and mutation_rate must be in [0, 1]')
        if min(self.tau_low, self.tau_ga, self.tau_plain, self.rho_sigma) <= 0:
            problems.append('pheromone levels and rho_sigma must be > 0')
        if problems:
            raise ValueError('; '.join(problems))

    def alpha_beta(self, x):
        """Eqs. 9-10 at sigmoid step x."""
        s = 1 / (1 + math.exp(-x))
        return self.alpha * s + self.mu * self.beta, self.beta * (1 - s)

    def rho(self, t):
        """Eqs. 11-12: Gaussian density at t / 10, plus 0.2, at ACO iteration t."""
        z = (t / 10 - self.rho_mean) / self.rho_sigma
        return math.exp(-z * z / 2) / (math.sqrt(2 * math.pi) * self.rho_sigma) + 0.2


@dataclass(frozen=True)
class _Objective:
    """What HGACO searches: cost of an order and the ACO heuristic.

    eta has a start row n. A closed tour has no start, so its start row is
    uniform and holds no pheromone; its edges are symmetric and the closing
    edge counts.
    """

    cost: object       # order -> float
    eta: np.ndarray    # (n + 1, n)
    closed: bool


class _Search(Search):
    """Search on an _Objective, which also carries the ACO heuristic."""

    def __init__(self, objective, budget):
        super().__init__(objective.cost, budget)
        self.objective = objective


def _roulette(costs, count, rng):
    """Indices drawn with probability proportional to fitness 1 / cost."""
    ok = np.isfinite(costs) & (costs > 0)
    fitness = np.zeros(len(costs))
    fitness[ok] = 1 / costs[ok]
    if fitness.sum() <= 0:
        return rng.integers(len(costs), size=count)
    return rng.choice(len(costs), size=count, p=fitness / fitness.sum())


def _genetic_stage(search, n, params, budget, rng):
    """Xie et al.'s GA. Returns the best order found by it."""
    size = min(params.population, budget)
    pop = [rng.permutation(n) for _ in range(size)]
    costs = np.array([search.evaluate(p) for p in pop])
    used = size
    while used < budget and size >= 2:
        start = used
        best = int(np.argmin(costs))
        new_pop, new_costs = [pop[best]], [costs[best]]  # keep the best (elitism)
        parents = _roulette(costs, size + 1, rng)
        for a, b in zip(parents[::2], parents[1::2]):
            if rng.random() < params.crossover_rate:
                i, j = sorted(rng.choice(n + 1, size=2, replace=False))
                kids = [(pmx(pop[a], pop[b], i, j), None), (pmx(pop[b], pop[a], i, j), None)]
            else:
                kids = [(pop[a].copy(), costs[a]), (pop[b].copy(), costs[b])]
            for child, cost in kids:
                if len(new_pop) == size or used >= budget:
                    break
                if rng.random() < params.mutation_rate:
                    child, cost = inversion_mutation(child, rng), None
                if cost is None:
                    cost = search.evaluate(child)
                    used += 1
                new_pop.append(child)
                new_costs.append(cost)
        pop, costs = new_pop, np.array(new_costs)
        if used == start:  # P_c = P_m = 0: nothing new can appear
            break
    return pop[int(np.argmin(costs))]


def _edges(order, n, closed):
    """Pheromone entries (rows, columns) of an order; row n is the start of an open path."""
    order = np.asarray(order)
    if closed:
        return order, np.roll(order, -1)
    return np.concatenate(([n], order[:-1])), order


def _deposit(tau, order, amount, n, closed):
    rows, cols = _edges(order, n, closed)
    np.add.at(tau, (rows, cols), amount)
    if closed:
        np.add.at(tau, (cols, rows), amount)


def _ant_stage(search, n, params, ga_best, rng):
    """Xie et al.'s improved ant system, seeded with the GA's best order."""
    objective = search.objective
    if ga_best is None:
        tau = np.full((n + 1, n), params.tau_plain)
    else:
        tau = np.full((n + 1, n), params.tau_low)
        rows, cols = _edges(ga_best, n, objective.closed)
        tau[rows, cols] = params.tau_ga
        if objective.closed:
            tau[cols, rows] = params.tau_ga
    if objective.closed:
        tau[n] = 1.0
    log_eta = np.log(np.where(objective.eta > 0, objective.eta, np.nan))
    iterations = math.ceil(search.left / params.ants)
    for t in range(iterations):
        x = 0.0 if iterations == 1 else params.sigmoid_span * (2 * t / (iterations - 1) - 1)
        alpha, beta = params.alpha_beta(x)
        tours = []
        for _ in range(min(params.ants, search.left)):
            order, row = np.empty(n, dtype=int), n
            free = np.ones(n, dtype=bool)
            for step in range(n):
                score = alpha * np.log(tau[row]) + beta * log_eta[row]
                score = np.where(free & np.isfinite(score), score, -np.inf)
                if np.isfinite(score).any():
                    p = np.exp(score - score.max())
                else:
                    p = free.astype(float)
                row = order[step] = rng.choice(n, p=p / p.sum())
                free[row] = False
            tours.append((order, search.evaluate(order)))
        tau *= 1 - params.rho(t)
        for order, cost in tours:
            if np.isfinite(cost) and cost > 0:
                _deposit(tau, order, params.deposit / cost, n, objective.closed)
        if objective.closed:
            tau[n] = 1.0
        np.maximum(tau, np.finfo(float).tiny, out=tau)


def _floored_inverse(steps, n):
    """eta = 1 / step cost, off the diagonal; zero steps are floored."""
    off_diagonal = ~np.vstack((np.eye(n, dtype=bool), np.zeros((1, n), dtype=bool)))
    usable = steps[off_diagonal & np.isfinite(steps) & (steps > 0)]
    floor = STEP_FLOOR * np.median(usable) if usable.size else 1.0
    return np.where(off_diagonal, 1.0 / np.maximum(steps, floor), 0.0)


def _hgaco(objective, n, params, seed):
    """Run GA then ACO on an objective. Returns (search, ga evaluations, seconds)."""
    t0 = time.perf_counter()
    rng = np.random.default_rng(seed)
    search = _Search(objective, params.evaluations)
    ga_budget = 0
    if n == 1:
        search.evaluate(np.zeros(1, dtype=int))
    else:
        ga_budget = int(round(params.evaluations * params.ga_share))
        ga_best = _genetic_stage(search, n, params, ga_budget, rng) if ga_budget > 0 else None
        if search.left > 0:
            _ant_stage(search, n, params, ga_best, rng)
    return search, min(ga_budget, search.used), time.perf_counter() - t0


def centroid_tour_length(order, dist):
    """Closed tour length over the cell centre points (Xie et al. Eq. 1)."""
    order = np.asarray(order)
    return float(dist[order, np.roll(order, -1)].sum())


def nearest_corner_plan(problem, tour):
    """Turn a closed tour into a plan the way Xie et al. bridge cells (Sec. 3.3).

    The tour is opened at the cell nearest to the robot start (or at its first
    cell without a start). Each cell is then entered in the variant whose
    entry corner is nearest to where the robot is; the lane parity fixes the
    exit corner.
    """
    tour = [int(c) for c in tour]
    entry = [[r[0] for r in routes] for routes in problem.routes]
    centre = [c.centroid.coords[0] for c in problem.cells]
    if problem.start is not None:
        first = min(tour, key=lambda c: math.dist(centre[c], problem.start))
        k = tour.index(first)
        tour = tour[k:] + tour[:k]
    here, variants = problem.start, []
    for c in tour:
        a = 0 if here is None else min(range(len(entry[c])),
                                       key=lambda v: math.dist(entry[c][v], here))
        variants.append(a)
        here = problem.routes[c][a][-1]
    return tour, variants


def xie2024_order(problem, params=XieGaAcoParams(), seed=0):
    """Xie et al.'s method on a CoverageProblem: HGACO tour over cell centres,
    nearest-corner bridging. Plan.cost is the centre tour length."""
    centre = np.array([c.centroid.coords[0] for c in problem.cells])
    n = len(centre)
    dist = np.linalg.norm(centre[:, None] - centre[None, :], axis=-1)
    eta = _floored_inverse(np.vstack((dist, np.ones((1, n)))), n)
    eta[n] = 1.0  # a closed tour starts anywhere
    objective = _Objective(lambda order: centroid_tour_length(order, dist), eta, closed=True)
    search, ga_used, seconds = _hgaco(objective, n, params, seed)
    order, variants = nearest_corner_plan(problem, search.best_order)
    plan = Plan(tuple(order), tuple(variants), search.best_cost,
                problem.table.totals(order, variants))
    return RunResult(plan, seconds, search.used, ga_used, search.history)


def length_turns_order(table, params=XieGaAcoParams(), seed=0):
    """Ablation: the same HGACO on the shared table, minimising
    length + turn_weight * turns with the exact variant DP."""
    n = table.n_cells
    cost = table.combine({'length': 1.0, 'turns': params.turn_weight})
    steps = np.vstack((cost.edge_lower_bounds(), cost.start_lower_bounds()[None, :]))
    objective = _Objective(lambda order: best_variants(order, cost)[0],
                           _floored_inverse(steps, n), closed=False)
    search, ga_used, seconds = _hgaco(objective, n, params, seed)
    return RunResult(make_plan(table, cost, search.best_order), seconds, search.used, ga_used,
                     search.history)
