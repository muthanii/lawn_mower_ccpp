"""Tune every ordering method with the same effort (issue #21 fairness rule).

    ros2 run lawn_mower_planning run_tuning --out tuning.csv

Protocol, the same for each method:

- 10 tuning lawns (random_lawn seeds 100-109, 6-7 cells). They are not
  among the shared test lawns, so tuning does not see the test data.
- Each method is scored against the exact optimum of its own objective:
  energy for energy_ga and energy_pso, the centroid tour for xie2024,
  length + turns for xie_length_turns.
- Each method gets GRID_SIZE settings, the first being its default before
  tuning, at budgets 1000 and 2000 with seeds 0-2: 60 runs per setting.
- A setting scores the number of runs that reach the optimum; ties go to
  the lower mean relative gap (to GAP_DIGITS), then to the earlier setting,
  so a full tie keeps the default. The best setting becomes the default.
"""

import argparse
import csv
import sys
from dataclasses import replace

import numpy as np

from .brute_force import brute_force_order, brute_force_tour
from .energy_order import EnergyGaParams, EnergyPsoParams, energy_ga_order, energy_pso_order
from .maps import DEFAULT_WIDTH, random_lawn
from .path_cost import PathCost
from .problem import build_problem
from .xie_ga_aco import XieGaAcoParams, length_turns_order, xie2024_order

TUNING_SEEDS = tuple(range(100, 110))   # random_lawn seeds, disjoint from maps.RANDOM_SEEDS
TUNING_SIZE = (24.0, 16.0)              # m
TUNING_OBSTACLES = (1, 2)               # keeps 6-7 cells, small enough for brute force
BUDGETS = (1000, 2000)
RUN_SEEDS = (0, 1, 2)
GRID_SIZE = 5
TOLERANCE = 1e-9                        # relative; a run at most this far above the optimum hits it
GAP_DIGITS = 9                          # mean gaps equal to this many digits tie

# Settings tried per method. The first entry was the default before tuning.
GRIDS = {
    'energy_ga': [dict(population=50, mutation_rate=0.5), dict(population=100, mutation_rate=0.5),
                  dict(population=50, mutation_rate=0.3), dict(population=100, mutation_rate=0.3),
                  dict(population=30, mutation_rate=0.5)],
    'energy_pso': [dict(particles=60, max_velocity=0.5), dict(particles=40, max_velocity=0.5),
                   dict(particles=20, max_velocity=0.5), dict(particles=60, max_velocity=0.2),
                   dict(particles=60, max_velocity=1.0)],
    'xie2024': [dict(population=400, ants=100, ga_share=0.5),
                dict(population=100, ants=50, ga_share=0.5),
                dict(population=50, ants=25, ga_share=0.5),
                dict(population=100, ants=50, ga_share=0.3),
                dict(population=100, ants=50, ga_share=0.7)],
}
GRIDS['xie_length_turns'] = GRIDS['xie2024']

DEFAULTS = {'energy_ga': EnergyGaParams(), 'energy_pso': EnergyPsoParams(),
            'xie2024': XieGaAcoParams(), 'xie_length_turns': XieGaAcoParams()}


def tuning_lawns():
    return [random_lawn(s, size=TUNING_SIZE, obstacles=TUNING_OBSTACLES) for s in TUNING_SEEDS]


def tuning_cases(path_cost, width=DEFAULT_WIDTH):
    """(name, problem, {method: optimum of its objective}) for each tuning lawn."""
    out = []
    for lawn in tuning_lawns():
        problem = build_problem(lawn.area, width, path_cost, start=lawn.start)
        centre = np.array([c.centroid.coords[0] for c in problem.cells])
        dist = np.linalg.norm(centre[:, None] - centre[None, :], axis=-1)
        turn_weight = XieGaAcoParams().turn_weight
        energy = brute_force_order(problem.table, {'energy': 1.0}).cost
        optima = {'energy_ga': energy, 'energy_pso': energy,
                  'xie2024': brute_force_tour(dist)[0],
                  'xie_length_turns': brute_force_order(
                      problem.table, {'length': 1.0, 'turns': turn_weight}).cost}
        out.append((lawn.name, problem, optima))
    return out


def run_method(method, problem, params, seed):
    if method == 'energy_ga':
        return energy_ga_order(problem.table, params, seed)
    if method == 'energy_pso':
        return energy_pso_order(problem.table, params, seed)
    if method == 'xie2024':
        return xie2024_order(problem, params, seed)
    return length_turns_order(problem.table, params, seed)


def tune(cases, methods=tuple(GRIDS), log=print):
    """Score every setting of every method. Returns rows and the best setting per method."""
    rows, best = [], {}
    for method in methods:
        scores = []
        for k, setting in enumerate(GRIDS[method]):
            hits, gaps = 0, []
            for name, problem, optima in cases:
                for budget in BUDGETS:
                    params = replace(DEFAULTS[method], evaluations=budget, **setting)
                    for seed in RUN_SEEDS:
                        cost = run_method(method, problem, params, seed).plan.cost
                        gap = cost / optima[method] - 1
                        hits += gap <= TOLERANCE
                        gaps.append(gap)
                        rows.append(dict(method=method, setting=k, **setting, lawn=name,
                                         budget=budget, seed=seed, cost=cost, gap=gap))
            runs = len(gaps)
            scores.append((-hits, round(float(np.mean(gaps)), GAP_DIGITS), k))
            log(f'{method} {setting}: {hits}/{runs} optimal, mean gap {np.mean(gaps):.5f}')
        best[method] = GRIDS[method][min(scores)[2]]
        log(f'{method}: best {best[method]}')
    return rows, best


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--robot', default='pioneer3at', help='robot config name or path')
    parser.add_argument('--methods', nargs='*', default=list(GRIDS), choices=list(GRIDS))
    parser.add_argument('--out', default='tuning.csv', help='CSV file with every tuning run')
    args = parser.parse_args(argv)

    from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
    cfg = load_robot_config(resolve_robot_config(args.robot))
    rows, best = tune(tuning_cases(PathCost.from_config(cfg)), args.methods,
                      log=lambda s: print(s, file=sys.stderr))
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in ('method', 'setting'), k))
    with open(args.out, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, keys, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    for method, setting in best.items():
        print(f'{method}\t{setting}')


if __name__ == '__main__':
    main()
