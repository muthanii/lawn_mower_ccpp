"""Run the cell-ordering methods on the shared lawns and log one CSV row per run.

    ros2 run lawn_mower_planning run_experiments --seeds 10 --out results.csv

Each lawn is built once (cells, lanes, A* table with length, turns and
energy from the robot config) and every method plans on that same problem
with the same seeds and the same evaluation budget. Coverage, overlap and
connection length come from the assembled path, so all methods are measured
the same way. Rows are written as they finish, so a stopped run keeps its
results.

A method is a function (problem, settings, seed) -> RunResult, or a string
that says why it skips this lawn. Every method runs with settings.budget
evaluations; Settings holds each method's other parameters. Register new
methods in METHODS.

The methods in CHOSEN_DIRECTION plan on the decomposition whose direction
direction.choose_direction picks by energy, the others on the default one
(lanes along y). energy_direction_ga pays for the choice from its budget.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, replace

from .brute_force import brute_force_order
from .direction import CANDIDATE_LIMIT, SCREEN_EVALUATIONS, choose_direction
from .energy_order import EnergyGaParams, EnergyPsoParams, energy_ga_order, energy_pso_order
from .maps import DEFAULT_WIDTH, shared_lawns
from .path_cost import PathCost
from .problem import build_problem
from .search import RunResult
from .xie_ga_aco import XieGaAcoParams, length_turns_order, xie2024_order

BRUTE_FORCE_CELLS = 8  # energy_optimal runs only up to 8! = 40320 orders
# Lane angles each cell may use [deg from the BCD sweep direction]. Every
# method gets the same set: the table-based methods search over it, and
# xie2024 takes each cell's fewest-turns angle. --sweep-angles 0 gives the
# fixed vertical lanes of v1.9.0.
SWEEP_ANGLES_DEG = (0.0, 45.0, 90.0, 135.0)


# Tuned with tuning.py (see the package README): the paper's settings won for the ablation
# and tied for xie2024.
TUNED_LENGTH_TURNS = XieGaAcoParams()


@dataclass(frozen=True)
class Settings:
    """One evaluation budget for every method, and each method's own parameters."""

    budget: int = XieGaAcoParams.evaluations
    xie: XieGaAcoParams = XieGaAcoParams()
    ga: EnergyGaParams = EnergyGaParams()
    pso: EnergyPsoParams = EnergyPsoParams()
    xie_length_turns: XieGaAcoParams = TUNED_LENGTH_TURNS
    directions: int = CANDIDATE_LIMIT             # candidate decomposition directions
    screen_evaluations: int = SCREEN_EVALUATIONS  # evaluated orders to score one of them

    def budgeted(self, params, spent=0):
        """params with the shared budget, less the evaluations already spent."""
        return replace(params, evaluations=self.budget - spent)


def _energy_optimal(problem, settings, seed):
    """Exact energy-optimal plan (reference, deterministic, small lawns only)."""
    if 'energy' not in problem.table.metrics:
        return 'no energy model'
    if problem.table.n_cells > BRUTE_FORCE_CELLS:
        return f'more than {BRUTE_FORCE_CELLS} cells'
    t0 = time.perf_counter()
    plan = brute_force_order(problem.table, {'energy': 1.0})
    return RunResult(plan, time.perf_counter() - t0, 0)


def _energy_ga(problem, settings, seed, spent=0):
    if 'energy' not in problem.table.metrics:
        return 'no energy model'
    return energy_ga_order(problem.table, settings.budgeted(settings.ga, spent), seed)


def _energy_pso(problem, settings, seed):
    if 'energy' not in problem.table.metrics:
        return 'no energy model'
    return energy_pso_order(problem.table, settings.budgeted(settings.pso), seed)


def _xie2024(problem, settings, seed):
    return xie2024_order(problem, settings.budgeted(settings.xie), seed)


def _xie_length_turns(problem, settings, seed):
    return length_turns_order(problem.table, settings.budgeted(settings.xie_length_turns), seed)


METHODS = {
    'energy_ga': _energy_ga,              # our ordering alone, GA, default direction
    'energy_pso': _energy_pso,            # our method, PSO
    'xie2024': _xie2024,                  # baseline
    'xie_length_turns': _xie_length_turns,  # ablation: Xie's search, length + turns objective
    'energy_optimal': _energy_optimal,    # exact reference, small lawns
    'energy_direction_ga': _energy_ga,    # our full method: energy-chosen direction, then the GA
    'xie2024_direction': _xie2024,        # ablation: the baseline on our chosen direction
}
DETERMINISTIC = {'energy_optimal'}  # run once per lawn, not once per seed
# Methods that plan on the energy-chosen decomposition direction. The value
# says whether the method pays for the choice from its own budget: ours does,
# and the ablation gets the direction for free on top of its full budget.
CHOSEN_DIRECTION = {'energy_direction_ga': True, 'xie2024_direction': False}

COLUMNS = ['map', 'cells', 'method', 'seed', 'budget', 'robot', 'width', 'sweep_angles',
           'lane_angle', 'cost', 'length',
           'turns', 'energy', 'energy_wh', 'energy_per_area', 'connection_length', 'coverage',
           'overlap', 'seconds', 'evaluations', 'ga_evaluations', 'direction_evaluations',
           'order', 'variants']


def connection_length(problem, plan):
    """Path length outside the cells: the start link and the A* transitions [m]."""
    inside = sum(problem.table.cell['length'][c, a] for c, a in zip(plan.order, plan.variants))
    return plan.metrics['length'] - inside


def run_row(lawn, problem, method, seed, settings, robot, width, choice=None):
    """One run as a CSV row, or the reason the method skips this lawn.

    choice is the DirectionChoice of the lawn, for the methods in
    CHOSEN_DIRECTION. Its time counts in their seconds, and for a method that
    pays for the choice its evaluations count too.
    """
    spent = 0
    if method in CHOSEN_DIRECTION:
        if choice is None:
            return 'no energy model'
        problem = choice.problem
        if CHOSEN_DIRECTION[method]:
            spent = choice.evaluations
            if spent >= settings.budget:
                return f'the direction choice uses the whole budget ({spent} evaluations)'
            result = METHODS[method](problem, settings, seed, spent)
        else:
            result = METHODS[method](problem, settings, seed)
    else:
        result = METHODS[method](problem, settings, seed)
    if isinstance(result, str):
        return result
    plan = result.plan
    row = result.row()
    row.update(problem.coverage(plan))
    row.update(map=lawn.name, cells=problem.table.n_cells, method=method, seed=seed,
               budget=settings.budget, robot=robot, width=width,
               sweep_angles=' '.join(f'{math.degrees(a):g}' for a in problem.sweep_angles),
               lane_angle=round(math.degrees(problem.lane_angle), 3),
               connection_length=connection_length(problem, plan),
               evaluations=result.evaluations + spent, direction_evaluations=spent)
    if method in CHOSEN_DIRECTION:
        row['seconds'] += choice.seconds
    if 'energy' in plan.metrics:
        from lawn_mower_robot.energy import JOULES_PER_WH
        row['energy_wh'] = plan.metrics['energy'] / JOULES_PER_WH
        # Energy per mowed square metre, so that plans with different coverage compare.
        row['energy_per_area'] = plan.metrics['energy'] / (row['coverage'] / 100 * lawn.area.area)
    return row


def run(lawns, methods, seeds, settings, path_cost, robot, width, out=None, log=print,
        sweep_angles_deg=(0.0,)):
    """Run every method on every lawn with every seed. Returns the rows."""
    angles = tuple(math.radians(a) for a in sweep_angles_deg)
    rows = []
    writer = None
    if out is not None:
        new = not os.path.exists(out) or os.path.getsize(out) == 0
        f = open(out, 'a', newline='', encoding='utf-8')
        writer = csv.DictWriter(f, COLUMNS, extrasaction='ignore', lineterminator='\n')
        if new:
            writer.writeheader()
    try:
        for lawn in lawns:
            problem = build_problem(lawn.area, width, path_cost, start=lawn.start,
                                    sweep_angles=angles)
            log(f'{lawn.name}: {problem.table.n_cells} cells, {problem.table.n_variants} variants')
            choice = None
            if 'energy' in path_cost.metrics and any(m in CHOSEN_DIRECTION for m in methods):
                screen = replace(settings.ga, evaluations=settings.screen_evaluations)
                choice = choose_direction(lawn.area, width, path_cost, lawn.start, angles,
                                          limit=settings.directions, screen=screen,
                                          default=problem)
                log(f'  direction: lanes at {math.degrees(choice.lane_angle):g} deg, '
                    f'{choice.problem.table.n_cells} cells, {choice.evaluations} evaluations; '
                    + ', '.join(f'{math.degrees(a):g}: {e:.0f} J' for a, e in choice.scores))
            for method in methods:
                for seed in (seeds[:1] if method in DETERMINISTIC else seeds):
                    row = run_row(lawn, problem, method, seed, settings, robot, width, choice)
                    if isinstance(row, str):
                        log(f'  {method}: skipped, {row}')
                        break
                    rows.append(row)
                    if writer is not None:
                        writer.writerow(row)
                        f.flush()
                    log(f"  {method} seed {seed}: energy {row.get('energy', float('nan')):.0f} J, "
                        f"length {row['length']:.1f} m, turns {row['turns']:.0f}, "
                        f"{row['seconds']:.1f} s")
    finally:
        if writer is not None:
            f.close()
    return rows


def summarize(rows, metrics=('energy', 'length', 'turns', 'connection_length', 'coverage')):
    """Mean (and standard deviation over seeds) per lawn and method, as text."""
    groups = {}
    for row in rows:
        groups.setdefault((row['map'], row['method']), []).append(row)
    lines = ['map\tmethod\truns\t' + '\t'.join(metrics)]
    for (lawn, method), group in groups.items():
        cells = []
        for m in metrics:
            values = [float(r[m]) for r in group if m in r]
            sd = statistics.stdev(values) if len(values) > 1 else 0.0
            cells.append(f'{statistics.mean(values):.1f} ± {sd:.1f}' if values else '-')
        lines.append(f'{lawn}\t{method}\t{len(group)}\t' + '\t'.join(cells))
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--robot', default='pioneer3at', help='robot config name or path')
    parser.add_argument('--maps', nargs='*', help='lawn names (default: all)')
    parser.add_argument('--methods', nargs='*', default=list(METHODS), choices=list(METHODS))
    parser.add_argument('--seeds', type=int, default=10, help='seeds 0 .. N-1')
    parser.add_argument('--budget', type=int, default=Settings.budget,
                        help='evaluated orders per run, the same for every method')
    parser.add_argument('--ga-population', type=int, default=EnergyGaParams.population,
                        help='energy_ga population')
    parser.add_argument('--pso-particles', type=int, default=EnergyPsoParams.particles,
                        help='energy_pso swarm size')
    parser.add_argument('--xie-population', type=int, default=XieGaAcoParams.population,
                        help='xie2024 GA population')
    parser.add_argument('--xie-ants', type=int, default=XieGaAcoParams.ants,
                        help='xie2024 ants per iteration')
    parser.add_argument('--xie-ga-share', type=float, default=XieGaAcoParams.ga_share,
                        help='xie2024 share of the budget for its GA stage')
    parser.add_argument('--width', type=float, default=DEFAULT_WIDTH, help='cutting width [m]')
    parser.add_argument('--sweep-angles', type=float, nargs='+', default=list(SWEEP_ANGLES_DEG),
                        help='lane angles each cell may use [deg from the BCD direction], '
                             'must include 0; 0 alone gives fixed vertical lanes')
    parser.add_argument('--directions', type=int, default=CANDIDATE_LIMIT,
                        help='candidate decomposition directions for the methods that choose '
                             'one by energy (the default direction included)')
    parser.add_argument('--screen-evaluations', type=int, default=SCREEN_EVALUATIONS,
                        help='evaluated orders to score one candidate direction')
    parser.add_argument('--out', default='results.csv', help='CSV file (rows are appended)')
    args = parser.parse_args(argv)

    from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
    cfg = load_robot_config(resolve_robot_config(args.robot))
    lawns = shared_lawns()
    if args.maps:
        unknown = set(args.maps) - {lawn.name for lawn in lawns}
        if unknown:
            parser.error(f'unknown maps {sorted(unknown)}')
        lawns = [lawn for lawn in lawns if lawn.name in args.maps]
    settings = Settings(args.budget,
                        XieGaAcoParams(ga_share=args.xie_ga_share,
                                       population=args.xie_population, ants=args.xie_ants),
                        EnergyGaParams(population=args.ga_population),
                        EnergyPsoParams(particles=args.pso_particles),
                        directions=args.directions, screen_evaluations=args.screen_evaluations)
    rows = run(lawns, args.methods, list(range(args.seeds)), settings, PathCost.from_config(cfg),
               cfg.name, args.width, args.out, log=lambda s: print(s, file=sys.stderr),
               sweep_angles_deg=tuple(args.sweep_angles))
    print(summarize(rows))
    print(f'{len(rows)} runs written to {args.out}', file=sys.stderr)


if __name__ == '__main__':
    main()
