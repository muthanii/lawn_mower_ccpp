"""Run the cell-ordering methods on the shared lawns and log one CSV row per run.

    ros2 run lawn_mower_planning run_experiments --seeds 10 --out results.csv

Each lawn is built once (cells, lanes, A* table with length, turns and
energy from the robot config) and every method plans on that same problem
with the same seeds and the same evaluation budget. Coverage, overlap and
connection length come from the assembled path, so all methods are measured
the same way. Rows are written as they finish, so a stopped run keeps its
results.

A method is a function (problem, params, seed) -> RunResult, or a string
that says why it skips this lawn. Register new ones (our energy GA/PSO) in
METHODS.
"""

import argparse
import csv
import os
import statistics
import sys
import time

from .brute_force import brute_force_order
from .maps import DEFAULT_WIDTH, shared_lawns
from .path_cost import PathCost
from .problem import build_problem
from .xie_ga_aco import RunResult, XieGaAcoParams, length_turns_order, xie2024_order

BRUTE_FORCE_CELLS = 8  # energy_optimal runs only up to 8! = 40320 orders


def _energy_optimal(problem, params, seed):
    """Exact energy-optimal plan (reference, deterministic, small lawns only)."""
    if 'energy' not in problem.table.metrics:
        return 'no energy model'
    if problem.table.n_cells > BRUTE_FORCE_CELLS:
        return f'more than {BRUTE_FORCE_CELLS} cells'
    t0 = time.perf_counter()
    plan = brute_force_order(problem.table, {'energy': 1.0})
    return RunResult(plan, time.perf_counter() - t0, 0, 0)


METHODS = {
    'xie2024': xie2024_order,
    'xie_length_turns': lambda problem, params, seed: length_turns_order(problem.table,
                                                                         params, seed),
    'energy_optimal': _energy_optimal,
}
DETERMINISTIC = {'energy_optimal'}  # run once per lawn, not once per seed

COLUMNS = ['map', 'cells', 'method', 'seed', 'budget', 'robot', 'width', 'cost', 'length',
           'turns', 'energy', 'energy_wh', 'connection_length', 'coverage', 'overlap',
           'seconds', 'evaluations', 'ga_evaluations', 'order', 'variants']


def connection_length(problem, plan):
    """Path length outside the cells: the start link and the A* transitions [m]."""
    inside = sum(problem.table.cell['length'][c, a] for c, a in zip(plan.order, plan.variants))
    return plan.metrics['length'] - inside


def run_row(lawn, problem, method, seed, params, robot, width):
    """One run as a CSV row, or the reason the method skips this lawn."""
    result = METHODS[method](problem, params, seed)
    if isinstance(result, str):
        return result
    plan = result.plan
    row = result.row()
    row.update(problem.coverage(plan))
    row.update(map=lawn.name, cells=problem.table.n_cells, method=method, seed=seed,
               budget=params.evaluations, robot=robot, width=width,
               connection_length=connection_length(problem, plan))
    if 'energy' in plan.metrics:
        from lawn_mower_robot.energy import JOULES_PER_WH
        row['energy_wh'] = plan.metrics['energy'] / JOULES_PER_WH
    return row


def run(lawns, methods, seeds, params, path_cost, robot, width, out=None, log=print):
    """Run every method on every lawn with every seed. Returns the rows."""
    rows = []
    writer = None
    if out is not None:
        new = not os.path.exists(out) or os.path.getsize(out) == 0
        f = open(out, 'a', newline='', encoding='utf-8')
        writer = csv.DictWriter(f, COLUMNS, extrasaction='ignore')
        if new:
            writer.writeheader()
    try:
        for lawn in lawns:
            problem = build_problem(lawn.area, width, path_cost, start=lawn.start)
            log(f'{lawn.name}: {problem.table.n_cells} cells')
            for method in methods:
                for seed in (seeds[:1] if method in DETERMINISTIC else seeds):
                    row = run_row(lawn, problem, method, seed, params, robot, width)
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
    parser.add_argument('--budget', type=int, default=XieGaAcoParams.evaluations,
                        help='evaluated orders per run')
    parser.add_argument('--ga-share', type=float, default=XieGaAcoParams.ga_share)
    parser.add_argument('--population', type=int, default=XieGaAcoParams.population)
    parser.add_argument('--ants', type=int, default=XieGaAcoParams.ants)
    parser.add_argument('--width', type=float, default=DEFAULT_WIDTH, help='cutting width [m]')
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
    params = XieGaAcoParams(evaluations=args.budget, ga_share=args.ga_share,
                            population=args.population, ants=args.ants)
    rows = run(lawns, args.methods, list(range(args.seeds)), params, PathCost.from_config(cfg),
               cfg.name, args.width, args.out, log=lambda s: print(s, file=sys.stderr))
    print(summarize(rows))
    print(f'{len(rows)} runs written to {args.out}', file=sys.stderr)


if __name__ == '__main__':
    main()
