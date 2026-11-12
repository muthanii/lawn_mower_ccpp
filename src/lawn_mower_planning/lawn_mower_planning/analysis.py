"""Significance tests and paper tables from an experiment CSV (run_experiments).

    ros2 run lawn_mower_planning analyze_results results.csv --tables paper/tables

Runs are paired by lawn and seed. For each pair the relative difference
d = 100 (a / b - 1) [%] of a metric is formed, method a against method b,
and a two-sided Wilcoxon signed-rank test asks whether d is centred on 0:

- per lawn: the seeds of that lawn, Holm-corrected over the lawns;
- across lawns: the mean difference of each lawn, one value per lawn. Seeds
  on one lawn are not independent, so the lawn is the unit of this test;
  relative differences make large and small lawns weigh the same.

The effect size is the matched-pairs rank-biserial correlation
r = (W+ - W-) / (W+ + W-): -1 when a is lower in every pair, +1 when higher.
Pairs with d = 0 are left out of the test and of r (Wilcoxon's method); a
lawn where all pairs tie has no test.

A deterministic method (energy_optimal) has one run per lawn; it is only
reported as a reference value, not tested.

When the CSV has energy_direction_ga (our full method: the decomposition
direction chosen by energy, then the GA order), it is the method that the
tables compare with Xie et al., and xie2024_direction (their ordering on our
chosen decomposition) splits the gain into the direction and the ordering.
Without it the tables compare energy_ga, our ordering on the shared default
decomposition.
"""

import argparse
import csv
import math
import os
import statistics
import sys
from dataclasses import dataclass

import numpy as np

ORDER_ONLY = 'energy_ga'          # our ordering on the shared default decomposition
FULL = 'energy_direction_ga'      # our full method: energy-chosen direction, then the GA
SAME_DIRECTION = 'xie2024_direction'  # Xie et al.'s ordering on our chosen decomposition
COMPARISONS = ((FULL, 'xie2024'),                  # our full method against the baseline
               (FULL, SAME_DIRECTION),             # the ordering alone, on our decomposition
               (SAME_DIRECTION, 'xie2024'),        # the direction alone, with their ordering
               ('energy_ga', 'xie2024'),           # the ordering alone, default decomposition
               ('energy_ga', 'xie_length_turns'),  # the energy objective alone (same search inputs)
               ('energy_pso', 'xie2024'))          # the PSO form of our ordering
METRICS = ('energy', 'energy_per_area', 'length', 'turns', 'connection_length', 'coverage',
           'overlap', 'seconds')
LABELS = {'energy': 'Energy (kJ)', 'energy_per_area': r'Energy per mowed area (J/m$^2$)',
          'length': 'Path length (m)', 'turns': 'Turns',
          'connection_length': 'Connection length (m)', 'coverage': r'Coverage (\%)',
          'overlap': r'Overlap (\%)', 'seconds': 'Planning time (s)'}
SCALE = {'energy': 1e-3}  # J -> kJ in the tables
METHOD_LABELS = {'energy_ga': 'Ours (GA)', 'energy_pso': 'Ours (PSO)',
                 'xie2024': 'Xie et al.', 'xie_length_turns': 'Xie search, L+T',
                 'energy_optimal': 'Optimum'}
# With our full method in the CSV, the columns say which decomposition each method uses.
DIRECTION_LABELS = {FULL: 'Ours', 'energy_ga': 'Ours, order only', 'energy_pso': 'Order only, PSO',
                    'xie2024': 'Xie et al.', SAME_DIRECTION: 'Xie et al., our dir.',
                    'xie_length_turns': 'Xie search, L+T', 'energy_optimal': 'Optimum'}
ALPHA = 0.05


@dataclass(frozen=True)
class Test:
    n: int            # pairs used (non-zero differences)
    mean_diff: float  # mean relative difference over all pairs [%]
    p: float          # two-sided Wilcoxon p, nan without a test
    r: float          # rank-biserial effect size, nan without a test


def has(rows, method):
    return any(r['method'] == method for r in rows)


def main_method(rows):
    """The method the tables compare with Xie et al.: our full method if it ran."""
    return FULL if has(rows, FULL) else ORDER_ONLY


def load(path):
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


def lawns(rows):
    """Lawn names in file order, with their cell counts."""
    out = {}
    for r in rows:
        out.setdefault(r['map'], int(r['cells']))
    return out


def values(rows, lawn, method, metric):
    """{seed: value} of one method on one lawn."""
    return {int(r['seed']): float(r[metric]) for r in rows
            if r['map'] == lawn and r['method'] == method and r.get(metric) not in (None, '')}


def relative_differences(rows, a, b, metric, lawn=None):
    """100 (a / b - 1) for every lawn and seed both methods ran."""
    out = []
    for name in ([lawn] if lawn else lawns(rows)):
        va, vb = values(rows, name, a, metric), values(rows, name, b, metric)
        out += [100 * (va[s] / vb[s] - 1) for s in sorted(set(va) & set(vb)) if vb[s] != 0]
    return np.array(out)


def wilcoxon(d):
    """Two-sided Wilcoxon signed-rank test of the differences d against 0."""
    from scipy.stats import rankdata
    from scipy.stats import wilcoxon as scipy_wilcoxon
    mean = float(np.mean(d)) if len(d) else math.nan
    nonzero = d[np.abs(d) > 1e-12]
    if len(nonzero) == 0:
        return Test(0, mean, math.nan, math.nan)
    ranks = rankdata(np.abs(nonzero))
    w_plus, w_minus = ranks[nonzero > 0].sum(), ranks[nonzero < 0].sum()
    r = (w_plus - w_minus) / (w_plus + w_minus)
    p = scipy_wilcoxon(nonzero).pvalue if len(nonzero) > 1 else 1.0
    return Test(len(nonzero), mean, float(p), float(r))


def holm(pvalues):
    """Holm-Bonferroni adjusted p-values; nan stays nan."""
    p = np.asarray(pvalues, dtype=float)
    out = np.full(len(p), math.nan)
    idx = [i for i in np.argsort(p) if not math.isnan(p[i])]
    m, running = len(idx), 0.0
    for k, i in enumerate(idx):
        running = max(running, min(1.0, (m - k) * p[i]))
        out[i] = running
    return out


def per_lawn(rows, a, b, metric):
    """{lawn: Test with Holm-adjusted p}."""
    names = list(lawns(rows))
    tests = [wilcoxon(relative_differences(rows, a, b, metric, name)) for name in names]
    adjusted = holm([t.p for t in tests])
    return {name: Test(t.n, t.mean_diff, adj, t.r)
            for name, t, adj in zip(names, tests, adjusted)}


def across_lawns(rows, a, b, metric):
    """Test over the lawns: one mean relative difference per lawn."""
    means = [d.mean() for d in (relative_differences(rows, a, b, metric, name)
                                for name in lawns(rows)) if len(d)]
    return wilcoxon(np.array(means))


def mean_sd(rows, lawn, method, metric):
    v = list(values(rows, lawn, method, metric).values())
    if not v:
        return None
    return statistics.mean(v), (statistics.stdev(v) if len(v) > 1 else 0.0)


def fmt_p(p):
    if math.isnan(p):
        return '--'
    if p < 0.001:
        return '$<$0.001'
    return f'{p:.3f}'


DIGITS = {'energy': 2, 'turns': 0}  # decimals in the tables; 1 for the rest


def _fmt(value, metric):
    return f'{value * SCALE.get(metric, 1):.{DIGITS.get(metric, 1)}f}'


def _signed(x):
    """Signed number in math mode, so the minus sign typesets."""
    return f'${x:+.2f}$'


def _escape(name):
    return name.replace('_', r'\_')


def energy_table(rows, robot):
    """LaTeX table: energy per lawn and method, and our method against Xie et al."""
    ours = main_method(rows)
    full = ours == FULL
    labels = DIRECTION_LABELS if full else METHOD_LABELS
    methods = [m for m in (FULL, 'energy_ga', 'energy_pso', 'xie2024', SAME_DIRECTION,
                           'xie_length_turns', 'energy_optimal') if has(rows, m)]
    tests = per_lawn(rows, ours, 'xie2024', 'energy')
    seeds = max(len(values(rows, lawn, ours, 'energy')) for lawn in lawns(rows))
    head = ' & '.join(labels[m] for m in methods)
    if full:
        setup = (r'Ours: the decomposition direction chosen by energy, then the GA order. '
                 r'Our dir.: the ordering of Xie et al.\ on our chosen decomposition. '
                 r'All other columns use the default decomposition (lanes along $y$), and '
                 r'Cells is its cell count. $\Delta$: relative difference of Ours to Xie et al.; ')
        optimum = (r'The optimum is the exact best order on the default decomposition (brute '
                   r'force), only computed for lawns of up to 8 cells.}')
    else:
        setup = r'$\Delta$: relative difference of our GA to Xie et al.; '
        optimum = (r'The optimum is exact (brute force) and only computed for lawns of up to '
                   r'8 cells.}')
    lines = [
        _header(),
        r'\begin{table*}[t]',
        r'\centering',
        r'\caption{Energy of the coverage plan (kJ, mean $\pm$ standard deviation over '
        f'{seeds} seeds) on the shared test lawns, robot model \\texttt{{{_escape(robot)}}}. '
        + setup + r'$p$: two-sided Wilcoxon '
        r'signed-rank test over the seeds, Holm-corrected over the lawns. '
        + optimum,
        r'\label{tab:ordering-energy}',
        r'\small',
        r'\setlength{\tabcolsep}{4pt}',
        r'\resizebox{\textwidth}{!}{%',
        r'\begin{tabular}{l r ' + 'r ' * len(methods) + r'r r}',
        r'\toprule',
        f'Lawn & Cells & {head} & $\\Delta$ (\\%) & $p$ \\\\',
        r'\midrule',
    ]
    for lawn, cells in lawns(rows).items():
        cols = []
        for m in methods:
            ms = mean_sd(rows, lawn, m, 'energy')
            if ms is None:
                cols.append('--')
            elif m == 'energy_optimal':
                cols.append(_fmt(ms[0], 'energy'))
            else:
                cols.append(f'{_fmt(ms[0], "energy")} $\\pm$ {_fmt(ms[1], "energy")}')
        t = tests[lawn]
        lines.append(f'\\texttt{{{_escape(lawn)}}} & {cells} & ' + ' & '.join(cols)
                     + f' & {_signed(t.mean_diff)} & {fmt_p(t.p)} \\\\')
    t = across_lawns(rows, ours, 'xie2024', 'energy')
    lines += [
        r'\midrule',
        f'All lawns & & \\multicolumn{{{len(methods)}}}{{l}}{{test over the {t.n} lawn means, '
        f'rank-biserial $r = {t.r:+.2f}$}} & {_signed(t.mean_diff)} & {fmt_p(t.p)} \\\\',
        r'\bottomrule',
        r'\end{tabular}}',
        r'\end{table*}',
    ]
    return '\n'.join(lines) + '\n'


def metrics_table(rows, robot):
    """LaTeX table: every metric, our method against Xie et al. and against one ablation.

    The ablation is Xie et al.'s ordering on our chosen decomposition when our
    full method ran, and otherwise their search with a length-and-turns
    objective (against our GA, on the default decomposition).
    """
    ours = main_method(rows)
    if ours == FULL:
        other = SAME_DIRECTION
        caption = (r'\caption{Our method (the decomposition direction chosen by energy, then '
                   r'the GA order) against Xie et al.\ on the default decomposition, and against '
                   r'the ordering of Xie et al.\ on our chosen decomposition (same dir.), which '
                   r'leaves the ordering as the only difference, over all lawns and '
                   r'seeds, robot model \texttt{' + _escape(robot) + r'}. ')
        block = r'vs.\ Xie et al., same dir.'
    else:
        other = 'xie_length_turns'
        caption = (r'\caption{Our GA against Xie et al.\ and against the search of Xie et al.\ '
                   r'with a length-and-turns objective on the same transition table (L+T), over '
                   r'all lawns and seeds, robot model \texttt{' + _escape(robot) + r'}. ')
        block = r'vs.\ L+T'
    lines = [
        _header(),
        r'\begin{table}[t]',
        r'\centering',
        caption + r'Means over all runs; $\Delta$: '
        r'mean over the lawns of the relative difference of ' +
        ('our method' if ours == FULL else 'our GA') + r', paired by seed; $p$: '
        r'two-sided Wilcoxon signed-rank test over the lawn means; $r$: rank-biserial effect '
        r'size.}',
        r'\label{tab:ordering-metrics}',
        r'\small',
        r'\setlength{\tabcolsep}{4pt}',
        r'\resizebox{\linewidth}{!}{%',
        r'\begin{tabular}{l r r r r r r r r}',
        r'\toprule',
        r' & & \multicolumn{4}{c}{vs.\ Xie et al.} & \multicolumn{3}{c}{' + block + r'} \\',
        r'\cmidrule(lr){3-6} \cmidrule(lr){7-9}',
        r'Metric & Ours & Xie & $\Delta$ (\%) & $p$ & $r$ & $\Delta$ (\%) & $p$ & $r$ \\',
        r'\midrule',
    ]
    for metric in METRICS:
        mine = [float(r[metric]) for r in rows if r['method'] == ours and r.get(metric)]
        xie = [float(r[metric]) for r in rows if r['method'] == 'xie2024' and r.get(metric)]
        if not mine or not xie:
            continue
        a = across_lawns(rows, ours, 'xie2024', metric)
        b = across_lawns(rows, ours, other, metric)
        lines.append(f'{LABELS[metric]} & {_fmt(statistics.mean(mine), metric)} & '
                     f'{_fmt(statistics.mean(xie), metric)} & {_signed(a.mean_diff)} & '
                     f'{fmt_p(a.p)} & {_r_tex(a.r)} & {_signed(b.mean_diff)} & {fmt_p(b.p)} & '
                     f'{_r_tex(b.r)} \\\\')
    lines += [r'\bottomrule', r'\end{tabular}}', r'\end{table}']
    return '\n'.join(lines) + '\n'


def _r(r):
    return '--' if math.isnan(r) else f'{r:+.2f}'


def _r_tex(r):
    return '--' if math.isnan(r) else _signed(r)


def _header():
    return ('% Generated by `ros2 run lawn_mower_planning analyze_results`; do not edit by hand.\n'
            '% Regenerate it from the experiment CSV after every run.')


def report(rows):
    """Plain-text report of every comparison and metric."""
    lines = []
    for a, b in COMPARISONS:
        if not has(rows, a) or not has(rows, b):
            continue
        lines.append(f'{a} vs {b}')
        for metric in METRICS:
            t = across_lawns(rows, a, b, metric)
            if math.isnan(t.mean_diff):  # a CSV from before this column
                continue
            lines.append(f'  {metric:18s} across lawns: mean diff {t.mean_diff:+7.2f} %  '
                         f'p {fmt_p(t.p):>9s}  r {_r(t.r):>5s}  n {t.n}')
        lines.append('  energy per lawn (Holm):')
        for lawn, t in per_lawn(rows, a, b, 'energy').items():
            mark = '*' if not math.isnan(t.p) and t.p < ALPHA else ' '
            lines.append(f'    {lawn:12s} {t.mean_diff:+7.2f} %  p {fmt_p(t.p):>9s}{mark} '
                         f'r {_r(t.r):>5s}  n {t.n}')
    return '\n'.join(lines).replace('$<$', '<')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('results', help='CSV from run_experiments')
    parser.add_argument('--tables', help='folder for the LaTeX tables (e.g. paper/tables)')
    args = parser.parse_args(argv)
    rows = load(args.results)
    if not rows:
        parser.error(f'{args.results} has no runs')
    robots = sorted({r['robot'] for r in rows})
    if len(robots) > 1:
        parser.error(f'{args.results} mixes robots {robots}; analyse one robot at a time')
    # CSVs from before the sweep_angles column have fixed vertical lanes ('0').
    angles = sorted({r.get('sweep_angles') or '0' for r in rows})
    if len(angles) > 1:
        parser.error(f'{args.results} mixes sweep angle sets {angles}; analyse one at a time')
    print(report(rows))
    if args.tables:
        os.makedirs(args.tables, exist_ok=True)
        for name, text in (('ordering-energy.tex', energy_table(rows, robots[0])),
                           ('ordering-metrics.tex', metrics_table(rows, robots[0]))):
            path = os.path.join(args.tables, name)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
            print(f'wrote {path}', file=sys.stderr)


if __name__ == '__main__':
    main()
