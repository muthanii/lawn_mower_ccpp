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
"""

import argparse
import csv
import math
import os
import statistics
import sys
from dataclasses import dataclass

import numpy as np

OURS = 'energy_ga'
COMPARISONS = (('energy_ga', 'xie2024'),           # our method against the baseline
               ('energy_ga', 'xie_length_turns'),  # the energy objective alone (same search inputs)
               ('energy_pso', 'xie2024'))          # the PSO form of our method
METRICS = ('energy', 'length', 'turns', 'connection_length', 'coverage', 'seconds')
LABELS = {'energy': 'Energy (kJ)', 'length': 'Path length (m)', 'turns': 'Turns',
          'connection_length': 'Connection length (m)', 'coverage': r'Coverage (\%)',
          'seconds': 'Planning time (s)'}
SCALE = {'energy': 1e-3}  # J -> kJ in the tables
METHOD_LABELS = {'energy_ga': 'Ours (GA)', 'energy_pso': 'Ours (PSO)',
                 'xie2024': 'Xie et al.', 'xie_length_turns': 'Xie search, L+T',
                 'energy_optimal': 'Optimum'}
ALPHA = 0.05


@dataclass(frozen=True)
class Test:
    n: int            # pairs used (non-zero differences)
    mean_diff: float  # mean relative difference over all pairs [%]
    p: float          # two-sided Wilcoxon p, nan without a test
    r: float          # rank-biserial effect size, nan without a test


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
    """Signed number with a real minus sign, set in the table's text font."""
    return f'{x:+.2f}'.replace('-', r'\textminus{}')


def _escape(name):
    return name.replace('_', r'\_')


def energy_table(rows, robot):
    """LaTeX table: energy per lawn and method, and our GA against Xie et al."""
    methods = [m for m in ('energy_ga', 'energy_pso', 'xie2024', 'xie_length_turns',
                           'energy_optimal') if any(r['method'] == m for r in rows)]
    tests = per_lawn(rows, OURS, 'xie2024', 'energy')
    seeds = max(len(values(rows, lawn, OURS, 'energy')) for lawn in lawns(rows))
    head = ' & '.join(METHOD_LABELS[m] for m in methods)
    lines = [
        _header(),
        r'\begin{table}[t]',
        r'\centering',
        r'\caption{Energy of the coverage plan (kJ, mean $\pm$ standard deviation over '
        f'{seeds} seeds) on the shared test lawns, robot model \\texttt{{{_escape(robot)}}}. '
        r'$\Delta$: relative difference of our GA to Xie et al.; $p$: two-sided Wilcoxon '
        r'signed-rank test over the seeds, Holm-corrected over the lawns. '
        r'The optimum is exact (brute force) and only computed for lawns of up to 8 cells.}',
        r'\label{tab:ordering-energy}',
        r'\small',
        r'\setlength{\tabcolsep}{4pt}',
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
                cols.append(f'{_fmt(ms[0], "energy")} \\textpm{{}} {_fmt(ms[1], "energy")}')
        t = tests[lawn]
        lines.append(f'\\texttt{{{_escape(lawn)}}} & {cells} & ' + ' & '.join(cols)
                     + f' & {_signed(t.mean_diff)} & {fmt_p(t.p)} \\\\')
    t = across_lawns(rows, OURS, 'xie2024', 'energy')
    lines += [
        r'\midrule',
        f'All lawns & & \\multicolumn{{{len(methods)}}}{{l}}{{test over the {t.n} lawn means, '
        f'rank-biserial $r$ = {_signed(t.r)}}} & {_signed(t.mean_diff)} & {fmt_p(t.p)} \\\\',
        r'\bottomrule',
        r'\end{tabular}',
        r'\end{table}',
    ]
    return '\n'.join(lines) + '\n'


def metrics_table(rows, robot):
    """LaTeX table: every metric, our GA against Xie et al. and against the L+T ablation."""
    lines = [
        _header(),
        r'\begin{table}[t]',
        r'\centering',
        r'\caption{Our GA against Xie et al.\ and against the search of Xie et al.\ with a '
        r'length-and-turns objective on the same transition table (L+T), over all lawns and '
        r'seeds, robot model \texttt{' + _escape(robot) + r'}. Means over all runs; $\Delta$: '
        r'mean over the lawns of the relative difference of our GA, paired by seed; $p$: '
        r'two-sided Wilcoxon signed-rank test over the lawn means; $r$: rank-biserial effect '
        r'size.}',
        r'\label{tab:ordering-metrics}',
        r'\small',
        r'\setlength{\tabcolsep}{4pt}',
        r'\begin{tabular}{l r r r r r r r r}',
        r'\toprule',
        r' & & \multicolumn{4}{c}{vs.\ Xie et al.} & \multicolumn{3}{c}{vs.\ L+T} \\',
        r'\cmidrule(lr){3-6} \cmidrule(lr){7-9}',
        r'Metric & Ours & Xie & $\Delta$ (\%) & $p$ & $r$ & $\Delta$ (\%) & $p$ & $r$ \\',
        r'\midrule',
    ]
    for metric in METRICS:
        ours = [float(r[metric]) for r in rows if r['method'] == OURS and r.get(metric)]
        xie = [float(r[metric]) for r in rows if r['method'] == 'xie2024' and r.get(metric)]
        if not ours or not xie:
            continue
        a = across_lawns(rows, OURS, 'xie2024', metric)
        b = across_lawns(rows, OURS, 'xie_length_turns', metric)
        lines.append(f'{LABELS[metric]} & {_fmt(statistics.mean(ours), metric)} & '
                     f'{_fmt(statistics.mean(xie), metric)} & {_signed(a.mean_diff)} & '
                     f'{fmt_p(a.p)} & {_r_tex(a.r)} & {_signed(b.mean_diff)} & {fmt_p(b.p)} & '
                     f'{_r_tex(b.r)} \\\\')
    lines += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
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
        if not any(r['method'] == b for r in rows) or not any(r['method'] == a for r in rows):
            continue
        lines.append(f'{a} vs {b}')
        for metric in METRICS:
            t = across_lawns(rows, a, b, metric)
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
