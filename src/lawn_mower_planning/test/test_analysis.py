import csv
import math

import numpy as np
import pytest

from lawn_mower_planning import analysis

FIELDS = ['map', 'cells', 'method', 'seed', 'robot', 'energy', 'length', 'turns',
          'connection_length', 'coverage', 'seconds']


def fake_rows(robot='pioneer3at'):
    """Two lawns, 10 seeds: ours always 2 % below Xie et al., ablation 1 % below."""
    rows = []
    for lawn, cells, base in (('lawn_a', 7, 1000.0), ('lawn_b', 10, 2000.0)):
        for seed in range(10):
            xie = base * (1 + 0.001 * seed)
            for method, factor in (('energy_ga', 0.98), ('xie2024', 1.0),
                                   ('xie_length_turns', 0.99), ('energy_pso', 0.985)):
                v = xie * factor
                rows.append(dict(map=lawn, cells=str(cells), method=method, seed=str(seed),
                                 robot=robot, energy=str(v), length=str(v / 10),
                                 turns=str(100), connection_length=str(v / 100),
                                 coverage=str(95.0), seconds=str(1.0)))
        rows.append(dict(map=lawn, cells=str(cells), method='energy_optimal', seed='0',
                         robot=robot, energy=str(base * 0.97), length='1', turns='1',
                         connection_length='1', coverage='95', seconds='1'))
    return rows


def test_wilcoxon_signs_and_exact_p():
    t = analysis.wilcoxon(-np.arange(1, 11, dtype=float))
    assert t.n == 10 and t.r == -1.0
    assert t.p == pytest.approx(2 / 2 ** 10)  # exact two-sided minimum for 10 pairs
    tie = analysis.wilcoxon(np.zeros(5))
    assert tie.n == 0 and math.isnan(tie.p) and math.isnan(tie.r)


def test_holm():
    adjusted = analysis.holm([0.01, 0.04, math.nan, 0.03])
    assert adjusted[0] == pytest.approx(0.03)
    assert adjusted[3] == pytest.approx(0.06)
    assert adjusted[1] == pytest.approx(0.06)  # never below an earlier adjusted p
    assert math.isnan(adjusted[2])


def test_paired_relative_differences():
    rows = fake_rows()
    d = analysis.relative_differences(rows, 'energy_ga', 'xie2024', 'energy')
    assert len(d) == 20 and d == pytest.approx(-2.0)
    t = analysis.across_lawns(rows, 'energy_ga', 'xie2024', 'energy')
    assert t.n == 2 and t.mean_diff == pytest.approx(-2.0) and t.r == -1.0
    turns = analysis.across_lawns(rows, 'energy_ga', 'xie2024', 'turns')
    assert turns.n == 0 and math.isnan(turns.p)  # all pairs tie


def test_tables_and_report():
    rows = fake_rows()
    energy = analysis.energy_table(rows, 'pioneer3at')
    assert r'\texttt{lawn\_a}' in energy and r'\label{tab:ordering-energy}' in energy
    assert '0.97 & $-2.00$' in energy  # optimum (kJ) then our difference to Xie et al.
    assert energy.count(r'\\') == 2 + 1 + 1  # header, two lawns, the across-lawns row
    metrics = analysis.metrics_table(rows, 'pioneer3at')
    assert 'Energy (kJ) & 1.48 & 1.51 & $-2.00$' in metrics
    text = analysis.report(rows)
    assert 'energy_ga vs xie2024' in text and 'lawn_b' in text


def direction_rows():
    """fake_rows plus our full method (10 % below Xie et al.) and Xie et al. on our direction."""
    rows = fake_rows()
    for r in [r for r in rows if r['method'] == 'xie2024']:
        for method, factor in (('energy_direction_ga', 0.90), ('xie2024_direction', 0.93)):
            v = float(r['energy']) * factor
            rows.append(dict(r, method=method, energy=str(v), length=str(v / 10),
                             connection_length=str(v / 100)))
    return rows


def test_tables_compare_the_full_method_when_it_ran():
    rows = direction_rows()
    assert analysis.main_method(fake_rows()) == 'energy_ga'
    assert analysis.main_method(rows) == 'energy_direction_ga'
    energy = analysis.energy_table(rows, 'pioneer3at')
    assert 'Ours & Ours, order only & Order only, PSO & Xie et al. & Xie et al., our dir.' in energy
    assert '0.97 & $-10.00$' in energy  # the difference is now the full method's
    metrics = analysis.metrics_table(rows, 'pioneer3at')
    assert 'same dir.' in metrics and 'L+T' not in metrics
    # Against Xie et al.: -10 %. Against their ordering on our direction: 0.90 / 0.93 - 1.
    assert 'Energy (kJ) & 1.36 & 1.51 & $-10.00$' in metrics
    assert f'${100 * (0.90 / 0.93 - 1):+.2f}$' in metrics
    text = analysis.report(rows)
    for pair in ('energy_direction_ga vs xie2024', 'energy_direction_ga vs xie2024_direction',
                 'xie2024_direction vs xie2024', 'energy_ga vs xie2024'):
        assert pair in text


def test_command_line(tmp_path, capsys):
    path = tmp_path / 'results.csv'
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, FIELDS)
        writer.writeheader()
        writer.writerows(fake_rows())
    analysis.main([str(path), '--tables', str(tmp_path / 'tables')])
    assert (tmp_path / 'tables' / 'ordering-energy.tex').exists()
    assert (tmp_path / 'tables' / 'ordering-metrics.tex').exists()
    assert 'energy_ga vs xie2024' in capsys.readouterr().out
    with open(path, 'a', newline='', encoding='utf-8') as f:
        csv.DictWriter(f, FIELDS).writerows(fake_rows('faculty_robot')[:1])
    with pytest.raises(SystemExit):
        analysis.main([str(path)])


def test_command_line_refuses_mixed_sweep_angles(tmp_path):
    path = tmp_path / 'results.csv'
    fields = FIELDS + ['sweep_angles']
    rows = fake_rows()
    for i, row in enumerate(rows):
        row['sweep_angles'] = '0' if i else '0 45 90 135'
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fields)
        writer.writeheader()
        writer.writerows(rows)
    with pytest.raises(SystemExit):
        analysis.main([str(path)])
