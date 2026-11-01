import csv

import pytest

from lawn_mower_planning.experiment import COLUMNS, connection_length, main, run, summarize
from lawn_mower_planning.maps import lawn_by_name
from lawn_mower_planning.path_cost import PathCost
from lawn_mower_planning.problem import build_problem
from lawn_mower_planning.xie_ga_aco import XieGaAcoParams

PARAMS = XieGaAcoParams(evaluations=300, population=30, ants=15)


def test_run_writes_one_row_per_run(tmp_path):
    out = tmp_path / 'results.csv'
    rows = run([lawn_by_name('two_beds')], ['xie2024', 'xie_length_turns', 'energy_optimal'],
               [0, 1], PARAMS, PathCost(), 'none', 0.5, str(out), log=lambda s: None)
    # Two seeds for each search; the energy reference skips a table without energy.
    assert [r['method'] for r in rows] == ['xie2024'] * 2 + ['xie_length_turns'] * 2
    with open(out, encoding='utf-8') as f:
        saved = list(csv.DictReader(f))
    assert len(saved) == 4 and list(saved[0]) == COLUMNS
    for r in rows:
        assert 0 <= r['connection_length'] <= r['length']
        assert 80 < r['coverage'] <= 100 and r['cells'] == 7
    # Appending keeps a single header.
    run([lawn_by_name('two_beds')], ['xie2024'], [2], PARAMS, PathCost(), 'none', 0.5, str(out),
        log=lambda s: None)
    with open(out, encoding='utf-8') as f:
        assert sum(1 for line in f if line.startswith('map,')) == 1


def test_energy_optimal_is_the_least_energy(tmp_path):
    from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
    cost = PathCost.from_config(load_robot_config(resolve_robot_config('pioneer3at')))
    rows = run([lawn_by_name('two_beds')], ['xie2024', 'energy_optimal'], [0, 1], PARAMS, cost,
               'pioneer3at', 0.5, log=lambda s: None)
    # energy_optimal is deterministic, so it runs once, not once per seed.
    assert [r['method'] for r in rows] == ['xie2024', 'xie2024', 'energy_optimal']
    best = rows[-1]
    assert all(best['energy'] <= r['energy'] for r in rows)
    assert best['energy_wh'] == pytest.approx(best['energy'] / 3600)


def test_connection_length_is_outside_the_cells():
    lawn = lawn_by_name('two_beds')
    problem = build_problem(lawn.area, 0.5, start=lawn.start)
    from lawn_mower_planning.brute_force import brute_force_order
    plan = brute_force_order(problem.table, {'length': 1})
    inside = sum(problem.table.cell['length'][c, a] for c, a in zip(plan.order, plan.variants))
    assert connection_length(problem, plan) == pytest.approx(plan.metrics['length'] - inside)


def test_summary_has_one_line_per_map_and_method():
    rows = [{'map': 'a', 'method': 'm', 'energy': 1.0}, {'map': 'a', 'method': 'm', 'energy': 3.0}]
    text = summarize(rows, metrics=('energy',))
    assert text.splitlines()[1] == 'a\tm\t2\t2.0 ± 1.4'


def test_command_line(tmp_path, capsys):
    out = tmp_path / 'cli.csv'
    main(['--maps', 'two_beds', '--methods', 'xie2024', '--seeds', '2', '--budget', '200',
          '--population', '20', '--ants', '10', '--out', str(out)])
    assert 'two_beds\txie2024\t2' in capsys.readouterr().out
    with open(out, encoding='utf-8') as f:
        assert len(list(csv.DictReader(f))) == 2
    with pytest.raises(SystemExit):
        main(['--maps', 'moon'])
