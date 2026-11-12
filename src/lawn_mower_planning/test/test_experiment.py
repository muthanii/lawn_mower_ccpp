import csv

import pytest

from lawn_mower_planning.energy_order import EnergyGaParams, EnergyPsoParams
from lawn_mower_planning.experiment import COLUMNS, Settings, connection_length, main, run, \
    summarize
from lawn_mower_planning.maps import lawn_by_name
from lawn_mower_planning.path_cost import PathCost
from lawn_mower_planning.problem import build_problem
from lawn_mower_planning.xie_ga_aco import XieGaAcoParams

PARAMS = Settings(300, XieGaAcoParams(population=30, ants=15), EnergyGaParams(population=20),
                  EnergyPsoParams(particles=20))


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
    methods = ['energy_ga', 'energy_pso', 'xie2024', 'energy_optimal']
    rows = run([lawn_by_name('two_beds')], methods, [0, 1], PARAMS, cost, 'pioneer3at', 0.5,
               log=lambda s: None)
    # energy_optimal is deterministic, so it runs once, not once per seed.
    assert [r['method'] for r in rows] == ['energy_ga'] * 2 + ['energy_pso'] * 2 \
        + ['xie2024'] * 2 + ['energy_optimal']
    best = rows[-1]
    assert all(best['energy'] <= r['energy'] + 1e-6 for r in rows)
    assert all(r['budget'] == 300 for r in rows)
    assert all(r['evaluations'] == 300 for r in rows[:-1])
    assert best['energy_wh'] == pytest.approx(best['energy'] / 3600)


def test_chosen_direction_methods(tmp_path):
    from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
    cost = PathCost.from_config(load_robot_config(resolve_robot_config('pioneer3at')))
    settings = Settings(300, PARAMS.xie, PARAMS.ga, PARAMS.pso, directions=2,
                        screen_evaluations=50)
    methods = ['energy_ga', 'xie2024', 'energy_direction_ga', 'xie2024_direction']
    logs = []
    rows = run([lawn_by_name('two_beds')], methods, [0, 1], settings, cost, 'pioneer3at', 0.5,
               log=logs.append)
    by = {m: [r for r in rows if r['method'] == m] for m in methods}
    assert all(len(v) == 2 for v in by.values())
    # two_beds is wider than it is tall, so the energy choice turns the lanes along x.
    assert any('direction: lanes at 0 deg' in line for line in logs)
    assert all(r['lane_angle'] == 90 for m in ('energy_ga', 'xie2024') for r in by[m])
    assert all(r['lane_angle'] == 0 for m in methods[2:] for r in by[m])
    # Our method pays for the choice (2 directions x 50) from the shared budget.
    assert all(r['direction_evaluations'] == 100 and r['evaluations'] == 300
               for r in by['energy_direction_ga'])
    assert all(r['direction_evaluations'] == 0 and r['evaluations'] == 300
               for m in methods if m != 'energy_direction_ga' for r in by[m])
    for seed in (0, 1):
        assert by['energy_direction_ga'][seed]['energy'] < by['energy_ga'][seed]['energy']
        assert by['xie2024_direction'][seed]['energy'] < by['xie2024'][seed]['energy']
    lawn_area = lawn_by_name('two_beds').area.area
    for r in rows:
        assert r['energy_per_area'] == pytest.approx(
            r['energy'] / (r['coverage'] / 100 * lawn_area))
    # A budget that the choice would use up skips our method, and no energy model skips both.
    tight = Settings(100, PARAMS.xie, PARAMS.ga, PARAMS.pso, directions=2, screen_evaluations=50)
    rows = run([lawn_by_name('two_beds')], methods[2:], [0], tight, cost, 'pioneer3at', 0.5,
               log=lambda s: None)
    assert [r['method'] for r in rows] == ['xie2024_direction']
    assert run([lawn_by_name('two_beds')], methods[2:], [0], settings, PathCost(), 'none', 0.5,
               log=lambda s: None) == []


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
          '--xie-population', '20', '--xie-ants', '10', '--out', str(out)])
    assert 'two_beds\txie2024\t2' in capsys.readouterr().out
    with open(out, encoding='utf-8') as f:
        assert len(list(csv.DictReader(f))) == 2
    with pytest.raises(SystemExit):
        main(['--maps', 'moon'])
