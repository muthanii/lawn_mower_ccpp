from lawn_mower_planning import tuning
from lawn_mower_planning.maps import shared_lawns
from lawn_mower_planning.path_cost import PathCost
from lawn_mower_planning.problem import build_problem


def test_tuning_lawns_are_small_and_not_test_lawns():
    lawns = tuning.tuning_lawns()
    assert len(lawns) == 10
    assert not {lawn.name for lawn in lawns} & {lawn.name for lawn in shared_lawns()}
    for lawn in lawns:
        n = build_problem(lawn.area, 0.5, start=lawn.start).table.n_cells
        assert n <= 8  # brute force gives the optimum


def test_every_method_gets_the_same_number_of_settings():
    assert {len(grid) for grid in tuning.GRIDS.values()} == {tuning.GRID_SIZE}
    assert set(tuning.GRIDS) == set(tuning.DEFAULTS)


def test_tune_scores_against_each_objective(monkeypatch):
    from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
    cost = PathCost.from_config(load_robot_config(resolve_robot_config('pioneer3at')))
    monkeypatch.setattr(tuning, 'TUNING_SEEDS', (100,))
    monkeypatch.setattr(tuning, 'BUDGETS', (300,))
    monkeypatch.setattr(tuning, 'RUN_SEEDS', (0,))
    small = {m: [dict(grid[0], **({'population': 20} if 'population' in grid[0] else
                                  {'particles': 20})),
                 dict(grid[1], **({'population': 30} if 'population' in grid[1] else
                                  {'particles': 30}))]
             for m, grid in tuning.GRIDS.items()}
    for grid in small.values():
        for setting in grid:
            if 'ants' in setting:
                setting['ants'] = 10
    monkeypatch.setattr(tuning, 'GRIDS', small)
    cases = tuning.tuning_cases(cost)
    rows, best = tuning.tune(cases, log=lambda s: None)
    assert len(rows) == 4 * 2  # methods x settings x lawns x budgets x seeds
    assert all(r['gap'] >= -1e-9 for r in rows)  # nothing beats the exact optimum
    assert all(best[m] in small[m] for m in small)
