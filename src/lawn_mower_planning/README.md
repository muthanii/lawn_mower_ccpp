# lawn_mower_planning

Cell ordering for the coverage planner. The pipeline splits the lawn into cells (BCD), covers each cell with back-and-forth lanes and joins the cells with A*. This package decides the order in which the cells are visited. For now it has the parts that every ordering method shares, and the Xie et al. (2024) baseline (issue #21).

| Module | Content |
| --- | --- |
| `transitions.py` | `TransitionTable`, the precomputed table that every ordering method reads. Also the exact variant decoder (`best_variants`) and `Plan`. |
| `brute_force.py` | Exact ordering that tries every permutation, for maps of up to 9 cells. Used for sanity checks. |
| `xie_ga_aco.py` | Re-implementation of the Xie et al. (2024) GA–ACO ordering, the main baseline. |

The BCD, lane and A* code is not written yet. The tests use a synthetic lawn (`test/conftest.py`) in its place.

## Transition table

A cell can be covered in `k` variants: the start corner and lane direction. The variant fixes where the robot enters and leaves the cell. For each metric (`length`, `turns`, `energy`, ...) the table holds:

| Array | Shape | Cost of |
| --- | --- | --- |
| `cell[m]` | `(n, k)` | Covering cell `c` in variant `a`. |
| `transition[m]` | `(n, k, n, k)` | The A* path from the exit of `(i, a)` to the entry of `(j, b)`. |
| `start[m]` | `(n, k)` | Driving from the robot start to the entry of `(c, a)`. `None` for a free start. |

`inf` marks a variant that cannot be used. An ordering method sees only a weighted sum of the metrics, `table.combine({metric: weight})`. Xie et al.'s objective is `{'length': 1, 'turns': w_T}`. Our method's objective is `{'energy': 1}`. So both methods use the same cells, lanes and A* paths, and they differ only in the objective (an ablation).

For a fixed cell order, `best_variants` finds the best variant of each cell exactly, by dynamic programming over the order in O(n k²). So the metaheuristics search only over permutations. `Plan.metrics` sums every metric in the table along the chosen plan. This means a baseline run also reports energy from our model, although its objective does not use energy.

## Xie et al. (2024) GA–ACO baseline

> Xie, X., Yan, Z., Zhang, Z., Qin, Y., Jin, H., & Xu, M. (2024). Hybrid genetic ant colony optimization algorithm for full-coverage path planning of gardening pruning robots. *Intelligent Service Robotics*, 17, 661–683. https://doi.org/10.1007/s11370-024-00525-6

```python
from lawn_mower_planning.xie_ga_aco import XieGaAcoParams, xie_ga_aco_order

result = xie_ga_aco_order(table, XieGaAcoParams(evaluations=5000, turn_weight=1.0), seed=0)
result.plan.order, result.plan.variants   # visiting order and variant of each cell
result.row()                              # cost, length, turns, energy, seconds, evaluations, ...
```

The algorithm:

1. **Objective.** `J = length + w_T · turns`. `w_T` (`turn_weight`) is the cost of one turn, in metres of path.
2. **GA stage.** The chromosome is a cell permutation, and it is decoded with `best_variants`. The GA uses tournament selection, ordered crossover (OX), inversion mutation and elitism.
3. **Handover.** The `handover_elites` best GA orders lay the initial pheromone. An edge that is used by every elite starts near `tau_max`. All other edges start at `tau_min`.
4. **ACO stage.** A max–min ant system. An ant picks the next cell with probability ∝ `tau^alpha · eta^beta`, where `eta = 1 / cheapest A* transition` to that cell. The pheromone evaporates, the global best order deposits `1/J`, and the pheromone stays in `[tau_min, tau_max]`, with `tau_max = 1/(rho J_best)` and `tau_min = tau_max / 2n`. The robot start is an extra pheromone row.

**Compute budget.** Each decoded order counts as one evaluation, in both stages. `evaluations` is the total budget and `ga_share` splits it between the stages. To give our method the same budget, cap its decoded orders at the same number. Also log `seconds` for a wall-clock comparison.

**Fallback.** `ga_share=1` gives the GA-only version that the issue describes, without the ACO layer. `ga_share=0` runs ACO alone, which is useful for an ablation of the hybrid.

**Why not DEAP.** DEAP draws from the global `random` module, so a run could not be reproduced from its own seed. Also, rosdep has no `python3-deap` key, so CI could not install it. The operators are short, so they are written here on a NumPy generator.

**Sanity check.** `test/test_xie_ga_aco.py` compares the result with `brute_force_order` on ten 7-cell maps. With 1000 evaluations (a fifth of the 5040 orders) it finds the exact optimum on all ten maps.

### Interpretation, to check against the full text

The full text was not available when this code was written. The authors were asked for it on 24 Oct 2026. The abstract says only that a hybrid genetic ant colony algorithm orders the sub-regions, that A* joins them, and that the algorithm uses "a new update strategy of heuristic and pheromone factors". Every item below is our choice. When the paper arrives, check each item, change the code or the defaults where they differ, and state any item that stays ambiguous in our paper.

| Item | Our choice | Where |
| --- | --- | --- |
| Chromosome | Cell permutation. The entry corner of each cell is set by the exact DP, not encoded in the chromosome. | `best_variants` |
| Fitness | `length + w_T · turns`, with `w_T = 1` m per turn. The weight in the paper is unknown. | `XieGaAcoParams.turn_weight` |
| GA operators | Tournament (3), OX (0.9), inversion mutation (0.2), 2 elites, population 50. | `XieGaAcoParams` |
| GA → ACO handover | After a fixed share of the budget (0.5), the 10 best GA orders lay the pheromone. | `ga_share`, `handover_elites` |
| ACO heuristic | `1 / cheapest A* transition`, kept static. Their "new heuristic update strategy" is not known. | `_heuristic` |
| Pheromone update | Max–min ant system, global-best deposit, `rho = 0.1`, 20 ants, `alpha = 1`, `beta = 2`. Their "new pheromone update strategy" is not known. | `_ant_stage` |
| Start and end | The plan starts at the robot start (if given) and ends in the last cell, with no return. | `TransitionTable.start` |
| Turn count | The count that the table gives. The lane and A* code must count turns in the same way for both methods. | Table builder |

### Fairness

- Run both methods on the same maps, with the same seeds and the same `evaluations` budget, using one table per map.
- Tune both methods with the same effort.
- Report this as a re-implementation of Xie et al.'s ordering. Do not compare with the numbers in their paper, because they used different maps and a different robot.
- Report their metrics (length and turns) next to energy. Coverage and overlap come from the lanes, so they are equal for both methods unless the lane code changes. Log them when the pipeline exists.

## Test

```bash
colcon test --packages-select lawn_mower_planning
```
