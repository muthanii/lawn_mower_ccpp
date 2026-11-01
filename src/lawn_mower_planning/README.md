# lawn_mower_planning

The coverage planner and its cell ordering. The pipeline splits the lawn into cells (BCD), covers each cell with back-and-forth lanes and joins the cells with A* paths. An ordering method then decides the order in which the cells are visited. The package has the parts that every ordering method shares, and the Xie et al. (2024) HGACO baseline (issue #21).

| Module | Content |
| --- | --- |
| `problem.py` | `build_problem`: lawn → cells → lanes → A* transitions → `TransitionTable`. `CoverageProblem.path` and `.coverage` turn a plan into waypoints, coverage % and overlap %. |
| `decomposition.py` | Boustrophedon cell decomposition (BCD) of a polygon with obstacle holes. |
| `lanes.py` | Back-and-forth lanes in one cell, in the four start-corner variants. |
| `roadmap.py` | A* on the visibility graph of the free area: exact shortest paths among polygonal obstacles. |
| `path_cost.py` | Length, turn count and energy of a polyline. Energy comes from the `lawn_mower_robot` skid-steer model. |
| `transitions.py` | `TransitionTable`, the precomputed table that every ordering method reads. Also the exact variant decoder (`best_variants`) and `Plan`. |
| `brute_force.py` | Exact orderings (plans and closed tours) by trying every permutation, for maps of up to 9 cells. Used for sanity checks. |
| `maps.py` | The shared test lawns: 10 lawns with 7 to 15 cells each. |
| `experiment.py` | The experiment runner: every method on every lawn and seed, one CSV row per run. |
| `energy_order.py` | Our method: GA or PSO that orders the cells by modelled energy. |
| `operators.py`, `search.py` | Permutation operators, and the evaluation budget and result shared by the searches. |
| `xie_ga_aco.py` | Re-implementation of the Xie et al. (2024) HGACO ordering, the main baseline, and a length-and-turns ablation. |

## Pipeline

```python
from shapely.geometry import box
from lawn_mower_planning.path_cost import PathCost
from lawn_mower_planning.problem import build_problem
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config

lawn = box(0, 0, 20, 10).difference(box(5, 3, 8, 6))   # obstacles are holes
cost = PathCost.from_config(load_robot_config(resolve_robot_config('pioneer3at')))
problem = build_problem(lawn, width=0.5, path_cost=cost, start=(1, 1, 0))
problem.table          # TransitionTable with length, turns and energy
```

1. **Free area.** The lawn shrinks by `clearance` (default `width / 2`), so that the robot centre stays that far from the edges and obstacles.
2. **Cells.** BCD with a sweep line along the lanes. `lane_angle` sets the lane heading in the world frame (default: along y). The geometry is computed in a frame where the lanes are vertical, and `path` turns it back.
3. **Lanes.** A cell `w` wide gets `ceil(w / width)` lanes, spread evenly. The robot changes lane along a straight line, or along the cell boundary where the straight line would leave the cell. The 4 variants are the start corners.
4. **Transitions.** A* on the visibility graph, from the exit of each `(cell, variant)` to the entry of every other one, and from the robot start.
5. **Metrics.** The robot drives each straight from rest and turns in place at each vertex. One vertex with a heading change of more than 1° is one turn, so a lane change counts 2. The same rule applies inside cells, on transitions and at the junctions between them, including the turn into and out of each lane. So the turn count of a plan equals the turn count of its assembled path (tested). Energy is the sum of `SkidSteerEnergyModel.straight` and `.turn` over the segments and turns, at the robot's maximum linear and angular speed.

`coverage` buffers the whole path, transitions included, by `width / 2`. Coverage is the part of the lawn inside this swath. Overlap is the area covered more than once, as a share of the swath. The clearance band along the lawn edges is never covered, so coverage stays below 100 % on every map. This is the same for every ordering method.

Known simplifications, to state in the paper:

- Point turns (stop-turn-go) everywhere. No arc turns at lane ends.
- The A* paths are shortest in length, not in energy or turns, for both methods.
- The cutter is on for transitions too, so transitions count toward coverage and overlap.

## Transition table

A cell can be covered in `k` variants: the start corner and lane direction. The variant fixes where the robot enters and leaves the cell. For each metric (`length`, `turns`, `energy`, ...) the table holds:

| Array | Shape | Cost of |
| --- | --- | --- |
| `cell[m]` | `(n, k)` | Covering cell `c` in variant `a`. |
| `transition[m]` | `(n, k, n, k)` | The A* path from the exit of `(i, a)` to the entry of `(j, b)`. |
| `start[m]` | `(n, k)` | Driving from the robot start to the entry of `(c, a)`. `None` for a free start. |

`inf` marks a variant that cannot be used. An ordering method on the table sees only a weighted sum of the metrics, `table.combine({metric: weight})`. Our method's objective is `{'energy': 1}`, and the length-and-turns ablation's is `{'length': 1, 'turns': w_T}`. Xie et al.'s own objective is the centroid tour, which uses the cell geometry instead of the table (see below). Every method uses the same cells, lanes and A* paths.

For a fixed cell order, `best_variants` finds the best variant of each cell exactly, by dynamic programming over the order in O(n k²). So the metaheuristics search only over permutations. `Plan.metrics` sums every metric in the table along the chosen plan. This means a baseline run also reports energy from our model, although its objective does not use energy.

## Test lawns

`maps.shared_lawns()` returns the lawns that every method runs on. The lawns are polygons in metres with obstacle holes, and each has a robot start pose. With the default 0.5 m cutting width each lawn gives 6 to 15 cells (`test/test_maps.py` checks this).

| Lawn | Cells | Area (m²) | Content |
| --- | --- | --- | --- |
| `two_beds` | 7 | 216 | Small rectangle, two flower beds. |
| `backyard` | 7 | 396 | Shed in a corner, a bed and a tree. |
| `l_shape` | 7 | 452 | L-shaped lawn, two beds. |
| `three_beds` | 10 | 400 | Rectangle, three staggered beds. |
| `garden` | 10 | 689 | Irregular boundary, a tree, a triangular bed and a pond. |
| `park_trees` | 13 | 975 | Large rectangle, four round trees. |
| `sports_edge` | 13 | 1273 | Large rectangle, beds and a tree spread out. |
| `random_1` | 10 | 551 | 4 random obstacles (seed 1). |
| `random_2` | 15 | 542 | 5 random obstacles (seed 2). |
| `random_3` | 7 | 591 | 2 random obstacles (seed 3). |

Round obstacles are octagons. `random_lawn(seed)` places seeded random beds and trees at least 2 m apart, and draws again until the lawn has 6 to 15 cells. The same seed always gives the same lawn.

## Experiments

```bash
ros2 run lawn_mower_planning run_experiments --robot pioneer3at --seeds 10 --budget 20000 --out results.csv
```

The runner builds each lawn once, with length, turns and energy from the robot config. Every method then plans on that same problem, with seeds 0 to N−1 and the same evaluation budget. It writes one CSV row per run as each run finishes, so a stopped run keeps its rows. Then it prints the mean ± standard deviation per lawn and method.

| Option | Default | Meaning |
| --- | --- | --- |
| `--robot` | `pioneer3at` | Robot config name or path. |
| `--maps` | all | Lawn names. |
| `--methods` | all | Methods from `experiment.METHODS`. |
| `--seeds` | 10 | Seeds 0 to N−1. |
| `--budget` | 20000 | Evaluated orders per run, the same for every method. |
| `--ga-population`, `--pso-particles` | 50, 60 | Our GA and PSO. |
| `--xie-population`, `--xie-ants`, `--xie-ga-share` | 400, 100, 0.5 | Xie et al.'s HGACO. |
| `--width` | 0.5 | Cutting width and lane spacing [m]. |
| `--out` | `results.csv` | CSV file. New rows are appended. |

Methods:

| Method | What it is |
| --- | --- |
| `energy_ga` | Our method, with the GA. |
| `energy_pso` | Our method, with the PSO. |
| `xie2024` | The baseline: Xie et al.'s HGACO tour over cell centroids, with nearest-corner bridging. |
| `xie_length_turns` | Ablation: the same HGACO, minimising length + turns on the shared table. |
| `energy_optimal` | Reference: the exact energy-optimal plan by brute force. It runs once per lawn (it does not depend on the seed), and only on lawns with up to 8 cells. |

To add a method, register a function `(problem, settings, seed) -> RunResult` in `experiment.METHODS`. `settings.budgeted(params)` gives the method its parameters with the shared budget.

CSV columns: `map`, `cells`, `method`, `seed`, `budget`, `robot`, `width`, `cost` (the method's own objective), `length` [m], `turns`, `energy` [J], `energy_wh`, `connection_length` (path outside the cells: start link and A* transitions [m]), `coverage` [%], `overlap` [%], `seconds`, `evaluations`, `ga_evaluations`, `order`, `variants`.

## Our method: energy-ordered cells

```python
from lawn_mower_planning.energy_order import EnergyGaParams, EnergyPsoParams, energy_ga_order, energy_pso_order

result = energy_ga_order(problem.table, EnergyGaParams(evaluations=20000), seed=0)
result = energy_pso_order(problem.table, EnergyPsoParams(evaluations=20000), seed=0)
```

Both searches minimise the modelled skid-steer energy of the whole plan: the coverage inside each cell, the A* transitions, the start link and every turn (`{'energy': 1}` on the shared table). Each cell order is decoded with the exact variant DP, so every order is scored with its best start corners. Each decode counts as one evaluation, against the same budget as the baseline. Set `weights` to minimise another mix of table metrics.

| Search | Encoding and operators | Defaults |
| --- | --- | --- |
| GA (`energy_ga_order`) | Cell permutation. Tournament selection (3), ordered crossover (OX), inversion mutation, elitism (2). A child that duplicates a member of the new generation is mutated again (up to 10 times), which keeps the population diverse. | population 50, crossover 0.9, mutation 0.5 |
| PSO (`energy_pso_order`) | Random keys: a particle is a vector of n keys in [0, 1], and the cell order is `argsort(keys)`. Clerc–Kennedy constriction (inertia 0.7298, c₁ = c₂ = 1.49618), velocity clamped to ±0.5, ring topology (each particle follows the best of itself and its two neighbours). A particle whose own best has not improved for 20 moves is drawn again. | 60 particles |

**Tuning.** Both methods were tuned with the same effort: a handful of settings each (population or swarm size, mutation rate, velocity limit, topology), scored on 10 small cases with known optima (six synthetic 7-cell tables and the four 7-cell test lawns) at 1000 and 2000 evaluations. The diversity rules (no duplicates in the GA, ring and restarts in the PSO) were the largest gains: without them both searches stall after a few hundred evaluations. Tune the Xie baseline with the same effort before final runs.

**Sanity checks** (`test/test_energy_order.py`). With 2000 evaluations (against 7! = 5040 for brute force), the GA finds the exact energy optimum on five synthetic 7-cell tables. The PSO stays within 1 % of the optimum on the same tables.

## Xie et al. (2024) HGACO baseline

> Xie, X., Yan, Z., Zhang, Z., Qin, Y., Jin, H., & Xu, M. (2024). Hybrid genetic ant colony optimization algorithm for full-coverage path planning of gardening pruning robots. *Intelligent Service Robotics*, 17, 661–683. https://doi.org/10.1007/s11370-024-00525-6

```python
from lawn_mower_planning.xie_ga_aco import XieGaAcoParams, length_turns_order, xie2024_order

result = xie2024_order(problem, XieGaAcoParams(evaluations=20000), seed=0)   # the baseline
result.plan.order, result.plan.variants   # visiting order and variant of each cell
result.row()                              # cost, length, turns, energy, seconds, evaluations, ...

ablation = length_turns_order(problem.table, XieGaAcoParams(evaluations=20000), seed=0)
```

### What Xie et al. do

The paper's objective is **not** length and turns. Xie et al. reduce the ordering to a closed travelling salesman tour over the sub-region centre points, with Euclidean distances (their Eqs. 1–3). Path length, turns, coverage and connection distance are only measured afterwards. `xie2024_order` follows the paper:

1. **Objective.** The length of a closed tour through the cell centroids.
2. **GA** (their Sec. 3.2.1, Table 2). The chromosome is a cell permutation, and fitness is 1 / tour length. The GA uses population N = 400, PMX crossover with P_c = 0.5, and inversion mutation with P_m = 0.9.
3. **Handover** (Sec. 3.2.3 (1)). The pheromone starts lower than in a plain ant colony everywhere, and higher on the edges of the GA's best tour.
4. **ACO** (Secs. 3.2.2–3.2.3, Table 1). An ant system: every ant deposits Q / L_k (Eqs. 4–6), with m = 100 ants, Q = 100 and η = 1 / d_ij. α and β change over the iterations with a sigmoid (Eqs. 9–10, α = 8, β = 1, μ = 0.95), and ρ follows a Gaussian density (Eqs. 11–12).
5. **Bridging** (Sec. 3.3). Each cell is entered at the corner nearest to the robot's current position. The lane parity fixes the exit corner. A* joins the cells.

`Plan.cost` is the centroid tour length. `Plan.metrics` gives length, turns and energy of the plan on the shared pipeline.

**Ablation.** `length_turns_order` runs the same HGACO on the shared `TransitionTable` and minimises `length + w_T · turns`, with the exact variant DP. This is the design in issue #21. It is not Xie et al.'s method, so report it as an ablation (Xie's search with a length-and-turns objective), if at all.

**Compute budget.** Each evaluated order counts as one evaluation, in both stages. `evaluations` is the total budget and `ga_share` splits it between the stages. It replaces the paper's G_max = 1000 and 200–500 ACO iterations, so all methods get the same budget. The paper's settings (N = 400, m = 100) are the defaults. Log `seconds` for a wall-clock comparison.

**Fallback.** `ga_share=1` runs only the GA. `ga_share=0` runs only the ant colony.

**Why not DEAP.** DEAP draws from the global `random` module, so a run could not be reproduced from its own seed. Also, rosdep has no `python3-deap` key, so CI could not install it. The operators are short, so they are written here on a NumPy generator.

**Sanity checks** (`test/test_xie_ga_aco.py`). With 4000 evaluations, `xie2024_order` finds the brute-force optimal tour on eight random 8-cell maps. The ablation stays within 1 % of the brute-force optimum on 7-cell tables, and on the pipeline's 7-cell lawn.

### What the paper leaves open, and our choice

State these in our paper as interpretations.

| Item | Paper | Our choice |
| --- | --- | --- |
| Eqs. 9–10 | Printed as `α' = α / (1 − e^−x) + μβ` and `β' = β e^−x / (1 − e^−x)`. The text says sigmoid, and the printed form divides by zero at x = 0. | `α' = α·sigmoid(x) + μβ`, `β' = β·(1 − sigmoid(x))`. |
| Sigmoid step x | "Determined based on iteration number". | x runs linearly from −6 to +6 over the ACO iterations (`sigmoid_span`). |
| Eqs. 11–12 | `ρ(t) = N(t/10) + 0.2`. Mean y and σ are not given, and the exponent's sign is printed as positive. | y = 0, σ = 1, negative exponent. ρ falls from 0.60 to 0.2 within about 30 iterations. Table 1's ρ = 0.5 is then unused. |
| Initial pheromone | "Lower than" a plain ant colony, "higher" on the GA's best tour. No values. | 0.5 everywhere, 2.0 on the GA tour (1.0 without a GA stage). |
| GA selection | Fitness = 1 / length; the selection method is not named. | Roulette wheel on fitness, and the best tour is kept (elitism). |
| Termination | G_max = 1000 GA generations; 200–500 ACO iterations, by map. | One shared budget of evaluated orders (fairness rule of issue #21). |
| Where the tour opens | A closed tour; the paper does not say where the robot starts. | At the cell whose centroid is nearest the robot start, in the tour's direction. |
| First cell's corner | Not stated. | The corner nearest the robot start. |

### Differences in the shared pipeline

These parts are shared by every method on purpose, so they are not re-implemented from the paper. Mention them when comparing with Xie et al.

| Part | Xie et al. | Here |
| --- | --- | --- |
| Map | Occupancy grid. Irregular obstacles are expanded to bounding rectangles. | Polygons. |
| Decomposition | Rectangles from extending obstacle edges, split when the aspect ratio exceeds 1:2. | Boustrophedon cells. |
| A* | 8-connected grid, mixed diagonal and Manhattan heuristic. | Visibility graph, exact shortest paths. |
| Turns | Reported, but no counting rule given. | One turn per vertex with a heading change of more than 1°. A lane change counts 2. |

### Fairness

- Run all methods on the same maps, with the same seeds and the same `evaluations` budget, using one `CoverageProblem` per map.
- Tune both methods with the same effort.
- Report this as a re-implementation of Xie et al.'s ordering. Do not compare with the numbers in their paper, because they used different maps and a different robot.
- Report their metrics (length, turns, coverage and connection distance) next to energy.

## Test

```bash
colcon test --packages-select lawn_mower_planning
```
