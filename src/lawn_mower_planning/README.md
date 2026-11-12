# lawn_mower_planning

The coverage planner and its cell ordering. The pipeline splits the lawn into cells (BCD), covers each cell with back-and-forth lanes and joins the cells with A* paths. An ordering method then decides the order in which the cells are visited. Our method also chooses the direction of the decomposition by energy. The package has the parts that every ordering method shares, and the Xie et al. (2024) HGACO baseline (issue #21).

| Module | Content |
| --- | --- |
| `problem.py` | `build_problem`: lawn → cells → lanes → A* transitions → `TransitionTable`. `CoverageProblem.path` and `.coverage` turn a plan into waypoints, coverage % and overlap %. |
| `decomposition.py` | Boustrophedon cell decomposition (BCD) of a polygon with obstacle holes. |
| `lanes.py` | Back-and-forth lanes in one cell, in the four start-corner variants. |
| `roadmap.py` | A* on the visibility graph of the free area: exact shortest paths among polygonal obstacles. |
| `path_cost.py` | Length, turn count and energy of a polyline. Energy comes from the `lawn_mower_robot` skid-steer model. |
| `transitions.py` | `TransitionTable`, the precomputed table that every ordering method reads. Also the exact variant decoder (`best_variants`) and `Plan`. |
| `brute_force.py` | Exact orderings (plans and closed tours) by trying every permutation, for maps of up to 9 cells. Used for sanity checks. |
| `maps.py` | The shared test lawns: 10 lawns with 7 to 13 cells each. |
| `experiment.py` | The experiment runner: every method on every lawn and seed, one CSV row per run. |
| `analysis.py` | Significance tests and the LaTeX result tables for the paper. |
| `tuning.py` | The tuning protocol: the same tuning effort for every method, on separate tuning lawns. |
| `energy_order.py` | Our method, the ordering: GA or PSO that orders the cells by modelled energy. |
| `direction.py` | Our method, the decomposition: the BCD direction with the least modelled energy, from the directions of the lawn's edges. |
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
2. **Cells.** BCD with a sweep line along the lanes. `lane_angle` sets the lane heading in the world frame (default: along y). The geometry is computed in a frame where the lanes are vertical, and `path` turns it back. The vertices are snapped to a 1 nm grid first, so that every slab boundary passes exactly through its vertices (see [Decomposition fix](#decomposition-fix)).
3. **Lanes.** A cell `w` wide gets `ceil(w / width)` lanes, spread evenly. The robot changes lane along a straight line, or along the cell boundary where the straight line would leave the cell. Each cell can sweep at any angle in `sweep_angles` (radians, counterclockwise from the BCD direction; default `(0,)`, which gives every cell the same vertical lanes). The variants are the sweep angles times the 4 start corners: variant `a` is angle `a // 4` and corner `a % 4`. A cell that a turned lane would cross more than once cannot be swept back and forth in one piece, so that angle gets `inf` for that cell. Angle 0 is always usable, because BCD cells are monotone along the sweep line.
4. **Transitions.** A* on the visibility graph, from the exit of each `(cell, variant)` to the entry of every other one, and from the robot start.
5. **Metrics.** The robot drives each straight from rest and turns in place at each vertex. One vertex with a heading change of more than 1° is one turn, so a lane change counts 2. The same rule applies inside cells, on transitions and at the junctions between them, including the turn into and out of each lane. So the turn count of a plan equals the turn count of its assembled path (tested). Energy is the sum of `SkidSteerEnergyModel.straight` and `.turn` over the segments and turns, at the robot's maximum linear and angular speed.

`coverage` buffers the whole path, transitions included, by `width / 2`. Coverage is the part of the lawn inside this swath. Overlap is the area covered more than once, as a share of the swath. The clearance band along the lawn edges is never covered, so coverage stays below 100 % on every map. This is the same for every ordering method.

Known simplifications, to state in the paper:

- Point turns (stop-turn-go) everywhere. No arc turns at lane ends.
- The A* paths are shortest in length, not in energy or turns, for both methods.
- The cutter is on for transitions too, so transitions count toward coverage and overlap.

## Transition table

A cell can be covered in `k` variants: the sweep angle, the start corner and the lane direction. The variant fixes where the robot enters and leaves the cell. For each metric (`length`, `turns`, `energy`, ...) the table holds:

| Array | Shape | Cost of |
| --- | --- | --- |
| `cell[m]` | `(n, k)` | Covering cell `c` in variant `a`. |
| `transition[m]` | `(n, k, n, k)` | The A* path from the exit of `(i, a)` to the entry of `(j, b)`. |
| `start[m]` | `(n, k)` | Driving from the robot start to the entry of `(c, a)`. `None` for a free start. |

`inf` marks a variant that cannot be used. An ordering method on the table sees only a weighted sum of the metrics, `table.combine({metric: weight})`. Our method's objective is `{'energy': 1}`, and the length-and-turns ablation's is `{'length': 1, 'turns': w_T}`. Xie et al.'s own objective is the centroid tour, which uses the cell geometry instead of the table (see below). Every method uses the same cells, lanes and A* paths.

For a fixed cell order, `best_variants` finds the best variant of each cell exactly, by dynamic programming over the order in O(n k²). So the metaheuristics search only over permutations, and every method that reads the table also picks each cell's sweep angle exactly for its order. `Plan.metrics` sums every metric in the table along the chosen plan. This means a baseline run also reports energy from our model, although its objective does not use energy.

## Test lawns

`maps.shared_lawns()` returns the lawns that every method runs on. The lawns are polygons in metres with obstacle holes, and each has a robot start pose. With the default 0.5 m cutting width each lawn gives 7 to 13 cells on the default decomposition (`test/test_maps.py` checks the agreed range of 6 to 15).

| Lawn | Cells | Area (m²) | Content |
| --- | --- | --- | --- |
| `two_beds` | 7 | 216 | Small rectangle, two flower beds. |
| `backyard` | 7 | 396 | Shed in a corner, a bed and a tree. |
| `l_shape` | 7 | 452 | L-shaped lawn, two beds. |
| `three_beds` | 10 | 400 | Rectangle, three staggered beds. |
| `garden` | 10 | 689 | Irregular boundary, a tree, a triangular bed and a pond. |
| `park_trees` | 13 | 975 | Large rectangle, four round trees. |
| `sports_edge` | 13 | 1273 | Large rectangle, beds and a tree spread out. |
| `random_1` | 10 | 576 | 3 random obstacles (seed 1). |
| `random_2` | 10 | 583 | 3 random obstacles (seed 2). |
| `random_3` | 7 | 591 | 2 random obstacles (seed 3). |

Round obstacles are octagons. `random_lawn(seed)` places seeded random beds and trees at least 2 m apart, and draws again until the lawn has 6 to 15 cells. The same seed always gives the same lawn.

### Decomposition fix

Up to v1.10.0 the BCD rounded the slab boundaries to 9 decimals but not the polygon. An obstacle edge whose x is not a round number then started a rounding error after its slab boundary, and the sliver in between joined the cells above and below the obstacle into one cell. The lanes cover only one part of such a cell, so a part of the lawn was left out. This happened on the three random lawns only (4, 4 and 1 cells). Coverage on `random_1` and `random_2` was about 91 % for every method, and it is about 96 % now. The hand-made lawns have round coordinates and are unchanged.

The fix snaps the polygon to the same grid. It also changes which draw of `random_lawn` first has 6 to 15 cells, so `random_1`, `random_2` and several tuning lawns are now different lawns from those of v1.10.0. Results on the random lawns from before the fix (`paper/data/ordering-pioneer3at.csv` and `ordering-pioneer3at-angles.csv`) are not comparable with the new ones.

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
| `--sweep-angles` | 0 45 90 135 | Lane angles each cell may use [deg from the BCD direction]. Must include 0. `--sweep-angles 0` gives the fixed vertical lanes of v1.9.0. The analysis refuses a CSV that mixes angle sets. |
| `--directions` | 8 | Candidate decomposition directions for `energy_direction_ga` and `xie2024_direction`, the default direction included. |
| `--screen-evaluations` | 500 | Evaluated orders to score one candidate direction. |
| `--out` | `results.csv` | CSV file. New rows are appended. |

Methods:

| Method | What it is |
| --- | --- |
| `energy_direction_ga` | Our full method: the decomposition direction chosen by energy, then the GA order. The choice is paid from the budget. |
| `energy_ga` | Our ordering alone, with the GA, on the default decomposition. |
| `energy_pso` | Our ordering alone, with the PSO, on the default decomposition. |
| `xie2024` | The baseline: Xie et al.'s HGACO tour over cell centroids, with nearest-corner bridging, on the default decomposition. |
| `xie2024_direction` | Ablation: the baseline's ordering on our chosen decomposition. It gets the direction for free, on top of its full budget. |
| `xie_length_turns` | Ablation: the same HGACO, minimising length + turns on the shared table. |
| `energy_optimal` | Reference: the exact energy-optimal plan by brute force. It runs once per lawn (it does not depend on the seed), and only on lawns with up to 8 cells. |

To add a method, register a function `(problem, settings, seed) -> RunResult` in `experiment.METHODS`. `settings.budgeted(params)` gives the method its parameters with the shared budget.

CSV columns: `map`, `cells` (of the decomposition the method planned on), `method`, `seed`, `budget`, `robot`, `width`, `sweep_angles` (degrees, space-separated), `lane_angle` (world heading of the decomposition's lanes [deg]; 90 is the default), `cost` (the method's own objective), `length` [m], `turns`, `energy` [J], `energy_wh`, `energy_per_area` (energy per mowed square metre [J/m²]: energy / (coverage × lawn area)), `connection_length` (path outside the cells: start link and A* transitions [m]), `coverage` [%], `overlap` [%], `seconds` (with the time of the direction choice, for the two methods that use it), `evaluations` (with those of the direction choice, for `energy_direction_ga`), `ga_evaluations`, `direction_evaluations`, `order`, `variants`.

## Our method: energy-ordered cells

```python
from lawn_mower_planning.energy_order import EnergyGaParams, EnergyPsoParams, energy_ga_order, energy_pso_order

result = energy_ga_order(problem.table, EnergyGaParams(evaluations=20000), seed=0)
result = energy_pso_order(problem.table, EnergyPsoParams(evaluations=20000), seed=0)
```

Both searches minimise the modelled skid-steer energy of the whole plan: the coverage inside each cell, the A* transitions, the start link and every turn (`{'energy': 1}` on the shared table). Each cell order is decoded with the exact variant DP, so every order is scored with its best start corners. Each decode counts as one evaluation, against the same budget as the baseline. Set `weights` to minimise another mix of table metrics.

| Search | Encoding and operators | Defaults |
| --- | --- | --- |
| GA (`energy_ga_order`) | Cell permutation. Tournament selection (3), ordered crossover (OX), inversion mutation, elitism (2). A child that duplicates a member of the new generation is mutated again (up to 10 times), which keeps the population diverse. | population 50, crossover 0.9, mutation 0.3 |
| PSO (`energy_pso_order`) | Random keys: a particle is a vector of n keys in [0, 1], and the cell order is `argsort(keys)`. Clerc–Kennedy constriction (inertia 0.7298, c₁ = c₂ = 1.49618), velocity clamped to ±1.0, ring topology (each particle follows the best of itself and its two neighbours). A particle whose own best has not improved for 20 moves is drawn again. | 60 particles |

**Tuning.** See [Tuning](#tuning): every method was tuned with the same protocol. The diversity rules (no duplicates in the GA, ring and restarts in the PSO) were added during development, because without them both searches stalled after a few hundred evaluations. That early work also used four of the 7-cell test lawns. The final settings come only from the separate tuning lawns.

**Sanity checks** (`test/test_energy_order.py`). With 2000 evaluations (against 7! = 5040 for brute force), the GA finds the exact energy optimum on five synthetic 7-cell tables. The PSO stays within 1 % of the optimum on the same tables.

## Our method: energy-chosen decomposition direction

```python
from lawn_mower_planning.direction import choose_direction

choice = choose_direction(lawn, width=0.5, path_cost=cost, start=(1, 1, 0), sweep_angles=angles)
choice.problem       # the CoverageProblem of the chosen direction
choice.lane_angle    # its lane heading [rad]
choice.scores        # the screening energy of every candidate
choice.evaluations   # evaluated orders the choice cost
```

The ordering can only change the connections between the cells, which are a few percent of the energy. The direction of the decomposition fixes the cells and the number of lanes, and so most of the turns. A point turn is expensive for a skid-steer robot: one 90° turn costs as much as 3.3 m of straight driving with the Pioneer 3-AT placeholder, and turns are about 30 % of the modelled energy on the default decomposition. So `energy_direction_ga` also chooses the direction by energy:

1. **Candidates.** The lane headings parallel to an edge of the lawn boundary or of an obstacle. A sweep that needs the fewest lanes for a convex polygon is parallel to one of its edges (Huang, 2001). The candidates are ranked by the total length of their edges and the first 8 are kept (`--directions`). The default heading (along y) is always one of them.
2. **Score.** Each candidate is decomposed, and a short GA run (500 evaluations, seed 0, `--screen-evaluations`) gives its energy.
3. **Choice.** The candidate with the least energy. The GA then orders the cells of that decomposition with the rest of the budget.

The choice is deterministic for a lawn. Its evaluations (500 per candidate, so 1000 to 4000 on the test lawns) come out of the method's 20000, and its time is in `seconds`.

**What the comparison means.** Xie et al. do not search over the decomposition direction: their sub-regions come from extending the obstacle edges of an axis-aligned grid map. So `xie2024` stays on the default decomposition, and `energy_direction_ga` against `xie2024` compares the two complete methods. This difference has two parts, and the paper must report both:

- `xie2024_direction` against `xie2024`: the direction alone, with their ordering.
- `energy_direction_ga` against `xie2024_direction`: the ordering alone, on the same decomposition.

The default lanes run along y, and most test lawns are wider than they are tall, so the default is the worse of the two axis directions on most of them. The gain of the direction depends on this, and a reader can object that a different default would shrink it. Report the ordering-only difference next to the full one, and say that the direction choice is not specific to our ordering: any method can use it.

**Coverage.** The lanes end at the clearance band, so the band at the lane ends is not mowed, and a different direction leaves a different band. Coverage therefore changes a little with the direction. `energy_per_area` (energy per mowed square metre) corrects for this, and the tables report it next to the energy.

## Analysis

```bash
ros2 run lawn_mower_planning analyze_results paper/data/ordering-pioneer3at-direction.csv --tables paper/tables
```

`analyze_results` reads the experiment CSV, prints every test and writes two LaTeX tables, which `paper/sections/results.tex` includes:

| File | Content |
| --- | --- |
| `paper/tables/ordering-energy.tex` | Energy per lawn and method (mean ± SD over the seeds), the difference of our method to Xie et al. and its p-value per lawn, and the test across lawns. |
| `paper/tables/ordering-metrics.tex` | Energy, energy per mowed area, length, turns, connection length, coverage, overlap and planning time: our method against Xie et al., and against Xie et al.'s ordering on our chosen decomposition. |

"Our method" in the tables is `energy_direction_ga` when the CSV has it. For a CSV without it, the tables compare `energy_ga` with Xie et al. and with the length-and-turns ablation, as in v1.10.0. The printed report has every comparison either way.

The tests:

- **Pairing.** Runs are paired by lawn and seed. Each pair gives a relative difference, `100 (ours / theirs − 1)` [%].
- **Per lawn.** A two-sided Wilcoxon signed-rank test over the seeds of the lawn, with Holm correction over the lawns. With 10 seeds the smallest possible p is 2/2¹⁰ ≈ 0.002, so 0.020 after Holm over 10 lawns.
- **Across lawns.** A Wilcoxon test over the mean difference of each lawn. Seeds on one lawn are not independent, so the lawn is the unit; relative differences make large and small lawns weigh the same.
- **Effect size.** The matched-pairs rank-biserial correlation r = (W⁺ − W⁻)/(W⁺ + W⁻): −1 when ours is lower in every pair.
- **Ties.** Pairs with no difference are left out (Wilcoxon's method). A lawn where all pairs tie gets no test (shown as "--").
- **Reference.** The exact optimum is deterministic, so it is reported but not tested.

The analysis refuses a CSV that mixes robots. Analyse one robot at a time, and regenerate the tables after every run. The CSV behind the current tables is `paper/data/ordering-pioneer3at-direction.csv`: the Pioneer 3-AT placeholder, so the tables are pipeline checks until the faculty robot runs replace them.

Results with the placeholder (10 lawns, 10 seeds, 20000 evaluations, sweep angles 0 45 90 135), as the mean over the lawns of the relative difference. Every difference in energy has p = 0.002 across the lawns and r = −1.00:

| Comparison | What differs | Energy | Energy per mowed area |
| --- | --- | --- | --- |
| `energy_direction_ga` against `xie2024` | Direction and ordering: the two complete methods | −10.30 % | −9.60 % |
| `xie2024_direction` against `xie2024` | Direction alone, with Xie et al.'s ordering | −7.28 % | −6.82 % |
| `energy_direction_ga` against `xie2024_direction` | Ordering alone, on our decomposition | −3.26 % | −2.99 % |
| `energy_ga` against `xie2024` | Ordering alone, on the default decomposition | −1.97 % | −1.61 % |

The full method has 24 % fewer turns than Xie et al., 0.8 % less coverage (95.1 % against 95.9 %) and takes 2.4 times as long to plan (5.7 s against 2.4 s), because it decomposes the lawn once per candidate direction. The direction choice picked lanes along x on all 10 lawns. All 10 are wider than they are tall, so the lawns do not yet test a case where the default direction is the better one.

The two older CSVs in `paper/data/` (`ordering-pioneer3at.csv`: fixed vertical lanes; `ordering-pioneer3at-angles.csv`: sweep angles) are from before the [decomposition fix](#decomposition-fix).


```bash
ros2 run lawn_mower_planning run_tuning --out tuning.csv
```

`tuning.py` tunes every method with the same effort, as the fairness rules of issue #21 require:

- **Lawns.** 10 tuning lawns (`random_lawn` seeds 100–109, 24 × 16 m, 1–2 obstacles, 6–7 cells). None of them is a shared test lawn, so tuning never sees the test data.
- **Target.** Each method is scored against the exact optimum of its own objective, from brute force: energy for `energy_ga` and `energy_pso`, the centroid tour for `xie2024`, and length + turns for `xie_length_turns`.
- **Effort.** 5 settings per method, the first being the default before tuning. Each setting runs at budgets 1000 and 2000 with seeds 0–2, so 60 runs per setting.
- **Choice.** The setting with the most runs at the optimum wins. Ties go to the lower mean gap, then to the earlier setting, so a full tie keeps the default.

Results (runs at the optimum, of 60):

| Method | Settings tried (runs at the optimum) | Chosen |
| --- | --- | --- |
| `energy_ga` | pop 50 mut 0.5 (48), pop 100 mut 0.5 (48), pop 50 mut 0.3 (54), pop 100 mut 0.3 (50), pop 30 mut 0.5 (51) | pop 50, mutation 0.3 |
| `energy_pso` | 60 particles v 0.5 (33), 40 v 0.5 (31), 20 v 0.5 (25), 60 v 0.2 (24), 60 v 1.0 (34) | 60 particles, max velocity 1.0 |
| `xie2024` | All five settings: 60 | The paper's settings (pop 400, 100 ants, GA share 0.5), by the tie rule |
| `xie_length_turns` | Paper (41), pop 100 ants 50 share 0.5 (40), pop 50 ants 25 (39), pop 100 ants 50 share 0.3 (38), share 0.7 (38) | The paper's settings |

These numbers are from the tuning lawns after the [decomposition fix](#decomposition-fix), which changed several of them. The choices for `energy_ga` and `xie2024` are the same as before the fix. The PSO's maximum velocity went from 0.5 to 1.0, and the ablation went from pop 100, 50 ants, GA share 0.3 back to the paper's settings.

The chosen settings are the defaults: in `EnergyGaParams` and `EnergyPsoParams`, in `XieGaAcoParams` (the paper's settings) for `xie2024`, and in `experiment.TUNED_LENGTH_TURNS` for the ablation.

Notes for the paper:

- The centroid tour of a 6–7-cell lawn is easy: every `xie2024` setting finds it in every run, so tuning cannot separate the settings, and the paper's settings stay.
- The PSO is the weakest search on its own objective (34 of 60). Report `energy_ga` as the main form of our method, and the PSO as a variant.
- Tuning ran at budgets of 1000 and 2000, below the 20000 of the experiments, so the searches are tuned for a tight budget.

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
5. **Bridging** (Sec. 3.3). Each cell is entered at the corner nearest to the robot's current position. The lane parity fixes the exit corner. A* joins the cells. With several sweep angles, each cell first takes its own fewest-turns angle (ties broken by length, `CoverageProblem.fewest_turns_angle`), and the nearest corner is chosen among that angle's corners.

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
| Sweep direction | One direction per sub-region, from the rectangular decomposition. Not searched. | Each cell's fewest-turns angle from the shared `sweep_angles`, chosen before ordering. |

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
