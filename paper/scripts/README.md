# Figure scripts

The scripts write every figure of the paper to `paper/figures/` as a vector PDF. `main.tex` includes them from there.

| Script | Figures |
| --- | --- |
| `make_figures.py` | Data figures: `pipeline`, `test-lawns`, `energy-model`, `direction-choice`, `example-plans`, `energy-results`, `convergence`, `fuzzy-controller`. They use the planner, the robot config and `paper/data/ordering-pioneer3at-direction.csv`, the same as the result tables. |
| `make_schematics.py` | Electronics: `robot-architecture` (the whole robot, faculty parts and added parts), `power-schematic`, `controller-wiring`, `controller-states`. |
| `style.py` | Fonts, sizes, colours, and the overlap check. |

## Run

In the workspace root, with Matplotlib, Shapely and schemdraw installed (`pip install matplotlib shapely schemdraw`):

```bash
PYTHONPATH=src/lawn_mower_planning:src/lawn_mower_robot:src/lawn_mower_navigation python3 paper/scripts/make_figures.py
```

```bash
python3 paper/scripts/make_schematics.py
```

- `--only NAME ...` makes only the named figures.
- `--preview DIR` also writes PNG previews to `DIR`.

`make_figures.py` takes about 3 minutes. Most of it is the `convergence` figure, which runs the GA and the PSO 10 times on 4 lawns.

## Overlap check

Each figure is checked before it is saved. The check prints an `OVERLAP` line for every text that touches another text, a line, or the outline of a box. Text inside a box is fine. Fix every `OVERLAP` line before you commit a figure.

## When to run again

- After a new experiment CSV: run `make_figures.py`. The data figures, like the tables, show the Pioneer 3-AT placeholder until the faculty robot runs are in. Change `ROBOT` and `CSV` at the top of `make_figures.py` for the faculty robot.
- After a change to the electronics: change `make_schematics.py`. The parts follow `src/lawn_mower_electronics`. Fuse ratings, the DC-DC converters and the controller board are not chosen yet, so the schematics show them as generic parts.
