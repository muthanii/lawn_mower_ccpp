"""Plot one Gazebo coverage run from the evaluator's trajectory.csv.

    PYTHONPATH=src/lawn_mower_planning:src/lawn_mower_robot:src/lawn_mower_navigation \
        python3 docs/tools/plot_run.py LAWN RESULTS_DIR OUT.png

Left: the path the robot drove (ground truth), with the blade cutting or not.
Right: the mowed area, the driven path widened by the cutting width, as the
evaluator scores it.
"""

import csv
import math
import os
import sys

import matplotlib

matplotlib.use('Agg')
from matplotlib import pyplot as plt  # noqa: E402
from matplotlib.patches import PathPatch  # noqa: E402
from matplotlib.path import Path  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

from lawn_mower_navigation.evaluation import Sample, coverage, piece_swaths, thin  # noqa: E402
from lawn_mower_navigation.lawns import find_lawn  # noqa: E402
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config  # noqa: E402

BLUE, ORANGE, INK, LAWN, EDGE, MOWED = '#2a78d6', '#eb6834', '#1f1f1e', '#eef5ea', '#5a8f4e', '#7cc46a'


def patch(geom, **kw):
    geoms = getattr(geom, 'geoms', [geom])
    verts, codes = [], []
    for g in geoms:
        if g.geom_type != 'Polygon':
            continue
        for ring in (g.exterior, *g.interiors):
            xy = list(ring.coords)
            verts += xy
            codes += [Path.MOVETO] + [Path.LINETO] * (len(xy) - 2) + [Path.CLOSEPOLY]
    return PathPatch(Path(verts, codes), **kw)


def main(lawn_name, results_dir, out, robot='faculty_robot_estimate'):
    lawn = find_lawn(lawn_name)
    width = load_robot_config(resolve_robot_config(robot)).mower.cutting_width
    with open(os.path.join(results_dir, 'trajectory.csv')) as f:
        samples = [Sample(float(r['t']), float(r['x']), float(r['y']), float(r['heading']),
                          r['cutting'] in ('1', 'True', 'true')) for r in csv.DictReader(f)]
    score = coverage(samples, lawn.area, width)
    mowed = unary_union(piece_swaths(thin(samples), width)).intersection(lawn.area)

    plt.rcParams.update({'font.family': 'serif', 'font.size': 9, 'axes.titlesize': 9})
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.9))
    x0, y0, x1, y1 = lawn.area.bounds
    for ax in axes:
        ax.add_patch(patch(lawn.area, fc=LAWN, ec=EDGE, lw=1.2, zorder=1))
        for hole in lawn.area.interiors:
            ax.add_patch(patch(Polygon(hole), fc='#8a8780', ec='none', zorder=4))
        ax.set_xlim(x0 - 0.5, x1 + 0.5)
        ax.set_ylim(y0 - 0.5, y1 + 0.5)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')

    ax = axes[0]
    run = [samples[0]]
    for s in samples[1:]:
        run.append(s)
        if s.cutting != run[0].cutting:
            xs, ys = [p.x for p in run], [p.y for p in run]
            ax.plot(xs, ys, color=BLUE if run[0].cutting else ORANGE, lw=0.9,
                    ls='-' if run[0].cutting else (0, (3, 2)), zorder=3)
            run = [s]
    ax.plot([p.x for p in run], [p.y for p in run], color=BLUE if run[0].cutting else ORANGE,
            lw=0.9, zorder=3)
    ax.plot([], [], color=BLUE, label='Blade cutting')
    ax.plot([], [], color=ORANGE, ls=(0, (3, 2)), label='Blade off')
    s0 = samples[0]
    ax.plot([s0.x], [s0.y], marker=(3, 0, math.degrees(s0.heading) - 90), ms=9, color=INK,
            ls='none', zorder=5, label='Start')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=3, frameon=False)
    ax.set_title(f'Driven path, {lawn.name}: {samples[-1].t - samples[0].t:.0f} s simulated')

    ax = axes[1]
    ax.add_patch(patch(mowed, fc=MOWED, ec='none', alpha=0.9, zorder=2))
    ax.set_title(f'Mowed area: {score["coverage"]:.1f} % coverage, '
                 f'{score["overlap"]:.1f} % overlap')
    ax.plot([], [], color=MOWED, lw=6, label='Mowed')
    ax.plot([], [], color=LAWN, lw=6, label='Not mowed')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=2, frameon=False)
    fig.tight_layout()
    fig.savefig(out, dpi=200, bbox_inches='tight')
    print(f'{out}: coverage {score["coverage"]:.1f} %, overlap {score["overlap"]:.1f} %')


if __name__ == '__main__':
    main(*sys.argv[1:])
