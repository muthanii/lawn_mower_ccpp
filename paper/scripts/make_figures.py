"""Data figures of the paper, written to paper/figures/ as PDF.

Run from the repository root (no ROS install needed):

    PYTHONPATH=src/lawn_mower_planning:src/lawn_mower_robot:src/lawn_mower_navigation \
        python3 paper/scripts/make_figures.py [--only NAME ...] [--preview DIR]

The figures use the same pipeline, robot config and experiment CSV as the
result tables, so they show the Pioneer 3-AT placeholder until the faculty
robot runs are in. Re-run this script after the CSV is regenerated.
"""

import argparse
import csv
import math
import os
import statistics
import sys

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import PathPatch
from matplotlib.path import Path
from shapely import affinity
from shapely.geometry import MultiPolygon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import style  # noqa: E402
from style import (AQUA, BLUE, CELL_EDGE, FULL, GRID, INK, INK_2, LAWN, LAWN_EDGE,  # noqa: E402
                   MUTED, OBSTACLE, ORANGE, YELLOW, panel_label, save)

from lawn_mower_planning.direction import choose_direction  # noqa: E402
from lawn_mower_planning.energy_order import (EnergyGaParams, EnergyPsoParams,  # noqa: E402
                                              energy_ga_order, energy_pso_order)
from lawn_mower_planning.experiment import SWEEP_ANGLES_DEG  # noqa: E402
from lawn_mower_planning.maps import DEFAULT_WIDTH, lawn_by_name, shared_lawns  # noqa: E402
from lawn_mower_planning.path_cost import PathCost  # noqa: E402
from lawn_mower_planning.problem import RIGHT_ANGLE, build_problem  # noqa: E402
from lawn_mower_planning.transitions import Plan  # noqa: E402
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config  # noqa: E402

ROBOT = 'pioneer3at'
CSV = os.path.join(style.DATA, 'ordering-pioneer3at-direction.csv')
SWEEP = tuple(math.radians(a) for a in SWEEP_ANGLES_DEG)
LAWN_ORDER = [lawn.name for lawn in shared_lawns()]


# ----------------------------------------------------------------------------- helpers

_cost = None


def path_cost():
    global _cost
    if _cost is None:
        _cost = PathCost.from_config(load_robot_config(resolve_robot_config(ROBOT)))
    return _cost


def rows():
    with open(CSV) as f:
        return list(csv.DictReader(f))


def problem_for(lawn, lane_angle_deg=90.0):
    return build_problem(lawn.area, DEFAULT_WIDTH, path_cost(), None,
                         math.radians(lane_angle_deg), lawn.start, SWEEP)


def plan_from_row(problem, row):
    order = tuple(int(c) for c in row['order'].split())
    variants = tuple(int(a) for a in row['variants'].split())
    return Plan(order, variants, float(row['cost']),
                problem.table.totals(order, variants))


def to_world(problem, geom):
    """Sweep-frame geometry back to the world frame (see CoverageProblem.path)."""
    return affinity.rotate(geom, -(RIGHT_ANGLE - problem.lane_angle), origin=(0, 0),
                           use_radians=True)


def points_to_world(problem, points):
    t = -(RIGHT_ANGLE - problem.lane_angle)
    c, s = math.cos(t), math.sin(t)
    return np.array([(c * x - s * y, s * x + c * y) for x, y in points])


def polys(geom):
    if geom.is_empty:
        return []
    if isinstance(geom, MultiPolygon):
        return list(geom.geoms)
    if geom.geom_type == 'Polygon':
        return [geom]
    return [g for g in getattr(geom, 'geoms', []) if g.geom_type == 'Polygon']


def draw_polygon(ax, geom, **kw):
    for p in polys(geom):
        verts, codes = [], []
        for ring in (p.exterior, *p.interiors):
            xy = list(ring.coords)
            verts += xy
            codes += [Path.MOVETO] + [Path.LINETO] * (len(xy) - 2) + [Path.CLOSEPOLY]
        ax.add_patch(PathPatch(Path(verts, codes), **kw))


def draw_lawn(ax, area, obstacles=True):
    draw_polygon(ax, area, facecolor=LAWN, edgecolor=LAWN_EDGE, linewidth=0.8, zorder=1)
    if obstacles:
        for hole in area.interiors:
            from shapely.geometry import Polygon
            draw_polygon(ax, Polygon(hole), facecolor=OBSTACLE, edgecolor='none',
                         hatch='////', alpha=0.55, zorder=2)


def draw_cells(ax, problem, labels=True, fill='#cfe3f8'):
    for i, cell in enumerate(problem.cells):
        w = to_world(problem, cell)
        draw_polygon(ax, w, facecolor=fill, edgecolor=BLUE, linewidth=0.6, zorder=3)
        if labels:
            from shapely.ops import polylabel
            p = polylabel(polys(w)[0], tolerance=0.05)
            ax.text(p.x, p.y, str(i + 1), ha='center', va='center', fontsize=6.5,
                    color=INK, zorder=6)


def draw_start(ax, lawn, size=1.0):
    x, y, th = lawn.start
    ax.plot([x], [y], marker=(3, 0, math.degrees(th) - 90), markersize=6, color=INK,
            zorder=8, linestyle='none')


def label_spots(regions, gap):
    """One point in each region for its label: far from the region's edge and from the
    labels already placed, so that neighbouring labels in narrow cells do not collide."""
    from shapely.geometry import Point
    from shapely.ops import polylabel
    regions = [max(polys(r), key=lambda g: g.area) for r in regions]
    placed = {}
    # Narrow cells have the fewest good spots, so they choose first.
    for i in sorted(range(len(regions)), key=lambda i: regions[i].area):
        region = regions[i]
        x0, y0, x1, y1 = region.bounds
        cands = [polylabel(region, tolerance=0.05)]
        cands += [Point(x, y) for x in np.linspace(x0, x1, 15) for y in np.linspace(y0, y1, 15)]
        best, score = None, -math.inf
        for c in cands:
            if not region.contains(c):
                continue
            edge = min(region.exterior.distance(c), 1.2)
            near = min((c.distance(q) for q in placed.values()), default=math.inf)
            sc = edge - 3.0 * max(0.0, gap - near)
            if sc > score:
                best, score = c, sc
        placed[i] = best
    return [placed[i] for i in range(len(regions))]


def draw_plan(ax, problem, plan, lane_colour=BLUE, link_colour=ORANGE, lw=0.55,
              number_cells=False):
    prev = (None, None)
    if number_cells:
        spots = label_spots([to_world(problem, problem.cells[c]) for c in plan.order], gap=3.6)
    for k, (c, a) in enumerate(zip(plan.order, plan.variants)):
        link = problem.links.get((*prev, c, a))
        if link:
            xy = points_to_world(problem, link)
            ax.plot(xy[:, 0], xy[:, 1], color=link_colour, lw=1.1, zorder=6,
                    solid_capstyle='round')
        xy = points_to_world(problem, problem.routes[c][a])
        ax.plot(xy[:, 0], xy[:, 1], color=lane_colour, lw=lw, zorder=5)
        if number_cells:
            p = spots[k]
            ax.text(p.x, p.y, str(k + 1), ha='center', va='center', fontsize=6.5,
                    color=INK, zorder=9,
                    bbox=dict(boxstyle='circle,pad=0.15', fc='white', ec=INK_2, lw=0.5))
        prev = (c, a)


def frame(ax, area, pad=0.8):
    x0, y0, x1, y1 = area.bounds
    ax.set_xlim(x0 - pad, x1 + pad)
    ax.set_ylim(y0 - pad, y1 + pad)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def scale_bar(ax, area, length=5.0):
    """Scale bar under the right end of the lawn, clear of the start marker."""
    _, y0, x1, _ = area.bounds
    ax.plot([x1 - length, x1], [y0 - 0.55, y0 - 0.55], color=INK, lw=1.2,
            solid_capstyle='butt', clip_on=False)
    ax.text(x1 - length / 2, y0 - 0.75, f'{length:g} m', ha='center', va='top', fontsize=6.5)


def legend_below(fig, axes, handles, ncol, gap=0.02):
    """Legend just under the drawn (aspect-adjusted) axes."""
    for ax in axes:
        ax.apply_aspect()
    bottom = min(ax.get_position(original=False).y0 for ax in axes)
    fig.legend(handles=handles, loc='upper center', ncol=ncol,
               bbox_to_anchor=(0.5, bottom - gap))


def rel(a, b):
    return 100 * (a / b - 1)


# ----------------------------------------------------------------------------- figures

def fig_pipeline(preview):
    """Free area, BCD cells, lanes and the assembled plan on one lawn."""
    lawn = lawn_by_name('garden')
    problem = problem_for(lawn)
    row = next(r for r in rows() if r['map'] == lawn.name and r['method'] == 'energy_ga'
               and r['seed'] == '0')
    plan = plan_from_row(problem, row)
    fig, axes = plt.subplots(1, 4, figsize=(FULL, 1.95))
    titles = ['Lawn and free area', 'Boustrophedon cells', 'Lanes of each cell',
              'Order and transitions']
    for k, ax in enumerate(axes):
        draw_lawn(ax, lawn.area)
        frame(ax, lawn.area)
        ax.set_title(f'({chr(97 + k)}) {titles[k]}', pad=3)
    free = to_world(problem, problem.free)
    for p in polys(free):
        for ring in (p.exterior, *p.interiors):
            xy = np.array(ring.coords)
            axes[0].plot(xy[:, 0], xy[:, 1], color=BLUE, lw=0.7, ls=(0, (3, 1.5)), zorder=4)
    draw_start(axes[0], lawn)
    scale_bar(axes[0], lawn.area)
    draw_cells(axes[1], problem)
    draw_cells(axes[2], problem, labels=False, fill='none')
    for c, a in zip(plan.order, plan.variants):
        xy = points_to_world(problem, problem.routes[c][a])
        axes[2].plot(xy[:, 0], xy[:, 1], color=BLUE, lw=0.45, zorder=5)
    draw_plan(axes[3], problem, plan, number_cells=True)
    draw_start(axes[3], lawn)
    handles = [Line2D([], [], color=BLUE, lw=0.7, ls=(0, (3, 1.5)), label='Free-area boundary'),
               Line2D([], [], color=BLUE, lw=0.8, label='Lanes'),
               Line2D([], [], color=ORANGE, lw=1.2, label='Start link and transitions'),
               Line2D([], [], color=INK, marker=(3, 0, -90), ls='none', ms=5, label='Robot start'),
               PathPatch(Path([(0, 0)]), fc=OBSTACLE, alpha=0.55, hatch='////', ec='none',
                         label='Obstacle')]
    fig.subplots_adjust(wspace=0.05)
    legend_below(fig, axes, handles, 5, gap=0.06)
    save(fig, 'pipeline', preview)


def fig_lawns(preview):
    """The ten test lawns with their BCD cells (default direction)."""
    lawns = shared_lawns()
    fig, axes = plt.subplots(2, 5, figsize=(FULL, 2.6))
    # One scale for all panels, so the sizes of the lawns compare at a glance.
    span_x = max(lw.area.bounds[2] - lw.area.bounds[0] for lw in lawns) + 1.0
    span_y = max(lw.area.bounds[3] - lw.area.bounds[1] for lw in lawns) + 1.0
    for ax, lawn in zip(axes.flat, lawns):
        problem = problem_for(lawn)
        draw_lawn(ax, lawn.area)
        draw_cells(ax, problem, labels=False)
        draw_start(ax, lawn)
        frame(ax, lawn.area, pad=0.5)
        x0, y0, x1, y1 = lawn.area.bounds
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        ax.set_xlim(cx - span_x / 2, cx + span_x / 2)
        ax.set_ylim(cy - span_y / 2, cy + span_y / 2)
        name = lawn.name.replace('_', r'\_')
        ax.set_title(f'$\\mathtt{{{name}}}$\n{len(problem.cells)} cells, '
                     f'{lawn.area.area:.0f} m$^2$', pad=2, fontsize=7)
    x0, y0, _, _ = lawns[0].area.bounds
    axes.flat[0].plot([x0, x0 + 10], [y0 - 1.5, y0 - 1.5], color=INK, lw=1.0,
                      clip_on=False, solid_capstyle='butt')
    axes.flat[0].text(x0 + 5, y0 - 2.2, '10 m (all panels)', ha='left', va='top', fontsize=6.5)
    fig.subplots_adjust(wspace=0.12, hspace=0.35)
    save(fig, 'test-lawns', preview)


def fig_energy_model(preview):
    """Energy of a straight drive and of a turn in place, and the turn share of a plan."""
    cost = path_cost()
    m = cost.energy_model
    lengths = np.linspace(0.05, 20, 200)
    straight = np.array([m.straight(d, cost.speed).electrical for d in lengths])
    angles = np.linspace(1, 180, 180)
    turn = np.array([m.turn(math.radians(a), cost.turn_rate).electrical for a in angles])
    per_metre = (m.straight(20, cost.speed).electrical - m.straight(10, cost.speed).electrical) / 10
    e90 = m.turn(math.pi / 2, cost.turn_rate).electrical
    e90_metres = e90 / per_metre

    # Share of the plan energy spent in turns, for the default and the chosen direction.
    data = rows()
    share = {}
    for method in ('xie2024', 'energy_direction_ga'):
        share[method] = []
        for name in LAWN_ORDER:
            row = next(r for r in data if r['map'] == name and r['method'] == method
                       and r['seed'] == '0')
            problem = _problem_cache(name, float(row['lane_angle']))
            pts = problem.path(plan_from_row(problem, row))
            ls, ts = cost.segments(pts)
            es = sum(m.straight(d, cost.speed).electrical for d in ls)
            et = sum(m.turn(a, cost.turn_rate).electrical for a in ts)
            share[method].append(100 * et / (es + et))

    fig, axes = plt.subplots(1, 3, figsize=(FULL, 1.95),
                             gridspec_kw={'width_ratios': [1, 1, 1.35]})
    ax = axes[0]
    ax.plot(lengths, straight, color=BLUE)
    ax.set_xlabel('Straight length $\\ell$ (m)')
    ax.set_ylabel('Battery energy (J)')
    ax.set_title('$E_\\mathrm{s}(\\ell)$, from rest', pad=3)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, None)
    ax.annotate(f'{per_metre:.1f} J/m', xy=(14, np.interp(14, lengths, straight)),
                xytext=(6, -14), textcoords='offset points', fontsize=7, color=INK_2)
    panel_label(axes[0], '(a)', x=-0.2)

    ax = axes[1]
    ax.plot(angles, turn, color=ORANGE)
    ax.set_xlabel('Turn angle $\\varphi$ (deg)')
    ax.set_ylabel('Battery energy (J)')
    ax.set_xticks([0, 45, 90, 135, 180])
    ax.set_xlim(0, 180)
    ax.set_ylim(0, 52)
    ax.set_title('$E_\\mathrm{r}(\\varphi)$, in place', pad=3)
    ax.plot([90], [e90], 'o', color=ORANGE, ms=4, mec='white', mew=0.8)
    ax.annotate(f'90$^\\circ$: {e90:.0f} J\n= {e90_metres:.1f} m straight', xy=(90, e90),
                xytext=(12, 38), textcoords='data', fontsize=7, color=INK_2,
                arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.5, shrinkA=2, shrinkB=3))
    panel_label(ax, '(b)', x=-0.2)

    ax = axes[2]
    y = np.arange(len(LAWN_ORDER))
    ax.barh(y - 0.2, share['energy_direction_ga'], height=0.38, color=BLUE, label='Ours')
    ax.barh(y + 0.2, share['xie2024'], height=0.38, color=ORANGE, label='Xie et al.')
    ax.set_yticks(y)
    ax.set_yticklabels(LAWN_ORDER, fontsize=6.5,
                       fontfamily='monospace')
    ax.invert_yaxis()
    ax.set_xlabel('Energy spent in turns (%)')
    ax.grid(True, axis='x')
    ax.set_axisbelow(True)
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 0.99), ncol=2, handlelength=1,
              borderaxespad=0)
    panel_label(ax, '(c)', x=-0.32, y=1.02)
    fig.subplots_adjust(wspace=0.55)
    save(fig, 'energy-model', preview)
    print(f'  90 deg turn = {e90:.1f} J = {e90_metres:.2f} m straight; '
          f'turn share xie {statistics.mean(share["xie2024"]):.1f} %, '
          f'ours {statistics.mean(share["energy_direction_ga"]):.1f} %')


_problems = {}


def _problem_cache(name, lane_angle):
    key = (name, round(lane_angle, 3))
    if key not in _problems:
        _problems[key] = problem_for(lawn_by_name(name), lane_angle)
    return _problems[key]


def fig_direction(preview, name='garden'):
    """Screening energy of the candidate directions, and the default vs chosen cells."""
    lawn = lawn_by_name(name)
    default = _problem_cache(name, 90.0)
    choice = choose_direction(lawn.area, DEFAULT_WIDTH, path_cost(), lawn.start, SWEEP,
                              default=default)
    chosen = choice.problem
    scores = sorted((math.degrees(a), e) for a, e in choice.scores if math.isfinite(e))
    best_angle = math.degrees(choice.lane_angle)
    e_default = next(e for a, e in scores if abs(a - 90) < 1e-6)
    e_best = next(e for a, e in scores if abs(a - best_angle) < 1e-6)
    fig = plt.figure(figsize=(FULL, 2.15))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.05], wspace=0.3)
    for k, (problem, angle, energy, label) in enumerate([
            (default, 90.0, e_default, 'Default'),
            (chosen, best_angle, e_best, 'Chosen')]):
        ax = fig.add_subplot(gs[k])
        draw_lawn(ax, lawn.area)
        best = _screen_plan(problem)
        for c, a in zip(best.order, best.variants):
            xy = points_to_world(problem, problem.routes[c][a])
            ax.plot(xy[:, 0], xy[:, 1], color=BLUE, lw=0.35, alpha=0.7, zorder=4)
        # Cell borders on top of the lanes, so the decomposition stays readable.
        for cell in problem.cells:
            draw_polygon(ax, to_world(problem, cell), facecolor='none', edgecolor=INK,
                         linewidth=0.8, zorder=5)
        frame(ax, lawn.area)
        ax.set_title(f'({chr(97 + k)}) {label}: lanes at {angle:.0f}°\n'
                     f'{len(problem.cells)} cells, {energy / 1000:.2f} kJ', pad=3)
    ax = fig.add_subplot(gs[2])
    xs = np.arange(len(scores))
    colours = [BLUE if abs(a - best_angle) < 1e-6 else (ORANGE if abs(a - 90) < 1e-6 else MUTED)
               for a, _ in scores]
    # Energy above the chosen direction, from zero, so the bar lengths compare honestly.
    above = [rel(e, e_best) for _, e in scores]
    ax.bar(xs, above, color=colours, width=0.6)
    # The chosen candidate is the zero of the axis: mark it, since its bar has no height.
    k_best = next(i for i, (a, _) in enumerate(scores) if abs(a - best_angle) < 1e-6)
    ax.plot([k_best], [0], marker='D', ms=4.5, color=BLUE, ls='none', clip_on=False, zorder=5)
    for x, v, (a, _) in zip(xs, above, scores):
        if abs(a - 90) < 1e-6:
            ax.text(x, v + 0.3, f'+{v:.1f} %', ha='center', va='bottom', fontsize=6.5,
                    color=INK)
    ax.set_xticks(xs)
    ax.set_xticklabels([f'{a:.0f}°' for a, _ in scores], fontsize=6.5, rotation=90)
    ax.set_ylim(0, max(above) * 1.7)
    ax.set_ylabel('Energy above the chosen (%)', labelpad=2)
    ax.set_xlabel('Candidate lane heading')
    ax.legend(handles=[PathPatch(Path([(0, 0)]), fc=ORANGE, label='Default (90°)'),
                       PathPatch(Path([(0, 0)]), fc=BLUE, label='Chosen'),
                       PathPatch(Path([(0, 0)]), fc=MUTED, label='Other candidates')],
              loc='upper right', handlelength=1, borderaxespad=0.2)
    ax.set_title('(c) Screening energy of each candidate', pad=3)
    for lab, (a, _) in zip(ax.get_xticklabels(), scores):
        if abs(a - best_angle) < 1e-6:
            lab.set_color(BLUE)
        elif abs(a - 90) < 1e-6:
            lab.set_color(ORANGE)
    save(fig, 'direction-choice', preview)
    print(f'  default {e_default / 1000:.2f} kJ, chosen {best_angle:.0f} deg '
          f'{e_best / 1000:.2f} kJ ({rel(e_best, e_default):.1f} %)')


def _screen_plan(problem):
    return energy_ga_order(problem.table, EnergyGaParams(evaluations=500), 0).plan


def fig_plans(preview, name='three_beds', seed='0'):
    """The plans of Xie et al. and ours on one lawn."""
    lawn = lawn_by_name(name)
    data = rows()
    picks = [('xie2024', 'Xie et al.'), ('xie2024_direction', 'Xie et al., our direction'),
             ('energy_direction_ga', 'Ours')]
    fig, axes = plt.subplots(1, 3, figsize=(FULL, 2.25))
    for k, (ax, (method, label)) in enumerate(zip(axes, picks)):
        row = next(r for r in data if r['map'] == name and r['method'] == method
                   and r['seed'] == seed)
        problem = _problem_cache(name, float(row['lane_angle']))
        plan = plan_from_row(problem, row)
        assert abs(plan.metrics['energy'] - float(row['energy'])) < 1e-6 * float(row['energy'])
        draw_lawn(ax, lawn.area)
        draw_cells(ax, problem, labels=False, fill='none')
        draw_plan(ax, problem, plan, number_cells=True)
        draw_start(ax, lawn)
        frame(ax, lawn.area)
        ax.set_title(f'({chr(97 + k)}) {label}\n{float(row["energy"]) / 1000:.2f} kJ, '
                     f'{float(row["length"]):.0f} m, {float(row["turns"]):.0f} turns', pad=3)
    handles = [Line2D([], [], color=BLUE, lw=0.8, label='Lanes'),
               Line2D([], [], color=ORANGE, lw=1.2, label='Start link and transitions'),
               Line2D([], [], color=INK, marker='o', mfc='white', ls='none', ms=5,
                      label='Visiting order'),
               Line2D([], [], color=INK, marker=(3, 0, -90), ls='none', ms=5, label='Robot start')]
    fig.subplots_adjust(wspace=0.06)
    legend_below(fig, axes, handles, 4)
    save(fig, 'example-plans', preview)


def _paired(data, a, b, metric='energy'):
    """Per lawn: list of paired relative differences 100 (a / b - 1) over the seeds."""
    index = {(r['map'], r['method'], r['seed']): float(r[metric]) for r in data}
    out = {}
    for name in LAWN_ORDER:
        out[name] = [rel(index[(name, a, s)], index[(name, b, s)])
                     for s in map(str, range(10))
                     if (name, a, s) in index and (name, b, s) in index]
    return out


def fig_energy_results(preview):
    """Paired energy differences per lawn, and the split into direction and ordering."""
    data = rows()
    full = _paired(data, 'energy_direction_ga', 'xie2024')
    same = _paired(data, 'energy_direction_ga', 'xie2024_direction')
    fig, axes = plt.subplots(1, 2, figsize=(FULL, 2.6), sharey=True,
                             gridspec_kw={'width_ratios': [1.25, 1]})
    y = np.arange(len(LAWN_ORDER))
    ax = axes[0]
    rng = np.random.default_rng(0)
    for off, d, colour, label in [(-0.17, full, BLUE, 'vs. Xie et al.'),
                                  (0.17, same, AQUA, 'vs. Xie et al., same direction')]:
        for i, name in enumerate(LAWN_ORDER):
            vals = d[name]
            jit = rng.uniform(-0.06, 0.06, len(vals))
            ax.scatter(vals, y[i] + off + jit, s=9, color=colour, alpha=0.55, lw=0,
                       zorder=3)
            ax.plot([statistics.median(vals)] * 2, [y[i] + off - 0.14, y[i] + off + 0.14],
                    color=INK, lw=1.2, zorder=4)
        ax.scatter([], [], s=12, color=colour, label=label)
    ax.axvline(0, color=INK_2, lw=0.6)
    ax.set_yticks(y)
    ax.set_yticklabels([n for n in LAWN_ORDER], fontsize=6.5, fontfamily='monospace')
    ax.invert_yaxis()
    ax.set_xlabel('Energy difference of ours (%)')
    ax.grid(True, axis='x')
    ax.set_axisbelow(True)
    ax.set_title('Paired by seed (dots: 10 seeds, tick: median)', pad=3)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=2, handletextpad=0.3)
    panel_label(ax, '(a)', x=-0.2)

    ax = axes[1]
    # Both parts in % of the energy of Xie et al., so they add up to the total of (a).
    index = {(r['map'], r['method'], r['seed']): float(r['energy']) for r in data}
    dmean, omean, tmean = [], [], []
    for n in LAWN_ORDER:
        seeds = [s for s in map(str, range(10)) if (n, 'xie2024', s) in index]
        base = [index[(n, 'xie2024', s)] for s in seeds]
        mid = [index[(n, 'xie2024_direction', s)] for s in seeds]
        ours = [index[(n, 'energy_direction_ga', s)] for s in seeds]
        dmean.append(statistics.mean(100 * (m - b) / b for m, b in zip(mid, base)))
        omean.append(statistics.mean(100 * (o - m) / b for o, m, b in zip(ours, mid, base)))
        tmean.append(statistics.mean(100 * (o - b) / b for o, b in zip(ours, base)))
    ax.barh(y, dmean, height=0.55, color=YELLOW, label='Direction choice')
    ax.barh(y, omean, left=dmean, height=0.55, color=AQUA, label='Energy ordering')
    ax.plot(tmean, y, marker='|', ms=9, mew=1.2, color=INK, ls='none', label='Total')
    ax.axvline(0, color=INK_2, lw=0.6)
    ax.set_xlabel('Mean energy difference to Xie et al. (%)')
    ax.grid(True, axis='x')
    ax.set_axisbelow(True)
    ax.set_title('Split of the gain', pad=3)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=3, handlelength=1)
    panel_label(ax, '(b)', x=-0.04)
    fig.subplots_adjust(wspace=0.08)
    save(fig, 'energy-results', preview)
    print('  mean full {:.2f} %, same direction {:.2f} %, direction alone {:.2f} %'.format(
        statistics.mean(statistics.mean(v) for v in full.values()),
        statistics.mean(statistics.mean(v) for v in same.values()),
        statistics.mean(dmean)))


def fig_convergence(preview, seeds=10, budget=20000):
    """Gap of the best plan to the exact energy optimum during the search."""
    data = rows()
    optimum = {r['map']: float(r['energy']) for r in data if r['method'] == 'energy_optimal'}
    names = [n for n in LAWN_ORDER if n in optimum]
    grid = np.unique(np.round(np.logspace(0, math.log10(budget), 200)).astype(int))
    fig, axes = plt.subplots(1, len(names), figsize=(FULL, 1.85), sharey=True)
    for ax, name in zip(np.atleast_1d(axes), names):
        problem = _problem_cache(name, 90.0)
        for method, run, params, colour, label in [
                ('GA', energy_ga_order, EnergyGaParams(evaluations=budget), BLUE, 'GA'),
                ('PSO', energy_pso_order, EnergyPsoParams(evaluations=budget), AQUA, 'PSO')]:
            curves = []
            for seed in range(seeds):
                hist = run(problem.table, params, seed).history
                ev = np.array([h[0] for h in hist])
                cost = np.array([h[1] for h in hist])
                idx = np.searchsorted(ev, grid, side='right') - 1
                vals = np.where(idx >= 0, cost[np.clip(idx, 0, None)], np.nan)
                curves.append(rel(vals, optimum[name]))
            curves = np.array(curves)
            med = np.nanmedian(curves, axis=0)
            lo, hi = np.nanpercentile(curves, 25, axis=0), np.nanpercentile(curves, 75, axis=0)
            ax.fill_between(grid, np.maximum(lo, 1e-3), np.maximum(hi, 1e-3), color=colour,
                            alpha=0.18, lw=0)
            ax.plot(grid, np.maximum(med, 1e-3), color=colour, label=label)
            hit = np.argmax(curves < 1e-9, axis=1)
            reached = (curves < 1e-9).any(axis=1)
            print(f'  {name} {method}: optimum in {reached.sum()}/{seeds} seeds, median at '
                  f'{np.median(grid[hit[reached]]) if reached.any() else float("nan"):.0f} evals')
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_ylim(1e-3, 30)
        ax.set_xlim(1, budget)
        ax.set_xticks([1, 10, 100, 1000, 10000])
        ax.set_yticks([1e-3, 1e-2, 0.1, 1, 10])
        ax.set_yticklabels(['opt.', '0.01', '0.1', '1', '10'])
        ax.set_title(f'$\\mathtt{{{name.replace("_", chr(92) + "_")}}}$, '
                     f'{problem.table.n_cells} cells', pad=3)
        ax.set_xlabel('Evaluations')
        ax.grid(True, which='major')
    np.atleast_1d(axes)[0].set_ylabel('Gap to optimum (%)')
    handles, labels = np.atleast_1d(axes)[0].get_legend_handles_labels()
    fig.subplots_adjust(wspace=0.08, top=0.8)
    from matplotlib.patches import Patch
    handles.append(Patch(fc=MUTED, alpha=0.35, lw=0))
    labels = [f'{l}, median of {seeds} seeds' for l in labels] + ['25–75 % of seeds']
    fig.legend(handles, labels, loc='lower center', ncol=3, bbox_to_anchor=(0.5, 0.9),
               handlelength=1.5)
    save(fig, 'convergence', preview)


def fig_fuzzy(preview):
    """Membership functions and control surface of the fuzzy steering controller."""
    from lawn_mower_navigation import tracker
    from lawn_mower_navigation.fuzzy import FuzzyTable, memberships
    labels = ['NB', 'NS', 'Z', 'PS', 'PB']
    fig = plt.figure(figsize=(FULL, 2.0))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.15], wspace=0.42)
    for k, (centres, xlabel, scale) in enumerate([
            (tracker.CROSS_TRACK_CENTRES, 'Cross-track error $e_y$ (m)\n(+: left of the lane)', 1.0),
            (tracker.HEADING_CENTRES, 'Heading error $e_\\psi$ (deg)\n(+: pointing left)', 180 / math.pi)]):
        ax = fig.add_subplot(gs[k])
        span = centres[-1] - centres[0]
        xs = np.linspace(centres[0] - 0.25 * span, centres[-1] + 0.25 * span, 400)
        mu = np.array([memberships(x, centres) for x in xs])
        for j in range(len(centres)):
            ax.plot(xs * scale, mu[:, j], color=[ORANGE, YELLOW, MUTED, AQUA, BLUE][j], lw=1.1)
            ax.text(centres[j] * scale, 1.05, labels[j], ha='center', va='bottom', fontsize=6.5)
        ax.set_ylim(0, 1.18)
        ax.set_yticks([0, 0.5, 1])
        ax.set_xlabel(xlabel)
        ax.set_xticks([-0.3, 0, 0.3] if scale == 1.0 else [-20, 0, 20])
        if k == 0:
            ax.set_ylabel('Membership $\\mu$')
        panel_label(ax, f'({chr(97 + k)})', x=-0.12, y=1.08)
    table = FuzzyTable(tracker.CROSS_TRACK_CENTRES, tracker.HEADING_CENTRES,
                       tracker.steering_rules())
    ey = np.linspace(-0.4, 0.4, 161)
    ep = np.linspace(-0.5, 0.5, 161)
    z = np.array([[table(a, b) for a in ey] for b in ep])
    ax = fig.add_subplot(gs[2])
    cmap = LinearSegmentedColormap.from_list('div', [ORANGE, '#f2f1ec', BLUE])
    im = ax.pcolormesh(ey, np.degrees(ep), z, cmap=cmap, vmin=-0.8, vmax=0.8,
                       shading='auto', rasterized=True)
    ax.contour(ey, np.degrees(ep), z, levels=[-0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6],
               colors=INK_2, linewidths=0.4)
    ax.set_xlabel('Cross-track error $e_y$ (m)')
    ax.set_ylabel('Heading error $e_\\psi$ (deg)')
    cb = fig.colorbar(im, ax=ax, fraction=0.06, pad=0.03)
    cb.set_label('Steering command\n(share of turn-rate limit)', fontsize=7)
    cb.set_ticks([-0.8, -0.4, 0, 0.4, 0.8])
    cb.set_ticklabels(['−0.8 right', '−0.4', '0', '0.4', '0.8 left'])
    cb.outline.set_linewidth(0.4)
    cb.ax.tick_params(labelsize=6.5)
    panel_label(ax, '(c)', x=-0.2, y=1.02)
    save(fig, 'fuzzy-controller', preview)


FIGURES = {
    'pipeline': fig_pipeline,
    'test-lawns': fig_lawns,
    'energy-model': fig_energy_model,
    'direction-choice': fig_direction,
    'example-plans': fig_plans,
    'energy-results': fig_energy_results,
    'convergence': fig_convergence,
    'fuzzy-controller': fig_fuzzy,
}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--only', nargs='*', choices=sorted(FIGURES), default=None)
    parser.add_argument('--preview', help='also write PNG previews to this directory')
    args = parser.parse_args(argv)
    style.setup()
    for name, make in FIGURES.items():
        if args.only is None or name in args.only:
            make(args.preview)


if __name__ == '__main__':
    main()
