"""Shared look of the paper figures: fonts, sizes, colours and saving."""

import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PAPER = os.path.dirname(HERE)
FIGURES = os.path.join(PAPER, 'figures')
DATA = os.path.join(PAPER, 'data')

# Text width of the cas-sc single-column class is about 6.5 in.
FULL = 6.5
HALF = 3.2

# Categorical slots, always in this order (checked for colour-vision deficiency).
BLUE = '#2a78d6'    # ours
ORANGE = '#eb6834'  # Xie et al.
AQUA = '#1baf7a'    # second variant of ours (PSO) / ablation
YELLOW = '#eda100'

INK = '#1f1f1e'
INK_2 = '#5f5e58'
MUTED = '#9b9a93'
GRID = '#e3e2dc'
LAWN = '#e6f2e2'
LAWN_EDGE = '#5a8f4e'
OBSTACLE = '#8a8780'
CELL_EDGE = '#ffffff'
TRANSITION = ORANGE

METHOD_COLOUR = {
    'energy_direction_ga': BLUE,
    'energy_ga': BLUE,
    'energy_pso': AQUA,
    'xie2024': ORANGE,
    'xie2024_direction': YELLOW,
}


def setup():
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['STIXGeneral', 'DejaVu Serif'],
        'mathtext.fontset': 'stix',
        'font.size': 8,
        'axes.titlesize': 8,
        'axes.labelsize': 8,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'legend.fontsize': 7,
        'axes.edgecolor': INK_2,
        'axes.labelcolor': INK,
        'xtick.color': INK_2,
        'ytick.color': INK_2,
        'text.color': INK,
        'axes.linewidth': 0.6,
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'axes.grid': False,
        'grid.color': GRID,
        'grid.linewidth': 0.5,
        'legend.frameon': False,
        'lines.linewidth': 1.2,
        'pdf.fonttype': 42,
        'ps.fonttype': 42,
        'savefig.dpi': 300,
    })


def _segments(points, step):
    """Points every `step` pixels along a polyline (display coordinates)."""
    import numpy as np
    out = []
    for p, q in zip(points, points[1:]):
        n = max(2, int(np.hypot(*(q - p)) / step) + 1)
        out.append(p + (q - p) * np.linspace(0, 1, n)[:, None])
    return np.vstack(out) if out else np.empty((0, 2))


def _clip(artist, pts):
    """Drop the points that the artist's clip box hides."""
    box = artist.get_clip_box() if artist.get_clip_on() else None
    if box is None or not len(pts):
        return pts
    keep = ((pts[:, 0] >= box.x0 - 0.5) & (pts[:, 0] <= box.x1 + 0.5)
            & (pts[:, 1] >= box.y0 - 0.5) & (pts[:, 1] <= box.y1 + 0.5))
    return pts[keep]


def overlaps(fig, pad=0.6):
    """Texts that touch another text, a line or the outline of a patch.

    Text inside a box is fine; text on the box edge, on a wire or on another
    label is not. Texts drawn with their own background box (the cell numbers
    on the maps) are meant to sit on the drawing and are skipped.
    Returns a list of messages.
    """
    import numpy as np
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    from matplotlib.text import Text
    fig.canvas.draw()
    r = fig.canvas.get_renderer()
    # Tick labels outside the view limits keep their Text objects but are not drawn.
    hidden = set()
    for ax in fig.axes:
        for axis in (ax.xaxis, ax.yaxis):
            shown = ax.axison and axis.get_visible()
            drawn = set(map(id, axis._update_ticks())) if shown else set()
            for tick in axis.get_major_ticks() + axis.get_minor_ticks():
                if id(tick) not in drawn:
                    hidden.update((id(tick.label1), id(tick.label2)))
    texts = [t for t in fig.findobj(Text)
             if t.get_visible() and t.get_text().strip() and t.get_bbox_patch() is None
             and t.figure is fig and id(t) not in hidden]
    # An annotation's extent includes its arrow; measure the text alone.
    boxes = [Text.get_window_extent(t, r).expanded(1, 1) for t in texts]
    boxes = [b.from_extents(b.x0 - pad, b.y0 - pad, b.x1 + pad, b.y1 + pad) for b in boxes]
    issues = []
    for i in range(len(texts)):
        for j in range(i + 1, len(texts)):
            if boxes[i].overlaps(boxes[j]):
                issues.append(f'text {texts[i].get_text()!r} overlaps text {texts[j].get_text()!r}')
    shapes = []
    for ln in fig.findobj(Line2D):
        if not ln.get_visible() or ln.get_linewidth() == 0 or len(ln.get_xydata()) < 2:
            continue
        if ln.get_label() == '_overlap_ok':
            continue
        pts = ln.get_transform().transform(ln.get_xydata())
        pts = pts[np.isfinite(pts).all(axis=1)]
        shapes.append(('line', _clip(ln, _segments(pts, 1.0))))
        if ln.get_marker() not in (None, 'None', '', ' ') and ln.get_linestyle() in ('None', ''):
            shapes.pop()
    for pa in fig.findobj(Patch):
        if (not pa.get_visible() or pa.get_label() == '_overlap_ok' or pa is fig.patch
                or pa.get_edgecolor()[3] == 0 or pa.get_linewidth() == 0):
            continue
        if any(pa is t.get_bbox_patch() for t in texts):
            continue
        if pa.axes is not None and (pa is pa.axes.patch or (
                type(pa).__name__ == 'Spine'
                and not (pa.axes.axison and pa.axes.get_frame_on()))):
            continue
        for poly in pa.get_path().to_polygons(closed_only=False):
            pts = pa.get_transform().transform(poly)
            shapes.append(('outline', _clip(pa, _segments(pts, 1.0))))
    for t, b in zip(texts, boxes):
        for kind, pts in shapes:
            if len(pts) and ((pts[:, 0] > b.x0) & (pts[:, 0] < b.x1)
                             & (pts[:, 1] > b.y0) & (pts[:, 1] < b.y1)).any():
                issues.append(f'text {t.get_text()!r} touches a {kind}')
                break
    return issues


def save(fig, name, preview_dir=None, check=True):
    """Write figures/<name>.pdf, and a PNG preview to preview_dir if given.

    With check, report every text that touches another text or a drawn line.
    """
    if check:
        for issue in overlaps(fig):
            print(f'  OVERLAP {name}: {issue}')
    os.makedirs(FIGURES, exist_ok=True)
    path = os.path.join(FIGURES, f'{name}.pdf')
    fig.savefig(path, bbox_inches='tight', pad_inches=0.02, metadata={'CreationDate': None})
    if preview_dir:
        os.makedirs(preview_dir, exist_ok=True)
        fig.savefig(os.path.join(preview_dir, f'{name}.png'), bbox_inches='tight',
                    pad_inches=0.02, dpi=200)
    plt.close(fig)
    print('wrote', os.path.relpath(path, PAPER))


def panel_label(ax, text, x=-0.02, y=1.02):
    ax.text(x, y, text, transform=ax.transAxes, ha='right', va='bottom',
            fontsize=8, fontweight='bold')
