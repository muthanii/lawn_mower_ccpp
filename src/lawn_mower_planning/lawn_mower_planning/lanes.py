"""Back-and-forth lanes in one BCD cell, in the four start-corner variants.

Lanes are vertical (along the sweep line). A cell w wide gets ceil(w / width)
lanes, spread evenly so that the outer lanes are at most width / 2 from the
cell sides. The robot turns from one lane to the next along a straight line,
or along the cell boundary where the straight line would leave the cell.
"""

import math

from shapely.geometry import LineString

from .decomposition import MIN_CONTACT

# Variant index -> (start on the left side, first lane goes up)
VARIANTS = ((True, True), (True, False), (False, True), (False, False))
INSIDE_TOLERANCE = 1e-7  # m


def lane_segments(cell, width):
    """Vertical lanes as [(x, y_low, y_high)], left to right."""
    xmin, ymin, xmax, ymax = cell.bounds
    count = max(1, math.ceil((xmax - xmin) / width - MIN_CONTACT))
    out = []
    for i in range(count):
        x = xmin + (i + 0.5) * (xmax - xmin) / count
        cut = cell.intersection(LineString([(x, ymin - 1), (x, ymax + 1)]))
        parts = [g for g in getattr(cut, 'geoms', [cut]) if g.length > MIN_CONTACT]
        if parts:
            ys = [y for _, y in max(parts, key=lambda g: g.length).coords]
            out.append((x, min(ys), max(ys)))
    return out


def _side_chain(cell, x0, x1, top):
    """Cell boundary vertices strictly between x0 and x1 on the top or bottom side."""
    lo, hi = sorted((x0, x1))
    _, ymin, _, ymax = cell.bounds
    out = []
    for x, y in cell.exterior.coords[:-1]:
        if lo + MIN_CONTACT < x < hi - MIN_CONTACT:
            cut = cell.intersection(LineString([(x, ymin - 1), (x, ymax + 1)]))
            ys = [v for g in getattr(cut, 'geoms', [cut]) for _, v in g.coords]
            if ys and abs(y - (max(ys) if top else min(ys))) < INSIDE_TOLERANCE:
                out.append((x, y))
    return sorted(out, reverse=x0 > x1)


def cell_route(cell, width, variant):
    """Waypoints that cover the cell in the given variant (index into VARIANTS)."""
    from_left, up = VARIANTS[variant]
    lanes = lane_segments(cell, width)
    if not from_left:
        lanes = lanes[::-1]
    inside = cell.buffer(INSIDE_TOLERANCE)
    points = []
    for x, lo, hi in lanes:
        start, end = ((x, lo), (x, hi)) if up else ((x, hi), (x, lo))
        if points and not inside.covers(LineString([points[-1], start])):
            points.extend(_side_chain(cell, points[-1][0], x, top=not up))
        points.extend([start, end])
        up = not up
    return points


def cell_routes(cell, width):
    return [cell_route(cell, width, v) for v in range(len(VARIANTS))]
