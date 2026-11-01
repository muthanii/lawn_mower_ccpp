"""Boustrophedon cell decomposition (BCD, Choset 2000) of a polygon with holes.

The sweep line is vertical and moves along x; rotate the polygon first for
another sweep direction. Every vertex x cuts the area into slabs, and each
slab into pieces (its connected parts). A piece joins its right neighbour
when they touch one-to-one; a split or merge of the sweep line's connectivity
(an obstacle starting or ending) starts a new cell. So each cell is the
union of a chain of pieces, and every vertical line meets it in one segment.
"""

from shapely.geometry import MultiPolygon, Polygon, box
from shapely.ops import unary_union

MIN_AREA = 1e-9    # m^2, smaller pieces are slivers from rounding
MIN_CONTACT = 1e-9  # m, pieces that share less of a slab edge do not touch


def polygons(geom):
    if isinstance(geom, Polygon):
        return [geom] if geom.area > MIN_AREA else []
    if isinstance(geom, MultiPolygon) or hasattr(geom, 'geoms'):
        return [p for g in geom.geoms for p in polygons(g)]
    return []


def boustrophedon_cells(area):
    """Split area (Polygon or MultiPolygon) into BCD cells, ordered by min x."""
    xs = sorted({round(x, 9) for p in polygons(area) for ring in [p.exterior, *p.interiors]
                 for x, _ in ring.coords})
    _, ymin, _, ymax = area.bounds
    slabs = [polygons(area.intersection(box(x0, ymin - 1, x1, ymax + 1)))
             for x0, x1 in zip(xs, xs[1:])]

    pieces = [p for slab in slabs for p in slab]
    first = [0]  # index of each slab's first piece in pieces
    for slab in slabs:
        first.append(first[-1] + len(slab))
    right = [[] for _ in pieces]
    left = [[] for _ in pieces]
    for s in range(len(slabs) - 1):
        for a in range(first[s], first[s + 1]):
            for b in range(first[s + 1], first[s + 2]):
                if pieces[a].intersection(pieces[b]).length > MIN_CONTACT:
                    right[a].append(b)
                    left[b].append(a)

    parent = list(range(len(pieces)))

    def root(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k

    for a, nbrs in enumerate(right):
        if len(nbrs) == 1 and len(left[nbrs[0]]) == 1:
            parent[root(nbrs[0])] = root(a)

    groups = {}
    for k, p in enumerate(pieces):
        groups.setdefault(root(k), []).append(p)
    cells = [unary_union(g) for g in groups.values()]
    broken = [c for c in cells if not isinstance(c, Polygon)]
    if broken:
        raise ValueError(f'{len(broken)} cells are not connected; check the polygon for slivers')
    return sorted(cells, key=lambda c: (c.bounds[0], c.bounds[1]))
