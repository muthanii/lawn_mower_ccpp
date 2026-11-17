"""Contour passes: the part of a hybrid coverage pattern that follows the edges.

Boustrophedon lanes (problem.py) cover the inside of the lawn, where the
robot has room to turn in place at each lane end. That leaves a strip along
the lawn edge and round each obstacle. Contour passes cover it in two rings:

    turn ring   the boundary of the free area, where a full turn in place
                still fits. The robot can drive it as a closed loop.
    side ring   closer to the edge, where the body fits only when it drives
                along the edge. At a corner that has no room to turn, the pass
                leaves the edge on a shallow diagonal to a point of the free
                area, turns there, and comes back to the next edge.

Every run and every pivot of a pass is checked with footprint.BodyCheck; a
piece that does not fit is left out. The runs along the edges keep a larger
margin than the manoeuvres that join them. Each pass starts and ends in the free
area, so that passes can be linked through the roadmap.
"""

import math
from dataclasses import dataclass

from shapely.geometry import Point
from shapely.ops import nearest_points

from .decomposition import polygons
from .footprint import heading

COLLINEAR = 1e-6          # m, simplification of the buffered rings
MIN_RUN = 0.5             # m, shortest run along an edge worth a pass
SETBACK_STEP = 0.25       # m, search step of the distance a pass leaves an edge before a corner
SETBACK_LIMIT = 6.0       # m
SAME_POINT = 1e-6         # m


def rings(area):
    """The boundary rings of an area as open vertex lists, collinear vertices removed."""
    out = []
    for poly in polygons(area.simplify(COLLINEAR)):
        for ring in [poly.exterior, *poly.interiors]:
            points = [tuple(c) for c in ring.coords[:-1]]
            if len(points) >= 3:
                out.append(points)
    return out


def _unit(p, q):
    d = math.dist(p, q)
    return ((q[0] - p[0]) / d, (q[1] - p[1]) / d)


def _along(p, direction, distance):
    return (p[0] + direction[0] * distance, p[1] + direction[1] * distance)


def _nearest(area, point):
    return tuple(nearest_points(area, Point(point))[0].coords[0])


@dataclass(frozen=True)
class Corner:
    """How a side ring gets round one of its vertices."""

    kind: str            # pivot: turn in place at the vertex; pull: through `inside`; blocked
    setback: float = 0.0  # m of each adjacent edge that a pull leaves out
    leave: tuple = None  # pull: point on the edge before the vertex
    inside: tuple = None  # pull: point of the free area where the robot turns
    join: tuple = None   # pull: point on the edge after the vertex


def turn_ring_passes(free):
    """One closed pass along each boundary ring of the free area."""
    return [ring + [ring[0]] for ring in rings(free)]


def _corner(vertex, before, after, free, run_check, pivot_check):
    """The Corner at `vertex` of a ring, between the vertices before and after it."""
    h_in, h_out = heading(before, vertex), heading(vertex, after)
    if pivot_check.pivot(vertex, h_in, h_out):
        return Corner('pivot')
    inside = _nearest(free, vertex)
    if math.dist(inside, vertex) < SAME_POINT:
        return Corner('blocked')
    u_in, u_out = _unit(before, vertex), _unit(vertex, after)
    reach = min(math.dist(before, vertex), math.dist(vertex, after), SETBACK_LIMIT)
    setback = SETBACK_STEP
    while setback <= reach:
        leave, join = _along(vertex, u_in, -setback), _along(vertex, u_out, setback)
        h_leave, h_join = heading(leave, inside), heading(inside, join)
        if (pivot_check.pivot(leave, h_in, h_leave) and pivot_check.run(leave, inside)
                and pivot_check.pivot(inside, h_leave, h_join) and pivot_check.run(inside, join)
                and pivot_check.pivot(join, h_join, h_out)):
            return Corner('pull', setback, leave, inside, join)
        setback += SETBACK_STEP
    return Corner('blocked')


def _spur(vertex, toward, free, run_check, pivot_check):
    """Point of the free area from which the robot can drive straight to `vertex`
    and turn there towards `toward`, or None. The same line serves to leave."""
    inside = _nearest(free, vertex)
    if math.dist(inside, vertex) < SAME_POINT:
        return inside
    ok = (pivot_check.run(inside, vertex) and pivot_check.run(vertex, inside)
          and pivot_check.pivot(vertex, heading(inside, vertex), heading(vertex, toward))
          and pivot_check.pivot(vertex, heading(toward, vertex), heading(vertex, inside)))
    return inside if ok else None


def side_ring_passes(side, free, run_check, pivot_check):
    """Passes along the boundary rings of `side`, the area the robot centre may
    use while it drives along an edge.

    free         the area where a full turn in place fits (the planner's free area)
    run_check    BodyCheck for the runs along the edges, with the full margin
    pivot_check  BodyCheck for the manoeuvres: the turns in place, and the short
                 diagonals between an edge and the free area. A diagonal near
                 an edge cannot keep the margin of a run along it.
    Returns a list of polylines. Each starts and ends at a point of `free`.
    """
    passes = []
    for ring in rings(side):
        passes.extend(_ring_passes(ring, free, run_check, pivot_check))
    return passes


def _ring_passes(ring, free, run_check, pivot_check):
    n = len(ring)
    corners = [_corner(ring[i], ring[i - 1], ring[(i + 1) % n], free, run_check, pivot_check)
               for i in range(n)]

    def edge_ends(i):
        """Start and end point of the run along edge i (vertex i to vertex i + 1)."""
        j = (i + 1) % n
        start = corners[i].join if corners[i].kind == 'pull' else ring[i]
        end = corners[j].leave if corners[j].kind == 'pull' else ring[j]
        return start, end

    usable = []
    for i in range(n):
        j = (i + 1) % n
        length = math.dist(ring[i], ring[j])
        start, end = edge_ends(i)
        usable.append(corners[i].kind != 'blocked' and corners[j].kind != 'blocked'
                      and length - corners[i].setback - corners[j].setback >= MIN_RUN
                      and run_check.run(start, end))

    # A run that ends at a pivot corner needs the next edge, or a spur to the free area.
    spur_in, spur_out = {}, {}
    changed = True
    while changed:
        changed = False
        for i in range(n):
            if not usable[i]:
                continue
            j = (i + 1) % n
            if corners[i].kind == 'pivot' and not usable[i - 1] and i not in spur_in:
                spur_in[i] = _spur(ring[i], ring[j], free, run_check, pivot_check)
            if corners[j].kind == 'pivot' and not usable[j] and i not in spur_out:
                spur_out[i] = _spur(ring[j], ring[i], free, run_check, pivot_check)
            if spur_in.get(i, True) is None or spur_out.get(i, True) is None:
                usable[i] = False
                spur_in.pop(i, None)
                spur_out.pop(i, None)
                changed = True
        # A neighbour that became unusable can leave a run without its way in or out.
        for i in list(spur_in):
            if not usable[i]:
                del spur_in[i]
        for i in list(spur_out):
            if not usable[i]:
                del spur_out[i]

    if not any(usable):
        return []

    def starts_pass(i):
        """Edge i begins a pass: it is entered from the free area."""
        return usable[i] and (corners[i].kind == 'pull' and not usable[i - 1]
                              or corners[i].kind == 'pivot' and not usable[i - 1])

    begin = [i for i in range(n) if starts_pass(i)]
    closed = not begin
    if closed:
        # Every edge is usable. Enter at a pull corner, or by a spur at a pivot corner.
        pulls = [i for i in range(n) if corners[i].kind == 'pull']
        if pulls:
            begin = [pulls[0]]
        else:
            for i in range(n):
                inside = _spur(ring[i], ring[(i + 1) % n], free, run_check, pivot_check)
                back = _spur(ring[i], ring[i - 1], free, run_check, pivot_check)
                if inside is not None and back is not None:
                    spur_in[i] = inside
                    spur_out[(i - 1) % n] = back
                    begin = [i]
                    break
            if not begin:
                return []

    passes = []
    for first in begin:
        points = []
        i = first
        if corners[i].kind == 'pull':
            points.append(corners[i].inside)
        else:
            points.append(spur_in[i])
        while True:
            start, end = edge_ends(i)
            points += [start, end]
            j = (i + 1) % n
            last = (closed and j == first) or not usable[j]
            if corners[j].kind == 'pull':
                points.append(corners[j].inside)
            elif last:
                points.append(spur_out[i])
            if last:
                break
            i = j
        passes.append(_clean(points))
    return passes


def _clean(points):
    out = [points[0]]
    for p in points[1:]:
        if math.dist(out[-1], p) > SAME_POINT:
            out.append(p)
    return out


def order_passes(passes, start, roadmap):
    """Visit the passes nearest first from `start`.

    A closed pass is entered at its vertex nearest to the robot, among its
    vertices in the free area; an open pass at its nearer end. Returns [(link, pass)], where link is the roadmap
    polyline from the previous position to the first point of the pass, or
    None when the pass cannot be reached (it is then left out).
    """
    todo = list(passes)
    here = tuple(start)
    out = []
    while todo:
        options = []
        for k, points in enumerate(todo):
            if math.dist(points[0], points[-1]) < SAME_POINT:
                entry = min((e for e in range(len(points) - 1)
                             if e == 0 or roadmap.visible(points[e], points[e])),
                            key=lambda e: math.dist(here, points[e]))
                route = points[entry:-1] + points[:entry] + [points[entry]]
                options.append((math.dist(here, route[0]), k, route))
            else:
                options.append((math.dist(here, points[0]), k, points))
                options.append((math.dist(here, points[-1]), k, points[::-1]))
        options.sort(key=lambda o: o[0])
        for _, k, route in options:
            link = roadmap.path(here, route[0])
            if link is not None:
                out.append((link, route))
                here = route[-1]
                todo.pop(k)
                break
        else:
            break   # nothing left that can be reached
    return out
