"""The shared pipeline: lawn -> BCD cells -> lanes -> A* transitions -> TransitionTable.

Every ordering method plans on the CoverageProblem that build_problem returns,
so the methods differ only in the order (and variants) they choose.

Geometry is computed in the sweep frame, where the lanes are vertical, and
CoverageProblem.path turns a plan back into world coordinates.
"""

import math
from dataclasses import dataclass

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

from .decomposition import boustrophedon_cells, polygons
from .lanes import VARIANTS, cell_routes
from .path_cost import PathCost, clean, heading
from .roadmap import Roadmap
from .transitions import TransitionTable

RIGHT_ANGLE = math.pi / 2


@dataclass(frozen=True)
class CoverageProblem:
    lawn: object         # world-frame lawn (Polygon with obstacle holes)
    width: float         # cutting width = lane spacing [m]
    lane_angle: float    # world heading of the lanes [rad]
    free: object         # sweep-frame area the robot centre may use
    cells: list          # sweep-frame BCD cells
    routes: list         # routes[c][a]: sweep-frame waypoints covering cell c in variant a
    links: dict          # (i, a, j, b) or (None, None, j, b) for the start -> polyline or None
    table: TransitionTable

    @property
    def _turn(self):
        return RIGHT_ANGLE - self.lane_angle

    def path(self, plan):
        """World-frame waypoints of a plan: start link, cells and transitions."""
        points = []
        prev = (None, None)
        for c, a in zip(plan.order, plan.variants):
            link = self.links.get((*prev, c, a))
            if link is None and prev != (None, None):
                raise ValueError(f'cell {c} cannot be reached from cell {prev[0]}')
            points.extend(link or [])
            points.extend(self.routes[c][a])
            prev = (c, a)
        line = affinity.rotate(LineString(clean(points)), -self._turn, origin=(0, 0),
                               use_radians=True)
        return list(line.coords)

    def coverage(self, plan):
        """Coverage and overlap [%] of the swath (path buffered by width / 2).

        coverage = swath area inside the lawn / lawn area
        overlap  = (sum of segment swaths - swath) / swath
        """
        points = self.path(plan)
        strips = [LineString([p, q]).buffer(self.width / 2, cap_style='flat')
                  for p, q in zip(points, points[1:])]
        swath = unary_union(strips)
        return {'coverage': 100 * swath.intersection(self.lawn).area / self.lawn.area,
                'overlap': 100 * (sum(s.area for s in strips) - swath.area) / swath.area}


def build_problem(lawn, width, path_cost=PathCost(), clearance=None, lane_angle=RIGHT_ANGLE,
                  start=None):
    """Decompose and precompute everything an ordering method needs.

    lawn       world-frame Polygon; obstacles are holes
    width      cutting width and lane spacing [m]
    path_cost  turns polylines into metrics (PathCost.from_config adds energy)
    clearance  distance the robot centre keeps from edges and obstacles [m],
               default width / 2
    lane_angle world heading of the lanes [rad]; default along y
    start      robot start pose (x, y, heading) in the world frame, or None
    """
    clearance = width / 2 if clearance is None else clearance
    turn = RIGHT_ANGLE - lane_angle
    sweep = affinity.rotate(lawn, turn, origin=(0, 0), use_radians=True)
    free = sweep.buffer(-clearance, join_style='mitre') if clearance > 0 else sweep
    if not polygons(free):
        raise ValueError('nothing left of the lawn after the clearance')
    cells = boustrophedon_cells(free)
    routes = [cell_routes(c, width) for c in cells]
    roadmap = Roadmap(free)
    n, k = len(cells), len(VARIANTS)
    metrics = path_cost.metrics

    cell = {m: np.zeros((n, k)) for m in metrics}
    trans = {m: np.full((n, k, n, k), np.inf) for m in metrics}
    begin = None
    links = {}
    entry = [[(r[0], heading(r[0], r[1])) for r in rs] for rs in routes]
    exit_ = [[(r[-1], heading(r[-2], r[-1])) for r in rs] for rs in routes]
    for c in range(n):
        for a in range(k):
            for m, v in path_cost(routes[c][a]).items():
                cell[m][c, a] = v
    for i in range(n):
        for a in range(k):
            p, h0 = exit_[i][a]
            for j in range(n):
                if j == i:
                    continue
                for b in range(k):
                    q, h1 = entry[j][b]
                    link = roadmap.path(p, q)
                    links[(i, a, j, b)] = link
                    if link is not None:
                        for m, v in path_cost(link, h0, h1).items():
                            trans[m][i, a, j, b] = v
    if start is not None:
        x, y, theta = start
        s = affinity.rotate(Point(x, y), turn, origin=(0, 0), use_radians=True).coords[0]
        if not roadmap.visible(s, s):
            raise ValueError(f'start ({x}, {y}) is outside the free area '
                             f'(lawn minus a {clearance} m clearance)')
        begin = {m: np.full((n, k), np.inf) for m in metrics}
        for j in range(n):
            for b in range(k):
                q, h1 = entry[j][b]
                link = roadmap.path(s, q)
                links[(None, None, j, b)] = link
                if link is not None:
                    for m, v in path_cost(link, theta + turn, h1).items():
                        begin[m][j, b] = v
    return CoverageProblem(lawn, width, lane_angle, free, cells, routes, links,
                           TransitionTable(cell, trans, begin))
