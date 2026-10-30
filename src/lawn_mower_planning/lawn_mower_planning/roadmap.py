"""Shortest paths between cells: A* on the visibility graph of the free area.

The nodes are the vertices of the free area (outer boundary and obstacles);
two points are joined when the straight segment between them stays in the
free area. The shortest path among polygonal obstacles bends only at these
vertices, so A* with the straight-line heuristic returns the exact shortest
path, with few vertices to count as turns.
"""

import heapq
import math

from shapely.geometry import LineString, Point
from shapely.prepared import prep

from .decomposition import polygons

INSIDE_TOLERANCE = 1e-7  # m


class Roadmap:
    def __init__(self, area):
        self._inside = prep(area.buffer(INSIDE_TOLERANCE))
        self.nodes = [tuple(c) for p in polygons(area)
                      for ring in [p.exterior, *p.interiors] for c in ring.coords[:-1]]
        self.edges = [[] for _ in self.nodes]
        for i, a in enumerate(self.nodes):
            for j in range(i + 1, len(self.nodes)):
                b = self.nodes[j]
                if self.visible(a, b):
                    d = math.dist(a, b)
                    self.edges[i].append((j, d))
                    self.edges[j].append((i, d))
        self._seen = {}

    def visible(self, a, b):
        if math.dist(a, b) == 0:
            return self._inside.covers(Point(a))
        return self._inside.covers(LineString([a, b]))

    def _visible_nodes(self, p):
        if p not in self._seen:
            self._seen[p] = [(i, math.dist(p, n)) for i, n in enumerate(self.nodes)
                             if self.visible(p, n)]
        return self._seen[p]

    def path(self, p, q):
        """Shortest polyline from p to q in the free area, or None if q is unreachable."""
        p, q = tuple(p), tuple(q)
        if self.visible(p, q):
            return [p, q]
        goal = len(self.nodes)
        to_goal = dict(self._visible_nodes(q))
        best = {}
        came = {}
        heap = []
        for i, d in self._visible_nodes(p):
            best[i] = d
            came[i] = None
            heapq.heappush(heap, (d + math.dist(self.nodes[i], q), d, i))
        while heap:
            _, d, i = heapq.heappop(heap)
            if i == goal:
                out = [q]
                i = came[goal]
                while i is not None:
                    out.append(self.nodes[i])
                    i = came[i]
                return [p] + out[::-1]
            if d > best.get(i, math.inf):
                continue
            steps = list(self.edges[i])
            if i in to_goal:
                steps.append((goal, to_goal[i]))
            for j, step in steps:
                nd = d + step
                if nd < best.get(j, math.inf):
                    best[j], came[j] = nd, i
                    h = 0.0 if j == goal else math.dist(self.nodes[j], q)
                    heapq.heappush(heap, (nd + h, nd, j))
        return None
