"""Обход запретных зон: кратчайший путь по графу видимости (Lozano-Pérez & Wesley, 1979).

Узлы графа — вершины раздутых препятствий (NFZ + запас) и вершины границы разрешённой зоны,
рёбра — пары взаимно видимых узлов. Для запроса p→q добавляем p и q и ищем путь Дейкстрой.
"""
from __future__ import annotations

import math
from functools import lru_cache

import networkx as nx
import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.prepared import prep

Pt = tuple[float, float]


class Router:
    def __init__(self, nfz: list[BaseGeometry], margin: float, allowed: BaseGeometry | None = None):
        self.enabled = bool(nfz) or allowed is not None
        # Запас чуть меньше, чем у рабочей области: галсы лежат вне раздутых NFZ, и концы галсов
        # не должны оказываться «внутри» препятствия.
        inflate = max(margin * 0.8, 1.0)
        obst = [z.buffer(inflate, quad_segs=2) for z in nfz]
        self.blocked = unary_union(obst) if obst else Polygon()
        self.allowed = allowed.buffer(1.0) if allowed is not None else None
        # для проверок пересечения — чуть ужатые препятствия, чтобы касание по ребру не считалось
        core = self.blocked.buffer(-0.5)
        self._core = None if core.is_empty else core
        if self._core is not None:
            shapely.prepare(self._core)
        if self.allowed is not None:
            shapely.prepare(self.allowed)
        self._blocked_core = prep(core) if self._core is not None else None
        self._allowed_p = prep(self.allowed) if self.allowed is not None else None
        nodes: list[Pt] = []
        for g in getattr(self.blocked, "geoms", [self.blocked]):
            if isinstance(g, Polygon) and not g.is_empty:
                nodes += [(x, y) for x, y in g.exterior.coords[:-1]]
        if allowed is not None:
            # вершины разрешённой зоны, чуть сдвинутые внутрь, — для обхода её вогнутостей
            inner = allowed.buffer(-inflate, quad_segs=2)
            for g in getattr(inner, "geoms", [inner]):
                if isinstance(g, Polygon) and not g.is_empty:
                    nodes += [(x, y) for x, y in g.exterior.coords[:-1]]
        self.nodes = [n for n in nodes if self._point_free(n)]
        self.graph = nx.Graph()
        self.graph.add_nodes_from(range(len(self.nodes)))
        for i, a in enumerate(self.nodes):
            rest = self.nodes[i + 1:]
            if not rest:
                continue
            for k in np.flatnonzero(self.visible_many(a, rest)):
                j = i + 1 + int(k)
                self.graph.add_edge(i, j, weight=math.dist(a, self.nodes[j]))

    def _point_free(self, p: Pt) -> bool:
        pt = Point(p)
        if self._blocked_core is not None and self._blocked_core.contains(pt):
            return False
        return self._allowed_p is None or self._allowed_p.contains(pt)

    def visible(self, a: Pt, b: Pt) -> bool:
        if math.dist(a, b) < 1e-6:
            return True
        line = LineString([a, b])
        if self._blocked_core is not None and self._blocked_core.intersects(line):
            return False
        return self._allowed_p is None or self._allowed_p.contains(line)

    def visible_many(self, a: Pt, pts: list[Pt]) -> np.ndarray:
        """Векторная проверка видимости из a во все точки pts."""
        if not pts:
            return np.zeros(0, dtype=bool)
        coords = np.empty((len(pts), 2, 2))
        coords[:, 0, :] = a
        coords[:, 1, :] = pts
        lines = shapely.linestrings(coords)
        ok = np.ones(len(pts), dtype=bool)
        if self._core is not None:
            ok &= ~shapely.intersects(self._core, lines)
        if self.allowed is not None:
            ok &= shapely.contains(self.allowed, lines)
        return ok

    def polyline_free(self, pts: list[Pt]) -> bool:
        if len(pts) < 2:
            return True
        line = LineString(pts)
        if self._core is not None and shapely.intersects(self._core, line):
            return False
        return self.allowed is None or bool(shapely.contains(self.allowed, line))

    def route(self, p: Pt, q: Pt) -> list[Pt]:
        """Точки пути p→q, включая концы. Если пути нет — прямая (проверка потом выдаст предупреждение)."""
        if not self.enabled or self.visible(p, q):
            return [p, q]
        return list(self._route_cached((round(p[0], 2), round(p[1], 2)), (round(q[0], 2), round(q[1], 2))))

    @lru_cache(maxsize=20000)
    def _route_cached(self, p: Pt, q: Pt) -> tuple[Pt, ...]:
        g = self.graph.copy()
        src, dst = "p", "q"
        g.add_node(src)
        g.add_node(dst)
        for i in np.flatnonzero(self.visible_many(p, self.nodes)):
            g.add_edge(src, int(i), weight=math.dist(p, self.nodes[i]))
        for i in np.flatnonzero(self.visible_many(q, self.nodes)):
            g.add_edge(int(i), dst, weight=math.dist(self.nodes[i], q))
        try:
            path = nx.shortest_path(g, src, dst, weight="weight")
        except nx.NetworkXNoPath:
            return (p, q)
        return tuple(p if k == src else q if k == dst else self.nodes[k] for k in path)


def length(pts: list[Pt]) -> float:
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
