"""Разбиение области с учётом дальности бортов (по мотивам DARP, Kapoutsis et al., 2017).

Область покрывается сеткой, выровненной по направлению галсов. Каждый борт «растит» свой участок
от клетки, ближайшей к его базе: на каждом шаге расширяется борт с наименьшей долей выполненной
нормы (площадь / целевая доля), беря соседнюю свободную клетку, ближайшую к своей базе.
Клетки дальше радиуса действия борта (заряд туда-обратно, дальность радиоканала) ему недоступны.
Так короткодействующие мультироторы остаются у своих баз, а дальние участки уходят самолётам.
"""
from __future__ import annotations

import heapq
import math

from shapely import affinity
from shapely.geometry import MultiPolygon, Point, Polygon, box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.validation import make_valid


_GRID_CACHE: dict[tuple, tuple] = {}


def _grid(area: BaseGeometry, angle: float, cell: float):
    """Клетки сетки (кэшируется: не зависит от долей бортов)."""
    key = (area.wkb, round(angle, 9), round(cell, 3))
    if key in _GRID_CACHE:
        return _GRID_CACHE[key]
    rot = affinity.rotate(area, -angle, origin=(0, 0), use_radians=True)
    minx, miny, maxx, maxy = rot.bounds
    nx_, ny_ = math.ceil((maxx - minx) / cell), math.ceil((maxy - miny) / cell)
    pieces: dict[tuple[int, int], BaseGeometry] = {}
    centers: dict[tuple[int, int], tuple[float, float]] = {}
    weights: dict[tuple[int, int], float] = {}
    for i in range(nx_):
        for j in range(ny_):
            sq = box(minx + i * cell, miny + j * cell, minx + (i + 1) * cell, miny + (j + 1) * cell)
            if not sq.intersects(rot):
                continue
            pc = sq.intersection(rot)
            if pc.area < 0.01 * cell * cell:
                continue
            pieces[(i, j)] = pc
            c = pc.centroid
            centers[(i, j)] = (c.x, c.y)
            weights[(i, j)] = pc.area
    if len(_GRID_CACHE) > 32:
        _GRID_CACHE.clear()
    _GRID_CACHE[key] = (pieces, centers, weights)
    return pieces, centers, weights


def grid_partition(
    area: BaseGeometry,
    angle: float,
    fractions: list[float],
    bases: list[tuple[float, float]],
    reach: list[float],
    cell: float | None = None,
) -> tuple[list[BaseGeometry], BaseGeometry]:
    """Возвращает участки бортов (в порядке fractions) и недостижимый остаток."""
    n = len(fractions)
    if cell is None:
        cell = max(math.sqrt(area.area / 1500.0), 100.0)
    rb = [affinity.rotate(Point(b), -angle, origin=(0, 0), use_radians=True).coords[0] for b in bases]
    pieces, centers, weights = _grid(area, angle, cell)

    total = sum(weights.values())
    s = sum(fractions)
    target = [f / s * total for f in fractions]
    far = cell / math.sqrt(2)
    dist = {k: [math.dist(centers[k], rb[d]) for d in range(n)] for k in pieces}
    ok = {k: [dist[k][d] + far <= reach[d] for d in range(n)] for k in pieces}

    owner: dict[tuple[int, int], int] = {}
    load = [0.0] * n
    frontier: list[list[tuple[float, tuple[int, int]]]] = [[] for _ in range(n)]

    def push_neighbours(d: int, k: tuple[int, int]) -> None:
        i, j = k
        for nb in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if nb in pieces and nb not in owner and ok[nb][d]:
                heapq.heappush(frontier[d], (dist[nb][d], nb))

    # стартовые клетки — ближайшие к базам
    for d in range(n):
        if target[d] <= 0:
            continue
        cands = sorted((dist[k][d], k) for k in pieces if ok[k][d] and k not in owner)
        if cands:
            k = cands[0][1]
            owner[k] = d
            load[d] += weights[k]
            push_neighbours(d, k)

    while True:
        order = sorted(
            (load[d] / target[d], d) for d in range(n) if target[d] > 0 and frontier[d]
        )
        grown = False
        for _, d in order:
            while frontier[d]:
                _, k = heapq.heappop(frontier[d])
                if k in owner:
                    continue
                owner[k] = d
                load[d] += weights[k]
                push_neighbours(d, k)
                grown = True
                break
            if grown:
                break
        if not grown:
            break

    # оставшиеся доступные клетки (отрезанные чужими участками) — наименее загруженному
    leftover: list[BaseGeometry] = []
    for k in pieces:
        if k in owner:
            continue
        avail = [d for d in range(n) if ok[k][d] and target[d] > 0]
        if avail:
            d = min(avail, key=lambda x: load[x] / target[x])
            owner[k] = d
            load[d] += weights[k]
        else:
            leftover.append(pieces[k])

    parts = []
    for d in range(n):
        g = unary_union([pieces[k] for k, o in owner.items() if o == d]) if target[d] > 0 else Polygon()
        parts.append(affinity.rotate(_polys(g.buffer(0.01).buffer(-0.01)), angle, origin=(0, 0), use_radians=True))
    rest = affinity.rotate(_polys(unary_union(leftover)), angle, origin=(0, 0), use_radians=True) if leftover else Polygon()
    return parts, rest


def _polys(g: BaseGeometry) -> BaseGeometry:
    if not g.is_valid:
        g = make_valid(g)
    if isinstance(g, (Polygon, MultiPolygon)):
        return g
    ps = [x for x in getattr(g, "geoms", []) if isinstance(x, Polygon)]
    return MultiPolygon(ps) if ps else Polygon()
