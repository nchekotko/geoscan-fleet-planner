"""Разбиение рабочей области между бортами полосами заданных долей площади
(Maza & Ollero, 2007: доли пропорциональны возможностям БВС).

Полосы режутся линиями, параллельными направлению галсов, поэтому каждая полоса содержит
галсы полной длины и минимум разворотов. Положение каждого разреза ищется бисекцией по площади.
"""
from __future__ import annotations

import math

from shapely import affinity
from shapely.geometry import MultiPolygon, Polygon, box
from shapely.geometry.base import BaseGeometry


def split_by_fractions(
    area: BaseGeometry, angle: float, fractions: list[float]
) -> list[BaseGeometry]:
    """Режет area на len(fractions) полос по оси, перпендикулярной направлению angle.
    Полосы идут снизу вверх в повёрнутой системе координат."""
    total = sum(fractions)
    if total <= 0:
        raise ValueError("пустые доли")
    rot = affinity.rotate(area, -angle, origin=(0, 0), use_radians=True)
    minx, miny, maxx, maxy = rot.bounds
    full = rot.area
    parts: list[BaseGeometry] = []
    y_prev = miny
    acc = 0.0
    for i, f in enumerate(fractions):
        acc += f / total
        if i == len(fractions) - 1:
            y_cut = maxy
        else:
            target = acc * full
            lo, hi = y_prev, maxy
            for _ in range(50):
                mid = (lo + hi) / 2
                if rot.intersection(box(minx - 1, miny - 1, maxx + 1, mid)).area < target:
                    lo = mid
                else:
                    hi = mid
            y_cut = (lo + hi) / 2
        strip = rot.intersection(box(minx - 1, y_prev, maxx + 1, y_cut))
        parts.append(affinity.rotate(_polygonal(strip), angle, origin=(0, 0), use_radians=True))
        y_prev = y_cut
    return parts


def strip_axis_position(point: tuple[float, float], angle: float) -> float:
    """Координата точки по оси, перпендикулярной галсам (для сопоставления баз и полос)."""
    return -point[0] * math.sin(angle) + point[1] * math.cos(angle)


def _polygonal(g: BaseGeometry) -> BaseGeometry:
    if isinstance(g, (Polygon, MultiPolygon)):
        return g
    polys = [x for x in getattr(g, "geoms", []) if isinstance(x, Polygon)]
    return MultiPolygon(polys) if polys else Polygon()
