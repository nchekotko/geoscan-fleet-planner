"""Покрытие области галсами (boustrophedon).

Галсы строятся параллельными линиями под углом `angle` с шагом `spacing` и обрезаются областью,
поэтому невыпуклые полигоны и дыры (NFZ) дают несколько отрезков на одной линии
(аналог клеточной декомпозиции Choset, 2000). Направление галсов выбирается перебором:
кандидаты — направления рёбер выпуклой оболочки (Huang, 2001) и сетка углов; оценка — время
полёта с учётом ветра (Coombes et al., 2017), а не длина.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry

from .fleet import DroneModel
from .turns import turn_time
from .wind import Wind, segment_time


@dataclass
class Pass:
    """Один галс: отрезок съёмки. line — номер линии развёртки, seq — порядок на линии."""

    line: int
    seq: int
    a: tuple[float, float]
    b: tuple[float, float]
    lo: float = 0.0  # интервал вдоль направления галсов (в повёрнутой системе)
    hi: float = 0.0

    @property
    def length(self) -> float:
        return math.dist(self.a, self.b)

    @property
    def mid(self) -> tuple[float, float]:
        return ((self.a[0] + self.b[0]) / 2, (self.a[1] + self.b[1]) / 2)


def sweep_passes(
    area: Polygon | MultiPolygon, angle: float, spacing: float, min_len: float = 1.0
) -> list[Pass]:
    """Галсы под углом angle (рад, 0 = восток) с шагом spacing. Первая линия — на spacing/2
    от края, чтобы полоса съёмки покрывала границу."""
    rot = affinity.rotate(area, -angle, origin=(0, 0), use_radians=True)
    minx, miny, maxx, maxy = rot.bounds
    passes: list[Pass] = []
    n_lines = max(1, math.ceil((maxy - miny) / spacing))
    # Центрируем набор линий, чтобы запас с обеих сторон был одинаковым.
    y0 = miny + ((maxy - miny) - (n_lines - 1) * spacing) / 2
    for i in range(n_lines):
        y = y0 + i * spacing
        cut = LineString([(minx - 1, y), (maxx + 1, y)]).intersection(rot)
        segs = _lines_of(cut)
        segs.sort(key=lambda s: min(s.coords[0][0], s.coords[-1][0]))
        for j, s in enumerate(s for s in segs if s.length >= min_len):
            x1, x2 = sorted((s.coords[0][0], s.coords[-1][0]))
            back = affinity.rotate(LineString([(x1, y), (x2, y)]), angle, origin=(0, 0), use_radians=True)
            c = list(back.coords)
            passes.append(Pass(line=i, seq=j, a=c[0], b=c[1], lo=x1, hi=x2))
    return passes


def _lines_of(g: BaseGeometry) -> list[LineString]:
    if g.is_empty:
        return []
    if isinstance(g, LineString):
        return [g]
    return [x for x in getattr(g, "geoms", []) if isinstance(x, LineString)]


def candidate_angles(area: BaseGeometry, step_deg: float = 10.0) -> list[float]:
    hull = area.convex_hull
    angles: set[float] = set()
    if isinstance(hull, Polygon):
        c = list(hull.exterior.coords)
        for p, q in zip(c, c[1:]):
            if math.dist(p, q) > 1e-6:
                angles.add(round(math.atan2(q[1] - p[1], q[0] - p[0]) % math.pi, 6))
    k = 0.0
    while k < 180.0:
        angles.add(round(math.radians(k), 6))
        k += step_deg
    return sorted(angles)


def estimate_coverage_time(
    passes: list[Pass], angle: float, spacing: float, drone: DroneModel, speed: float, wind: Wind
) -> float | None:
    """Оценка времени съёмки: галсы идут попеременно туда и обратно + развороты."""
    if not passes:
        return 0.0
    t_fwd = segment_time(1.0, speed, angle, wind)
    t_back = segment_time(1.0, speed, angle + math.pi, wind)
    if t_fwd is None or t_back is None:
        return None
    total_len = sum(p.length for p in passes)
    t = total_len * (t_fwd + t_back) / 2
    n_lines = len({p.line for p in passes})
    extra_segments = len(passes) - n_lines
    t += (n_lines - 1) * turn_time(drone, speed, spacing)
    # переходы между кусками одной линии через дыры — грубо, как лишний разворот
    t += extra_segments * turn_time(drone, speed, spacing)
    return t


def best_direction(
    area: Polygon | MultiPolygon, spacing: float, drone: DroneModel, speed: float, wind: Wind
) -> tuple[float, list[Pass], float]:
    best: tuple[float, list[Pass], float] | None = None
    for ang in candidate_angles(area):
        passes = sweep_passes(area, ang, spacing)
        t = estimate_coverage_time(passes, ang, spacing, drone, speed, wind)
        if t is None:
            continue
        if best is None or t < best[2]:
            best = (ang, passes, t)
    if best is None:
        raise ValueError(f"{drone.name}: ветер не позволяет выполнить съёмку ни в одном направлении")
    return best


def boustrophedon_cells(passes: list[Pass]) -> list[list[Pass]]:
    """Клеточная декомпозиция (Choset, 2000) по готовым галсам: куски соседних линий,
    которые переходят друг в друга один-к-одному, образуют одну ячейку. На событиях
    (появление/исчезновение дыры, ветвление области) начинается новая ячейка."""
    by_line: dict[int, list[Pass]] = {}
    for p in passes:
        by_line.setdefault(p.line, []).append(p)
    for v in by_line.values():
        v.sort(key=lambda p: p.seq)

    def overlaps(a: Pass, b: Pass) -> bool:
        return a.lo < b.hi and b.lo < a.hi

    cell_of: dict[int, int] = {}
    cells: list[list[Pass]] = []
    prev: list[Pass] = []
    for line in sorted(by_line):
        cur = by_line[line]
        for p in cur:
            ups = [q for q in prev if overlaps(p, q)]
            if len(ups) == 1:
                q = ups[0]
                downs = [r for r in cur if overlaps(q, r)]
                if len(downs) == 1 and (line - 1 == q.line):
                    cid = cell_of[id(q)]
                    cells[cid].append(p)
                    cell_of[id(p)] = cid
                    continue
            cell_of[id(p)] = len(cells)
            cells.append([p])
        prev = cur
    return cells
