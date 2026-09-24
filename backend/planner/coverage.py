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
from shapely.geometry import LineString, MultiPolygon, Polygon, box
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
    area: Polygon | MultiPolygon, angle: float, spacing: float, min_len: float = 1.0,
    swath: float = 0.0, free: BaseGeometry | None = None, need: BaseGeometry | None = None,
) -> list[Pass]:
    """Галсы под углом angle (рад, 0 = восток) с шагом spacing. Первая линия — на spacing/2
    от края, чтобы полоса съёмки покрывала границу.

    swath > 0 — «добор кромок»: галс продлевается за край участка настолько, чтобы полоса
    захвата накрыла всё, что попадает в её ширину. У наклонной кромки галс обрывается раньше
    соседнего, и между ними остаётся неснятый клин — на LiDAR-сценарии такие клинья давали
    0,8 % площади. Продление ограничено областью free (там, где летать можно), а need задаёт,
    ради чего продлевать: без неё — ради всего участка, с ней — только ради неснятых кусков
    (иначе на участках с рваной границей галсы удлиняются всюду, а покрытие почти не растёт).
    """
    rot = affinity.rotate(area, -angle, origin=(0, 0), use_radians=True)
    free_rot = (affinity.rotate(free, -angle, origin=(0, 0), use_radians=True)
                if (swath > 0 and free is not None) else None)
    need_rot = (affinity.rotate(need, -angle, origin=(0, 0), use_radians=True)
                if (swath > 0 and need is not None) else None)
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
        band = (box(minx - 1, y - swath / 2, maxx + 1, y + swath / 2)
                .intersection(rot if need_rot is None else need_rot)) if swath > 0 else None
        for j, s in enumerate(s for s in segs if s.length >= min_len):
            x1, x2 = sorted((s.coords[0][0], s.coords[-1][0]))
            if band is not None and not band.is_empty:
                x1 -= _room(band, free_rot, y, swath, x1, back=True)
                x2 += _room(band, free_rot, y, swath, x2, back=False)
            back = affinity.rotate(LineString([(x1, y), (x2, y)]), angle, origin=(0, 0), use_radians=True)
            c = list(back.coords)
            passes.append(Pass(line=i, seq=j, a=c[0], b=c[1], lo=x1, hi=x2))
    return passes


def _room(band: BaseGeometry, free_rot: BaseGeometry | None, y: float, swath: float,
          x: float, back: bool) -> float:
    """На сколько продлить галс за точку x, чтобы полоса захвата накрыла участок в пределах
    своей ширины. Дальше половины ширины полосы не продлеваем и из области полёта не выходим."""
    cap = swath / 2
    lo, hi = (x - cap, x) if back else (x, x + cap)
    part = band.intersection(box(lo, y - swath / 2, hi, y + swath / 2))
    if part.is_empty:
        return 0.0
    need = (x - part.bounds[0]) if back else (part.bounds[2] - x)
    if need <= 0.5:
        return 0.0
    if free_rot is not None:
        # не выходим за пределы разрешённого объёма и не заходим в запретные зоны
        seg = LineString([(x - need, y), (x, y)] if back else [(x, y), (x + need, y)])
        allowed = seg.intersection(free_rot)
        room = 0.0
        for part_line in _lines_of(allowed):
            a, b = sorted((part_line.coords[0][0], part_line.coords[-1][0]))
            if (back and b >= x - 1e-6) or (not back and a <= x + 1e-6):
                room = max(room, (x - a) if back else (b - x))
        need = min(need, room)
    return max(need, 0.0)


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
    passes: list[Pass], angle: float, spacing: float, drone: DroneModel, speed: float, wind: Wind,
    scale: float = 1.0,
) -> float | None:
    """Оценка времени съёмки: галсы идут попеременно туда и обратно + развороты.
    scale — пересчёт галсов, построенных с другим шагом s₀, на шаг spacing: длина галсов
    и число линий ∝ 1/шаг, т. е. scale = s₀ / spacing."""
    if not passes:
        return 0.0
    t_fwd = segment_time(1.0, speed, angle, wind)
    t_back = segment_time(1.0, speed, angle + math.pi, wind)
    if t_fwd is None or t_back is None:
        return None
    total_len = sum(p.length for p in passes) * scale
    t = total_len * (t_fwd + t_back) / 2
    n_lines = len({p.line for p in passes})
    extra_segments = len(passes) - n_lines
    t += max(n_lines * scale - 1, 0.0) * turn_time(drone, speed, spacing)
    # переходы между кусками одной линии через дыры — грубо, как лишний разворот
    t += extra_segments * scale * turn_time(drone, speed, spacing)
    return t


def ranked_directions(
    area: Polygon | MultiPolygon, specs: list[tuple[float, DroneModel, float]], wind: Wind
) -> list[list[tuple[float, float]]]:
    """Для каждого борта specs = [(шаг галсов, модель, скорость), …] — кандидаты направления
    с оценкой времени съёмки (угол, t), от лучшего к худшему; направления, в которых ветер
    не даёт держать линию пути, отброшены. Галсы строятся один раз, с шагом первого борта,
    для остальных время пересчитывается на их шаг (estimate_coverage_time, scale)."""
    out: list[list[tuple[float, float]]] = [[] for _ in specs]
    s0 = specs[0][0]
    for ang in candidate_angles(area):
        passes = sweep_passes(area, ang, s0)
        for rk, (spacing, drone, speed) in zip(out, specs):
            t = estimate_coverage_time(passes, ang, spacing, drone, speed, wind, scale=s0 / spacing)
            if t is not None:
                rk.append((ang, t))
    for rk in out:
        rk.sort(key=lambda x: x[1])  # сортировка устойчивая: при равенстве — меньший угол
    return out


def best_direction(
    area: Polygon | MultiPolygon, spacing: float, drone: DroneModel, speed: float, wind: Wind
) -> tuple[float, list[Pass], float]:
    ranked = ranked_directions(area, [(spacing, drone, speed)], wind)[0]
    if not ranked:
        raise ValueError(f"{drone.name}: ветер не позволяет выполнить съёмку ни в одном направлении")
    ang, t = ranked[0]
    return ang, sweep_passes(area, ang, spacing), t


def same_direction(a: float, b: float, tol: float) -> bool:
    """Галсы под углами a и b (рад) совпадают с точностью tol: направление берётся по модулю π."""
    d = abs(a - b) % math.pi
    return min(d, math.pi - d) < tol


def wind_axes(wind: Wind) -> list[float]:
    """Направления вдоль и поперёк ветра. Вдоль — галсы попеременно по ветру и против него,
    поперёк — без продольной составляющей, но со сносом (Coombes et al., 2017)."""
    if wind.speed_ms <= 0:
        return []
    wx, wy = wind.vector()
    a = math.atan2(wy, wx) % math.pi
    return [a, (a + math.pi / 2) % math.pi]


def rectangle_axis(area: BaseGeometry) -> list[float]:
    """Длинная сторона минимального ограничивающего прямоугольника (rotating calipers,
    Toussaint, 1983): у вытянутых областей галсы вдоль неё дают меньше разворотов."""
    rect = area.minimum_rotated_rectangle
    if not isinstance(rect, Polygon):
        return []
    c = list(rect.exterior.coords)
    p, q = max(zip(c, c[1:]), key=lambda e: math.dist(*e))
    return [math.atan2(q[1] - p[1], q[0] - p[0]) % math.pi]


def distinct_directions(angles: list[float], k: int, tol_deg: float = 5.0) -> list[float]:
    """Первые k попарно различных (с точностью tol_deg) направлений в порядке приоритета."""
    tol = math.radians(tol_deg)
    out: list[float] = []
    for a in angles:
        if len(out) >= k:
            break
        if not any(same_direction(a, b, tol) for b in out):
            out.append(a)
    return out


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
