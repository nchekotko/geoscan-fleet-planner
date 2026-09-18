"""Нижняя граница времени работ (makespan): релаксация модели планировщика.

Граница верна для любого плана, который может построить модель планировщика (mission.py,
planner.py), а не только для нашего. Что сохранено из модели:
  * вылет борта i не длиннее бюджета B_i; между вылетами — смена АКБ t_swap,i;
  * в каждом вылете есть взлёт и посадка: O_i = takeoff_s + landing_s (как в SortieBuilder);
  * галсы одного борта параллельны (один угол θ на все вылеты борта), идут с шагом s_i
    и воздушной скоростью V_i; путевая скорость — по треугольнику скоростей (wind.py);
  * чтобы снять площадь a, нужно пройти галсами не меньше a / s_i (плюс секущие a / s_tie,i
    у геофизики);
  * вылет начинается и заканчивается на базе, поэтому смещение по галсам в одну сторону
    должно быть возвращено перелётами или разворотами;
  * борта стартуют с одной ВПП по очереди: k-й старт — не раньше k · min(LAUNCH_INTERVAL_S).
Что ослаблено (граница от этого только ниже):
  * развороты не стоят времени; перелёты прямые, без обхода запретных зон;
  * борт может выбрать любой угол галсов (у каждого свой) и любую доступную базу;
  * интервал между стартами — наименьший из интервалов модели, порядок стартов любой;
  * радиус радиоканала и геометрия разбиения не учитываются.

Съёмка в одном вылете. Пусть по галсам вдоль +e пройдено S₊, вдоль −e — S₋ (S = S₊ + S₋,
Δ = S₊ − S₋), g± — путевые скорости на галсах в этих направлениях. Время на галсах
S₊/g₊ + S₋/g₋. Остальной горизонтальный полёт T_o (перелёты, развороты) должен:
  * вернуть смещение Δ: T_o ≥ |Δ|/ρ, где ρ — наибольшая проекция путевой скорости на
    нужное направление (V_тр − w·e для перелётов на транзитной скорости V_тр; у самолёта
    ещё V — в модели время разворота не зависит от ветра);
  * довести борт от базы до области и обратно: T_o ≥ R = min_P γ(P − b) + min_Q γ(b − Q),
    γ(d) = |d| / g_тр(d) — время прямого перелёта по треугольнику скоростей, P, Q ∈ область.
Минимум по Δ кусочно-линейной выпуклой функции достигается при Δ ∈ {0, min(ρR, S), S};
отсюда f_θ(S) — наименьшее горизонтальное время вылета, снимающего S метров галсов, и
S_θ(H) = max{S : f_θ(S) ≤ H} — наибольшая длина галсов при горизонтальном времени H.
Без ветра это просто S = V·(H − R): «полезное время вылета × скорость».

Непрерывная граница. S_θ(H)/H не убывает, поэтому за время T борт снимет не больше
a_i·(T + t_swap,i)/(B_i + t_swap,i), где a_i = s_i·max_θ S_θ(B_i − O_i) — площадь за полный
вылет (у последнего вылета нет смены АКБ после него — отсюда «+ t_swap»). Сумма по бортам
должна покрыть площадь A:
    T_непр = (A − Σ q_i·t_swap,i) / Σ q_i,  q_i = a_i / (B_i + t_swap,i).

Граница с целым числом вылетов (точнее). При фиксированном θ функция S_θ вогнута, поэтому
n вылетов лучше делать равными: cap_i(t) = max_θ max_n n·s_i·S_θ(min(Y_n/n, B_i − O_i)),
Y_n = t − n·O_i − (n − 1)·t_swap,i; max_θ и max_n переставимы, поэтому хватает огибающей
Ŝ(H) = max_θ S_θ(H), которая хранится на сетке по H (значение берётся в ближайшем узле
справа — Ŝ не убывает, оценка остаётся верхней). Борт, стартующий k-м с базы b, работает
t = T − k·Δt_min; распределение бортов по базам и местам в очереди — задача о назначениях
(венгерский алгоритм). Граница — наименьшее T, при котором наибольшая суммарная ёмкость
не меньше A (бисекция).

Угол θ перебирается с шагом 0,25° (плюс оси ветра); на погрешность перебора ёмкость
увеличена на 0,5 %. Время R ищется по точкам границы области с шагом 10 м минус поправка
на шаг (липшицева оценка), так что граница остаётся нижней.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from shapely.geometry import Point

from planner.mission import SortieBuilder
from planner.planner import Candidate, Planner
from planner.wind import Wind

try:  # очередь стартов на ВПП (если её нет в модели — граница её не учитывает)
    from planner.mission import LAUNCH_INTERVAL_S

    MIN_LAUNCH_INTERVAL_S = float(min(LAUNCH_INTERVAL_S.values()))
except ImportError:  # pragma: no cover
    MIN_LAUNCH_INTERVAL_S = 0.0

N_THETA = 720             # углы галсов на [0, π)
THETA_MARGIN = 0.005      # запас на дискретизацию угла
RING_STEP_M = 10.0        # шаг точек границы области при поиске R
H_GRID = 2048             # узлов сетки по горизонтальному времени вылета
MAX_SORTIES = 2000        # предохранитель перебора числа вылетов


def _ground_speed(v: float, heading: np.ndarray, wind: Wind) -> np.ndarray:
    """Путевая скорость вдоль heading так, как её считает модель (SortieBuilder.fly_time):
    треугольник скоростей, а если линию не удержать — max(0,5, V − W)."""
    wx, wy = wind.vector()
    ux, uy = np.cos(heading), np.sin(heading)
    along = wx * ux + wy * uy
    cross = -wx * uy + wy * ux
    ok = np.abs(cross) < v
    gs = along + np.sqrt(np.maximum(v * v - cross * cross, 0.0))
    ok &= gs > 0.5
    return np.where(ok, gs, max(0.5, v - wind.speed_ms))


def _ring_points(geom, step: float) -> np.ndarray:
    pts = []
    for poly in getattr(geom, "geoms", [geom]):
        for ring in [poly.exterior, *poly.interiors]:
            c = np.asarray(ring.coords)
            for a, b in zip(c[:-1], c[1:]):
                n = max(1, math.ceil(math.dist(a, b) / step))
                t = np.arange(n)[:, None] / n
                pts.append(a + (b - a) * t)
    return np.vstack(pts) if pts else np.zeros((0, 2))


def reach_time_s(p: Planner, base: tuple[float, float], v_transit: float) -> float:
    """R: нижняя оценка времени перелёта база → область → база (прямо, по треугольнику скоростей)."""
    if p.area.intersects(Point(*base)):
        return 0.0
    wind = p.req.wind
    pts = _ring_points(p.area, RING_STEP_M)
    g_min = max(0.5, v_transit - wind.speed_ms)  # наименьшая путевая скорость — константа Липшица
    d = pts - np.asarray(base)
    dist = np.hypot(d[:, 0], d[:, 1])
    head = np.arctan2(d[:, 1], d[:, 0])
    out = np.min(dist / _ground_speed(v_transit, head, wind))
    back = np.min(dist / _ground_speed(v_transit, head + math.pi, wind))
    # между точками границы время может быть меньше не более чем на (шаг/2)/g_min в каждую сторону
    return max(0.0, out + back - RING_STEP_M / g_min)


@dataclass
class DroneBound:
    """Модель вылета одного борта с одной базы."""

    instance_id: str
    base_id: str
    budget_s: float        # B — бюджет вылета
    overhead_s: float      # O — взлёт + посадка
    reach_s: float         # R — перелёт до области и обратно
    swap_s: float          # смена АКБ
    spacing_m: float
    area_per_sortie_m2: float = 0.0  # s·max_θ S_θ(B − O) (с запасом THETA_MARGIN)
    # огибающая Ŝ(H) = max_θ S_θ(H) на сетке H = 0, dh, 2dh, …, B − O
    _dh: float = field(repr=False, default=1.0)
    _s_grid: np.ndarray = field(repr=False, default=None)

    def s_hat(self, h: np.ndarray) -> np.ndarray:
        """Верхняя оценка Ŝ(h): значение в ближайшем узле сетки справа."""
        idx = np.clip(np.ceil(np.asarray(h) / self._dh - 1e-9), 0, len(self._s_grid) - 1).astype(int)
        return np.where(np.asarray(h) > 0, self._s_grid[idx], 0.0)

    def capacity_m2(self, t: float) -> float:
        """Наибольшая площадь, которую борт снимет за время t с целым числом вылетов."""
        o, sw, x = self.overhead_s, self.swap_s, self.budget_s - self.overhead_s
        if t < o + self.reach_s or x <= self.reach_s:
            return 0.0
        n_full = max(1, int((t + sw) // (self.budget_s + sw)))
        n_max = max(n_full, min(int((t + sw) // (o + self.reach_s + sw)), n_full + MAX_SORTIES))
        n = np.arange(n_full, n_max + 1, dtype=float)
        y = (t - n * o - (n - 1) * sw) / n
        val = n * self.s_hat(np.minimum(y, x))
        return float(val.max()) * self.spacing_m * (1 + THETA_MARGIN)


def _survey_len(h: np.ndarray, r: float, g_plus, g_minus, rho, tau) -> np.ndarray:
    """S_θ(H) для массива h формы (k, 1) по всем углам θ (результат (k, θ)).
    g_plus, g_minus, rho — по две ориентации: [0] — «+» это +e, [1] — «+» это −e."""
    out = None
    for gp, gm, rh in zip(g_plus, g_minus, rho):
        a0 = 1.0 / gp + tau                              # все галсы в одну сторону, Δ = S ≤ ρR
        pp = 0.5 * (1.0 / gp + 1.0 / gm)
        k = 0.5 * (1.0 / gm - 1.0 / gp)
        small = (h - r) / a0
        big = np.maximum((h - r + rh * r * k) / (pp + tau),   # Δ = ρR
                         h / (1.0 / gp + 1.0 / rh + tau))     # Δ = S
        s = np.where(small <= rh * r, small, big)
        s = np.where(h >= r, np.maximum(s, 0.0), 0.0)
        out = s if out is None else np.maximum(out, s)
    return out


def drone_bounds(p: Planner, c: Candidate) -> list[DroneBound]:
    """Модели вылета борта c для каждой базы, с которой он может работать."""
    wind = p.req.wind
    par = c.params
    sb = SortieBuilder(c.drone, par.speed_ms, par.altitude_agl_m, wind, c.budget_s, None)
    v, vt = par.speed_ms, sb.transit_speed
    overhead = sb.takeoff_s() + sb.landing_s()
    theta = np.linspace(0.0, math.pi, N_THETA, endpoint=False)
    wx, wy = wind.vector()
    if wind.speed_ms > 0:
        a = math.atan2(wy, wx) % math.pi
        theta = np.concatenate([theta, [a, (a + math.pi / 2) % math.pi]])
    g_f = _ground_speed(v, theta, wind)            # галс вдоль +e
    g_b = _ground_speed(v, theta + math.pi, wind)  # галс вдоль −e
    w_e = wx * np.cos(theta) + wy * np.sin(theta)
    fw_turn = v if c.drone.type == "fixed_wing" else 0.0
    # возврат смещения вдоль −e (если больше галсов по +e) и вдоль +e (если больше по −e)
    rho = (np.maximum(np.maximum(vt - w_e, fw_turn), 0.5), np.maximum(np.maximum(vt + w_e, fw_turn), 0.5))
    if par.tie_line_spacing_m:
        g_tie = np.maximum(_ground_speed(v, theta + math.pi / 2, wind), _ground_speed(v, theta - math.pi / 2, wind))
        tau = (par.line_spacing_m / par.tie_line_spacing_m) / g_tie
    else:
        tau = np.zeros_like(theta)
    x = c.budget_s - overhead
    out = []
    for bid in ([c.base_id] if c.base_id else list(p.bases)):
        r = reach_time_s(p, p.bases[bid], vt)
        db = DroneBound(c.instance_id, bid, c.budget_s, overhead, r, c.drone.swap_time_min * 60, par.line_spacing_m)
        if x > 0:
            hs = np.linspace(0.0, x, H_GRID)
            env = np.concatenate([_survey_len(hs[i:i + 256, None], r, (g_f, g_b), (g_b, g_f), rho, tau).max(axis=1)
                                  for i in range(0, H_GRID, 256)])
            db._dh, db._s_grid = hs[1] - hs[0], env
            db.area_per_sortie_m2 = float(env[-1]) * par.line_spacing_m * (1 + THETA_MARGIN)
        else:
            db._s_grid = np.zeros(1)
        out.append(db)
    return out


def _assign_max(w: np.ndarray) -> float:
    """Наибольший вес назначения строк (борта) столбцам (места в очередях), строк ≤ столбцов.
    Венгерский алгоритм (Kuhn–Munkres) на минимум для −w, O(n²·m)."""
    n, m = w.shape
    a = -w
    u, v = np.zeros(n + 1), np.zeros(m + 1)
    p = np.zeros(m + 1, dtype=int)
    way = np.zeros(m + 1, dtype=int)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(m + 1, np.inf)
        used = np.zeros(m + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            cur = a[i0 - 1] - u[i0] - v[1:]
            free = ~used[1:]
            upd = free & (cur < minv[1:])
            minv[1:][upd] = cur[upd]
            way[1:][upd] = j0
            cand = np.where(free, minv[1:], np.inf)
            j1 = int(np.argmin(cand)) + 1
            delta = cand[j1 - 1]
            u[p[used]] += delta
            v[used] -= delta
            minv[~used] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    return float(sum(w[p[j] - 1, j - 1] for j in range(1, m + 1) if p[j] != 0))


@dataclass
class LowerBound:
    area_m2: float
    continuous_s: float      # непрерывная релаксация, без очереди стартов
    integer_s: float         # целые вылеты и очередь стартов на ВПП — основная граница
    integer_noqueue_s: float  # целые вылеты без очереди стартов (для сравнения)
    drones: list[DroneBound]


def _bisect(enough, lo: float) -> float:
    """Наименьшее T ≥ lo с enough(T) (с точностью 0,5 с, возвращается нижний конец)."""
    hi = max(lo, 60.0) * 2
    while not enough(hi):
        lo, hi = hi, hi * 2
        if hi > 1e8:
            return math.inf
    for _ in range(60):
        if hi - lo < 0.5:
            break
        mid = (lo + hi) / 2
        if enough(mid):
            hi = mid
        else:
            lo = mid
    return lo


def lower_bound(p: Planner, launch_interval_s: float | None = None) -> LowerBound:
    """Нижняя граница времени работ для всех допущенных к работе бортов планировщика p."""
    dt = MIN_LAUNCH_INTERVAL_S if launch_interval_s is None else launch_interval_s
    area = p.area.area
    per = [drone_bounds(p, c) for c in p.candidates]
    flat = [d for ds in per for d in ds]
    # непрерывная релаксация: у каждого борта — лучшая из его баз
    best = [max(ds, key=lambda d: d.area_per_sortie_m2 / (d.budget_s + d.swap_s)) for ds in per]
    q = [(d.area_per_sortie_m2 / (d.budget_s + d.swap_s), d.swap_s) for d in best]
    qs = sum(x for x, _ in q)
    t_first = min(d.overhead_s + d.reach_s for d in flat)  # хотя бы один вылет
    t_cont = max((area - sum(x * sw for x, sw in q)) / qs, t_first) if qs > 0 else math.inf

    def cap_free(t: float) -> float:
        return sum(max(d.capacity_m2(t) for d in ds) for ds in per)

    t_int0 = _bisect(lambda t: cap_free(t) >= area, t_cont)

    bases = sorted({d.base_id for d in flat})
    n = len(per)

    def cap_queue(t: float) -> float:
        # места в очереди: (база, k) — старт не раньше k·dt
        w = np.zeros((n, len(bases) * n))
        for i, ds in enumerate(per):
            for d in ds:
                col0 = bases.index(d.base_id) * n
                for k in range(n):
                    w[i, col0 + k] = d.capacity_m2(t - k * dt)
        return _assign_max(w)

    t_int = t_int0 if dt <= 0 else _bisect(lambda t: cap_queue(t) >= area, t_int0)
    return LowerBound(area, t_cont, t_int, t_int0, flat)
