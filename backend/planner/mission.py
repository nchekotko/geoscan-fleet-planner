"""Полётное задание одного борта в своей области: порядок галсов, развороты, транзиты,
нарезка на вылеты по заряду.

Нарезка идёт по принципу route-first / cluster-second (Beasley, 1983): сначала строится один
маршрут обхода всех галсов, затем он режется на вылеты так, чтобы каждый укладывался в бюджет
(база → точка входа → галсы → база). Галс, который не помещается целиком, делится.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .avoid import Router
from .coverage import Pass, boustrophedon_cells
from .fleet import DroneModel
from .turns import MULTIROTOR_ACCEL, turn_options
from .wind import Wind, segment_time

# Накладные расходы взлёта/посадки, с (допущения): катапульта + выход на курс, парашют.
FIXED_WING_TAKEOFF_S = 60.0
FIXED_WING_LANDING_S = 120.0
MULTIROTOR_DESCENT_MS = 3.0


@dataclass
class Leg:
    """Участок маршрута. kind: takeoff | transit | survey | turn | return | landing."""

    kind: str
    points: list[tuple[float, float]]
    alt_agl: float
    duration_s: float
    distance_m: float


@dataclass
class Sortie:
    index: int
    base: tuple[float, float]
    base_id: str
    legs: list[Leg] = field(default_factory=list)
    start_s: float = 0.0
    max_divert_s: float = 0.0     # худшее время ухода на ближайшую площадку
    divert_site: str = ""

    @property
    def duration_s(self) -> float:
        return sum(l.duration_s for l in self.legs)

    @property
    def survey_length_m(self) -> float:
        return sum(l.distance_m for l in self.legs if l.kind in ("survey", "tie"))


@dataclass
class DirectedPass:
    a: tuple[float, float]
    b: tuple[float, float]
    kind: str = "survey"  # survey — основной галс, tie — секущий маршрут

    @property
    def heading(self) -> float:
        return math.atan2(self.b[1] - self.a[1], self.b[0] - self.a[0])

    @property
    def length(self) -> float:
        return math.dist(self.a, self.b)


def _snake(cell: list[Pass], start: tuple[float, float], kind: str = "survey") -> list[DirectedPass]:
    """Змейка по ячейке. Из 4 вариантов входа (первая/последняя линия, левый/правый конец)
    берём ближайший к текущей точке."""
    best: list[DirectedPass] = []
    best_d = math.inf
    for rev in (False, True):
        for first_fwd in (True, False):
            seq: list[DirectedPass] = []
            fwd = first_fwd
            for p in (reversed(cell) if rev else cell):
                seq.append(DirectedPass(p.a, p.b, kind) if fwd else DirectedPass(p.b, p.a, kind))
                fwd = not fwd
            d = math.dist(start, seq[0].a)
            if d < best_d:
                best, best_d = seq, d
    return best


def order_passes(passes: list[Pass], start: tuple[float, float], kind: str = "survey") -> list[DirectedPass]:
    """Порядок обхода: ячейки boustrophedon, внутри ячейки — змейка, следующая ячейка —
    ближайшая к концу предыдущей (жадно)."""
    if not passes:
        return []
    cells = boustrophedon_cells(passes)
    # возможные точки входа в ячейку: концы первого и последнего галса
    entries = [(c[0].a, c[0].b, c[-1].a, c[-1].b) for c in cells]
    route: list[DirectedPass] = []
    pos = start
    left = set(range(len(cells)))
    while left:
        best_i = min(left, key=lambda i: (min(math.dist(pos, e) for e in entries[i]), i))
        seq = _snake(cells[best_i], pos, kind)
        route += seq
        pos = seq[-1].b
        left.remove(best_i)
    return route


class SortieBuilder:
    def __init__(
        self,
        drone: DroneModel,
        speed: float,
        alt: float,
        wind: Wind,
        budget_s: float,
        router: Router | None = None,
    ):
        self.router = router
        self.blocked_turns = 0  # развороты, которые пришлось заменить обходом
        self.drone = drone
        self.speed = speed
        self.transit_speed = max(speed, drone.cruise_speed_ms)
        self.alt = alt
        self.wind = wind
        self.budget = budget_s

    # --- элементарные оценки -------------------------------------------------
    def fly_time(self, p: tuple[float, float], q: tuple[float, float], speed: float) -> float:
        d = math.dist(p, q)
        if d < 1e-6:
            return 0.0
        t = segment_time(d, speed, math.atan2(q[1] - p[1], q[0] - p[0]), self.wind)
        if t is None:
            # против сильного ветра линию не удержать — летим с минимально разумной скоростью
            t = d / max(0.5, speed - self.wind.speed_ms)
        return t

    def path(self, p: tuple[float, float], q: tuple[float, float]) -> list[tuple[float, float]]:
        return self.router.route(p, q) if self.router else [p, q]

    def path_time(self, pts: list[tuple[float, float]], speed: float) -> float:
        return sum(self.fly_time(a, b, speed) for a, b in zip(pts, pts[1:]))

    def approach(
        self, pos: tuple[float, float], heading: float | None, dp: "DirectedPass"
    ) -> tuple[str, list[tuple[float, float]], float, float]:
        """Подход к началу галса: транзит (после взлёта) или разворот. Если разворот задевает
        запретную зону, заменяем его путём по графу видимости."""
        if heading is None:
            pts = self.path(pos, dp.a)
            return "transit", pts, self.path_time(pts, self.transit_speed), _plen(pts)
        for mid, turn_len in turn_options(self.drone, pos, heading, dp.a, dp.heading):
            pts = [pos, *mid, dp.a]
            if self.router is None or self.router.polyline_free(pts):
                if self.drone.type == "fixed_wing":
                    return "turn", pts, turn_len / self.speed, turn_len
                return "turn", pts, self.fly_time(pos, dp.a, self.speed) + self.speed / MULTIROTOR_ACCEL, turn_len
        self.blocked_turns += 1
        pts = self.path(pos, dp.a)
        t = self.path_time(pts, self.transit_speed)
        # поправка на смену курса: полуокружность у самолёта, торможение/разгон у мультиротора
        t += math.pi * self.drone.turn_radius_m / self.speed if self.drone.type == "fixed_wing" else self.speed / MULTIROTOR_ACCEL
        return "transit", pts, t, _plen(pts)

    def divert_check(self, s: "Sortie", sites: dict[str, tuple[float, float]], step: float = 400.0) -> None:
        """Для точек вылета (с шагом step вдоль участков) — время до ближайшей площадки
        (база или резервная) с обходом зон, плюс посадка. Худшее значение — в s.max_divert_s."""
        worst, worst_site = 0.0, ""
        for leg in s.legs:
            if leg.kind in ("takeoff", "landing"):
                continue
            for a, b in zip(leg.points, leg.points[1:]):
                n = max(1, math.ceil(math.dist(a, b) / step))
                for k in range(n + 1):
                    p = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                    best, best_id = math.inf, ""
                    # маршрут строим только к двум ближайшим по прямой площадкам
                    near = sorted(sites.items(), key=lambda kv: math.dist(p, kv[1]))[:2]
                    for sid, q in near:
                        t = self.path_time(self.path(p, q), self.transit_speed)
                        if t < best:
                            best, best_id = t, sid
                    if best > worst:
                        worst, worst_site = best, best_id
        s.max_divert_s = worst + self.landing_s()
        s.divert_site = worst_site

    def takeoff_s(self) -> float:
        climb = self.alt / self.drone.climb_rate_ms
        return climb + (FIXED_WING_TAKEOFF_S if self.drone.type == "fixed_wing" else 0.0)

    def landing_s(self) -> float:
        if self.drone.type == "fixed_wing":
            return FIXED_WING_LANDING_S
        return self.alt / MULTIROTOR_DESCENT_MS

    def return_s(self, p: tuple[float, float], base: tuple[float, float]) -> float:
        return self.path_time(self.path(p, base), self.transit_speed) + self.landing_s()

    # --- сборка ---------------------------------------------------------------
    def build(
        self, route: list[DirectedPass], base: tuple[float, float], base_id: str
    ) -> list[Sortie]:
        sorties: list[Sortie] = []
        queue = list(route)
        while queue:
            s = Sortie(index=len(sorties), base=base, base_id=base_id)
            t = self.takeoff_s()
            s.legs.append(Leg("takeoff", [base, base], self.alt, t, 0.0))
            pos, heading = base, None
            added = 0
            while queue:
                dp = queue[0]
                app_kind, app_pts, app_t, app_len = self.approach(pos, heading, dp)
                pass_t = self.fly_time(dp.a, dp.b, self.speed)
                if t + app_t + pass_t + self.return_s(dp.b, base) <= self.budget:
                    s.legs.append(Leg(app_kind, app_pts, self.alt, app_t, app_len))
                    s.legs.append(Leg(dp.kind, [dp.a, dp.b], self.alt, pass_t, dp.length))
                    t += app_t + pass_t
                    pos, heading = dp.b, dp.heading
                    queue.pop(0)
                    added += 1
                    continue
                # не помещается целиком — пробуем часть галса
                frac = self._max_fraction(t + app_t, dp, base)
                if frac * dp.length >= 20.0:
                    cut = (dp.a[0] + frac * (dp.b[0] - dp.a[0]), dp.a[1] + frac * (dp.b[1] - dp.a[1]))
                    part_t = self.fly_time(dp.a, cut, self.speed)
                    s.legs.append(Leg(app_kind, app_pts, self.alt, app_t, app_len))
                    s.legs.append(Leg(dp.kind, [dp.a, cut], self.alt, part_t, math.dist(dp.a, cut)))
                    t += app_t + part_t
                    pos, heading = cut, dp.heading
                    queue[0] = DirectedPass(cut, dp.b, dp.kind)
                    added += 1
                break
            if added == 0:
                raise ValueError(
                    f"{self.drone.name}: не хватает заряда даже на участок галса — область слишком далеко от базы"
                )
            back = self.path(pos, base)
            back_t = self.path_time(back, self.transit_speed)
            s.legs.append(Leg("return", back, self.alt, back_t, _plen(back)))
            s.legs.append(Leg("landing", [base, base], self.alt, self.landing_s(), 0.0))
            sorties.append(s)
        return sorties

    def _max_fraction(self, t0: float, dp: DirectedPass, base: tuple[float, float]) -> float:
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = (lo + hi) / 2
            cut = (dp.a[0] + mid * (dp.b[0] - dp.a[0]), dp.a[1] + mid * (dp.b[1] - dp.a[1]))
            if t0 + self.fly_time(dp.a, cut, self.speed) + self.return_s(cut, base) <= self.budget:
                lo = mid
            else:
                hi = mid
        return lo


def schedule(sorties: list[Sortie], swap_s: float) -> float:
    """Проставляет время начала вылетов (последовательно, со сменой АКБ). Возвращает время
    окончания работ этим бортом."""
    t = 0.0
    for i, s in enumerate(sorties):
        if i:
            t += swap_s
        s.start_s = t
        t += s.duration_s
    return t


def _plen(pts: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
