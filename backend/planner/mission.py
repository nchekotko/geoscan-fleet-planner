"""Полётное задание одного борта в своей области: порядок галсов, развороты, транзиты,
нарезка на вылеты по заряду.

Нарезка идёт по принципу route-first / cluster-second (Beasley, 1983): сначала строится один
маршрут обхода всех галсов, затем он режется на вылеты так, чтобы каждый укладывался в бюджет
(база → точка входа → галсы → база). Разрезы ищутся точно, кратчайшим путём во вспомогательном
графе (процедура Split, Prins, 2004). Параллельно строится жадная нарезка, в которой галс,
не помещающийся целиком, делится; берётся лучший из двух вариантов.
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
# Интервал между стартами разных бортов с одной ВПП (допущения): катапульту нужно перезарядить
# и уложить следующий борт, мультиротору — освободить площадку.
LAUNCH_INTERVAL_S = {"fixed_wing": 300.0, "multirotor": 120.0}
FIXED_WING_LANDING_S = 120.0
MULTIROTOR_DESCENT_MS = 3.0
# Взлёт и посадка самолёта по ветру (руководство Геоскан 201: запуск и посадка строго против
# ветра, учёт сноса на парашюте). Разгон с катапульты, заход и парашют — допущения.
LAUNCH_RUN_M = 300.0          # прямолинейный участок после катапульты против ветра
FINAL_APPROACH_M = 500.0      # последний прямой участок захода против ветра
PARACHUTE_OPEN_AGL_M = 100.0  # высота раскрытия парашюта
PARACHUTE_SINK_MS = 5.0       # скорость снижения на парашюте
MIN_WIND_FOR_HEADING_MS = 1.0  # при более слабом ветре курс взлёта и посадки не важен


@dataclass
class Leg:
    """Участок маршрута. kind: takeoff | transit | survey | tie | turn | return | landing.

    speed_ms — скорость на участке: съёмочная на галсах и разворотах, транзитная на перелётах
    и возврате, вертикальная (набор высоты / снижение) на взлёте и посадке."""

    kind: str
    points: list[tuple[float, float]]
    alt_agl: float
    duration_s: float
    distance_m: float
    speed_ms: float = 0.0
    point_alts: list[float] | None = None  # высоты точек AGL, если они разные (взлёт, посадка)


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
        # единичный вектор «против ветра» — для катапульты и захода на посадку самолёта
        self.upwind: tuple[float, float] | None = None
        if drone.type == "fixed_wing" and wind.speed_ms >= MIN_WIND_FOR_HEADING_MS:
            wx, wy = wind.vector()
            n = math.hypot(wx, wy)
            self.upwind = (-wx / n, -wy / n)

    # --- взлёт и посадка ---------------------------------------------------------
    def _along(self, p: tuple[float, float], d: float) -> tuple[float, float]:
        return (p[0] + d * self.upwind[0], p[1] + d * self.upwind[1])

    def launch_point(self, base: tuple[float, float]) -> tuple[float, float]:
        """Конец разгона после катапульты: LAUNCH_RUN_M против ветра от ВПП."""
        return self._along(base, LAUNCH_RUN_M) if self.upwind else base

    def parachute_drift_m(self) -> float:
        """Снос на парашюте: ветер × время снижения с высоты раскрытия."""
        return self.wind.speed_ms * PARACHUTE_OPEN_AGL_M / PARACHUTE_SINK_MS

    def landing_entry(self, base: tuple[float, float]) -> tuple[float, float]:
        """Куда ведёт возврат: начало последнего прямого захода (самолёт при ветре) или ВПП."""
        if not self.upwind:
            return base
        return self._along(base, self.parachute_drift_m() - FINAL_APPROACH_M)

    def takeoff_leg(self, base: tuple[float, float]) -> Leg:
        if not self.upwind:
            return Leg("takeoff", [base, base], self.alt, self.takeoff_s(), 0.0, self.drone.climb_rate_ms)
        end = self.launch_point(base)
        gs = max(self.drone.cruise_speed_ms - self.wind.speed_ms, 5.0)
        h = min(self.alt, self.drone.climb_rate_ms * LAUNCH_RUN_M / gs)
        return Leg("takeoff", [base, end], self.alt, self.takeoff_s(), LAUNCH_RUN_M,
                   self.drone.climb_rate_ms, point_alts=[0.0, h])

    def landing_leg(self, base: tuple[float, float]) -> Leg:
        """Посадка. Самолёт при ветре: заход против ветра до точки раскрытия парашюта (с наветренной
        стороны на величину сноса), затем спуск на парашюте с дрейфом на ВПП."""
        if not self.upwind:
            return Leg("landing", [base, base], self.alt, self.landing_s(), 0.0, self.descent_ms())
        entry = self.landing_entry(base)
        chute = self._along(base, self.parachute_drift_m())
        return Leg("landing", [entry, chute, base], self.alt, self.landing_s(),
                   math.dist(entry, chute) + math.dist(chute, base), self.descent_ms(),
                   point_alts=[self.alt, min(PARACHUTE_OPEN_AGL_M, self.alt), 0.0])

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

    def descent_ms(self) -> float:
        """Средняя вертикальная скорость посадки (у самолёта — заход и спуск на парашюте)."""
        return self.alt / self.landing_s() if self.drone.type == "fixed_wing" else MULTIROTOR_DESCENT_MS

    def leg_speed(self, kind: str) -> float:
        return self.transit_speed if kind in ("transit", "return") else self.speed

    def return_s(self, p: tuple[float, float], base: tuple[float, float]) -> float:
        return self.path_time(self.path(p, self.landing_entry(base)), self.transit_speed) + self.landing_s()

    # --- сборка ---------------------------------------------------------------
    def build(
        self, route: list[DirectedPass], base: tuple[float, float], base_id: str
    ) -> list[Sortie]:
        """Нарезка маршрута на вылеты: точная (Split) и жадная, берётся та, у которой меньше
        время работы борта — сумма длительностей вылетов плюс смены АКБ между ними."""
        greedy = self._build_greedy(route, base, base_id)
        split = self._build_split(route, base, base_id)
        if split is not None and self._finish(split) < self._finish(greedy) - 1e-6:
            return split
        return greedy

    def _finish(self, sorties: list[Sortie]) -> float:
        swap = self.drone.swap_time_min * 60
        return sum(s.duration_s for s in sorties) + swap * max(0, len(sorties) - 1)

    def _build_split(
        self, route: list[DirectedPass], base: tuple[float, float], base_id: str
    ) -> list[Sortie] | None:
        """Процедура Split (Prins, 2004). Узлы — концы галсов 0..n, ребро i→j — вылет, который
        снимает галсы i..j-1 целиком: взлёт, подход к галсу i, галсы с разворотами, возврат
        с конца галса j-1, посадка. Ребро есть, только если вылет укладывается в бюджет; вес —
        длительность вылета плюс смена АКБ. Кратчайший путь 0→n даёт оптимальные разрезы при
        заданном порядке галсов. Галсы здесь не делятся: если какой-то галс не помещается даже
        в отдельный вылет, возвращается None, и остаётся жадная нарезка."""
        n = len(route)
        if n == 0:
            return None
        swap = self.drone.swap_time_min * 60
        take, land = self.takeoff_s(), self.landing_s()
        pass_t = [self.fly_time(dp.a, dp.b, self.speed) for dp in route]
        # разворот от конца галса k-1 к началу галса k внутри вылета
        inner = [None] + [self.approach(route[k - 1].b, route[k - 1].heading, route[k]) for k in range(1, n)]
        first: dict[int, tuple] = {}   # подход от базы к галсу i
        back: dict[int, tuple] = {}    # возврат с конца галса j на базу — по мере надобности
        best = [math.inf] * (n + 1)
        prev = [-1] * (n + 1)
        best[0] = -swap                # перед первым вылетом смены АКБ нет
        for i in range(n):
            if best[i] == math.inf:
                continue
            first[i] = self.approach(self.launch_point(base), None, route[i])
            t = take + first[i][2]
            for j in range(i, n):
                if j > i:
                    t += inner[j][2]
                t += pass_t[j]
                if t + land > self.budget:
                    break              # время только растёт — более длинные вылеты недопустимы
                if j not in back:
                    pts = self.path(route[j].b, self.landing_entry(base))
                    back[j] = (pts, self.path_time(pts, self.transit_speed))
                dur = t + back[j][1] + land
                if dur <= self.budget and best[i] + swap + dur < best[j + 1] - 1e-9:
                    best[j + 1], prev[j + 1] = best[i] + swap + dur, i
        if best[n] == math.inf:
            return None
        cuts: list[tuple[int, int]] = []
        j = n
        while j > 0:
            cuts.append((prev[j], j))
            j = prev[j]
        return [self._assemble(k, route[i:j], first[i], inner[i + 1:j], pass_t[i:j], back[j - 1], base, base_id)
                for k, (i, j) in enumerate(reversed(cuts))]

    def _assemble(
        self, index: int, passes: list[DirectedPass], first: tuple, turns: list[tuple],
        pass_t: list[float], back: tuple, base: tuple[float, float], base_id: str,
    ) -> Sortie:
        """Вылет из готовых кусков: взлёт, подход, галсы с разворотами, возврат, посадка."""
        s = Sortie(index=index, base=base, base_id=base_id)
        s.legs.append(self.takeoff_leg(base))
        for dp, app, t in zip(passes, [first, *turns], pass_t):
            app_kind, app_pts, app_t, app_len = app
            s.legs.append(Leg(app_kind, app_pts, self.alt, app_t, app_len, self.leg_speed(app_kind)))
            s.legs.append(Leg(dp.kind, [dp.a, dp.b], self.alt, t, dp.length, self.speed))
        back_pts, back_t = back
        s.legs.append(Leg("return", back_pts, self.alt, back_t, _plen(back_pts), self.transit_speed))
        s.legs.append(self.landing_leg(base))
        return s

    def _build_greedy(
        self, route: list[DirectedPass], base: tuple[float, float], base_id: str
    ) -> list[Sortie]:
        """Жадная нарезка: вылет набирается галсами, пока хватает бюджета; галс, который
        не помещается целиком, делится — часть в текущий вылет, остаток в следующий."""
        sorties: list[Sortie] = []
        queue = list(route)
        while queue:
            s = Sortie(index=len(sorties), base=base, base_id=base_id)
            t = self.takeoff_s()
            s.legs.append(self.takeoff_leg(base))
            pos, heading = self.launch_point(base), None
            added = 0
            while queue:
                dp = queue[0]
                app_kind, app_pts, app_t, app_len = self.approach(pos, heading, dp)
                pass_t = self.fly_time(dp.a, dp.b, self.speed)
                if t + app_t + pass_t + self.return_s(dp.b, base) <= self.budget:
                    s.legs.append(Leg(app_kind, app_pts, self.alt, app_t, app_len, self.leg_speed(app_kind)))
                    s.legs.append(Leg(dp.kind, [dp.a, dp.b], self.alt, pass_t, dp.length, self.speed))
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
                    s.legs.append(Leg(app_kind, app_pts, self.alt, app_t, app_len, self.leg_speed(app_kind)))
                    s.legs.append(Leg(dp.kind, [dp.a, cut], self.alt, part_t, math.dist(dp.a, cut), self.speed))
                    t += app_t + part_t
                    pos, heading = cut, dp.heading
                    queue[0] = DirectedPass(cut, dp.b, dp.kind)
                    added += 1
                break
            if added == 0:
                raise ValueError(
                    f"{self.drone.name}: не хватает заряда даже на участок галса — область слишком далеко от базы"
                )
            back = self.path(pos, self.landing_entry(base))
            back_t = self.path_time(back, self.transit_speed)
            s.legs.append(Leg("return", back, self.alt, back_t, _plen(back), self.transit_speed))
            s.legs.append(self.landing_leg(base))
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


def schedule(sorties: list[Sortie], swap_s: float, start_s: float = 0.0) -> float:
    """Проставляет время начала вылетов (последовательно, со сменой АКБ), первый — в start_s
    (очерёдность стартов на ВПП). Возвращает время окончания работ этим бортом."""
    t = start_s
    for i, s in enumerate(sorties):
        if i:
            t += swap_s
        s.start_s = t
        t += s.duration_s
    return t


def _plen(pts: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
