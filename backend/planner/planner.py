"""Оркестрация планирования для парка БВС.

1. Рабочая область = (область съёмки ∩ разрешённая зона) − buffer(NFZ).
2. Для каждого борта: выбор нагрузки, параметры съёмки, бюджет вылета; неподходящие исключаются.
3. Общее направление галсов, полосы с долями площади по производительности бортов.
4. Балансировка долей по фактическому времени (критерий «время работ»).
5. Для time_weight < 1 — локальный поиск по долям, минимизирующий
   J = w·T_max/T_ref + (1 − w)·ΣT/ΣT_ref.
6. Для time_weight = 1 — перебор порядка полос бортов одной базы и лексикографическая
   доводка (T_max, затем ΣT), включая ход «убрать борт» с большими накладными расходами.
7. Шаги 3–6 повторяются для нескольких направлений галсов (мультистарт), берётся лучший план.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from shapely.geometry import LineString, MultiPolygon, Point, Polygon, mapping, shape
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
import shapely
from shapely.errors import GEOSException
from shapely.validation import make_valid

from .avoid import Router
from .coverage import (
    best_direction,
    distinct_directions,
    ranked_directions,
    rectangle_axis,
    sweep_passes,
    wind_axes,
)
from .energy import check_wind, usable_flight_time_s
from .fleet import DroneModel, Payload, load_fleet
from .geo import LocalFrame
from .mission import Sortie, SortieBuilder, order_passes, schedule
from .terrain import TERRAIN_ATTRIBUTION, Terrain
from .turns import turn_overshoot
from .grid_partition import grid_partition
from .mission import FIXED_WING_LANDING_S, FIXED_WING_TAKEOFF_S, LAUNCH_INTERVAL_S, MULTIROTOR_DESCENT_MS
from .separation import H_MIN_M, closest_approach, track
from .partition import split_by_fractions, strip_axis_position
from .schemas import (
    DronePlanOut,
    ExcludedDrone,
    LegOut,
    PlanRequest,
    PlanResponse,
    SortieOut,
    Summary,
    TerrainInfo,
)
from .sensors import SurveyInfeasible, SurveyParams, survey_params
from .wind import Wind, ground_speed

# Минимальный вес времени работ в критерии «налёт» (w = 0): разрешает почти равные по налёту
# планы в пользу более быстрого.
MIN_TIME_WEIGHT = 0.02

# мультистарт по направлению галсов: число углов-кандидатов
MULTISTART_K = 5
MULTISTART_K_LARGE = 2
MULTISTART_LARGE_AREA_M2 = 20e6  # больше 20 км² — план строится секунды, углов меньше
MULTISTART_MIN_GAIN = 0.005  # другой угол берётся, если выигрыш по J больше 0,5 %
SCREEN_ITERS = 3  # итераций балансировки при отсеве угла
COVERAGE_TOL = 1e-5  # покрытие при смене угла не должно падать (допуск — доля площади)


class PlanningError(ValueError):
    pass


# запас радиуса действия на обходы NFZ, развороты и неточность прямой «база — точка»
REACH_MARGIN = 0.10
# «выгодный» радиус при делении сеткой: перелёт туда-обратно — не больше этой доли вылета
PROFITABLE_TRANSIT_SHARE = 0.5
# шаг эшелонов перелёта: k-й борт летает к области и обратно на k·шаг выше высоты съёмки
TRANSIT_LEVEL_STEP_M = 20.0
# перебор порядка бортов одной ВПП: при большом парке — только первые обмены соседей
MAX_ORDER_SEARCH_DRONES = 6
MAX_ORDERS_LARGE_FLEET = 4
TRANSIT_KINDS = ("transit", "return")


def round_trip_s_per_m(v: float, wind: Wind, steps: int = 72) -> float:
    """Время полёта туда-обратно на 1 м удаления при худшем направлении:
    max_θ [1/gs(V, θ) + 1/gs(V, θ + π)] по треугольнику скоростей (wind.ground_speed)."""
    def inv_gs(theta: float) -> float:
        gs = ground_speed(v, theta, wind)
        # как в SortieBuilder.fly_time: линию не удержать — минимально разумная скорость
        return 1.0 / (gs if gs is not None else max(0.5, v - wind.speed_ms))

    return max(inv_gs(2 * math.pi * k / steps) + inv_gs(2 * math.pi * k / steps + math.pi) for k in range(steps))


def energy_range_m(drone: DroneModel, params: SurveyParams, budget_s: float, wind: Wind) -> float:
    """Предельное удаление от базы, на которое борт долетает и возвращается без съёмки.
    Скорость транзита та же, что у SortieBuilder (не ниже крейсерской); радиус — лишь
    эвристика разбиения, точную проверку заряда делает SortieBuilder."""
    fw = drone.type == "fixed_wing"
    t_ops = params.altitude_agl_m / drone.climb_rate_ms + (
        FIXED_WING_TAKEOFF_S + FIXED_WING_LANDING_S if fw else params.altitude_agl_m / MULTIROTOR_DESCENT_MS
    )
    v = max(params.speed_ms, drone.cruise_speed_ms)
    return max(budget_s - t_ops, 0.0) / round_trip_s_per_m(v, wind)


UNCOVERED_TOL_M2 = 100.0  # допуск на неснятую площадь при сравнении планов (шум геометрии)
UNCOVERED_PENALTY = 10.0  # вес доли неснятой площади в целевой функции локального поиска


@dataclass
class Candidate:
    instance_id: str
    drone: DroneModel
    payload: Payload
    params: SurveyParams
    budget_s: float
    base_id: str | None
    productivity: float  # м²/с с учётом смены АКБ
    reach_m: float = math.inf  # радиус действия от базы: заряд туда-обратно и радиоканал
    work_reach_m: float = math.inf  # «выгодный» радиус: перелёт не больше половины вылета
    wind: Wind | None = None  # ветер на рабочей высоте борта


@dataclass
class DroneResult:
    cand: Candidate
    region: BaseGeometry
    base_id: str
    sorties: list[Sortie]
    finish_s: float

    @property
    def flight_s(self) -> float:
        return sum(s.duration_s for s in self.sorties)


@dataclass
class Evaluation:
    fractions: list[float]
    results: list[DroneResult]
    leftover_m2: float = 0.0  # не снято: самолёту не хватает места для разворота, мультиротора нет
    unreachable_m2: float = 0.0  # не снято: вне радиуса действия всех бортов

    @property
    def makespan(self) -> float:
        return max((r.finish_s for r in self.results), default=0.0)

    @property
    def total(self) -> float:
        return sum(r.flight_s for r in self.results)

    @property
    def uncovered_m2(self) -> float:
        """Площадь, которую план оставляет неснятой (у NFZ и вне радиуса действия)."""
        return self.leftover_m2 + self.unreachable_m2


@dataclass
class _Run:
    """Готовый план при одном направлении галсов (для мультистарта по углу)."""

    angle: float
    ev: Evaluation
    mode: str
    warnings: list[str]


def _polys(g: BaseGeometry) -> BaseGeometry:
    """Только полигональная часть геометрии, приведённая к валидному виду."""
    if not g.is_valid:
        g = make_valid(g)
    if isinstance(g, (Polygon, MultiPolygon)):
        return g
    ps = [x for x in getattr(g, "geoms", []) if isinstance(x, Polygon)]
    return MultiPolygon(ps) if ps else Polygon()


class Planner:
    def __init__(self, req: PlanRequest):
        self.req = req
        self.fleet = load_fleet()
        self.warnings: list[str] = []
        self.excluded: list[ExcludedDrone] = []

        survey_wgs = shape(req.survey_area)
        self.frame = LocalFrame.around(survey_wgs)
        f = self.frame
        area = f.to_local(survey_wgs)
        if req.allowed_area:
            allowed = f.to_local(shape(req.allowed_area))
            if not area.within(allowed):
                self.warnings.append("часть области съёмки вне разрешённой зоны и исключена")
            area = area.intersection(allowed)
            self.allowed = allowed
        else:
            self.allowed = None
        self.nfz = [f.to_local(shape(z)) for z in req.no_fly_zones]
        if self.nfz:
            blocked = unary_union([z.buffer(req.nfz_buffer_m) for z in self.nfz])
            area = area.difference(blocked)
        self.area = _polys(area.buffer(0))
        if self.area.is_empty or self.area.area < 100:
            raise PlanningError("рабочая область пуста после вычета запретных зон")
        self.mode = "strips"
        self.router = Router(self.nfz, req.nfz_buffer_m, self.allowed)
        self.bases = {b.id: f.lonlat_to_xy(b.lon, b.lat) for b in req.bases}
        self.reserve_sites = {}
        for r in req.reserve_sites:
            xy = f.lonlat_to_xy(r.lon, r.lat)
            pt = Point(xy)
            if any(z.contains(pt) for z in self.nfz):
                self.warnings.append(f"резервная площадка {r.id} внутри запретной зоны — не используется")
            elif self.allowed is not None and not self.allowed.buffer(1.0).contains(pt):
                self.warnings.append(f"резервная площадка {r.id} вне разрешённой зоны — не используется")
            else:
                self.reserve_sites[r.id] = xy
        if not self.bases:
            raise PlanningError("не задано ни одного взлётно-посадочного пункта")
        self.candidates = self._candidates()
        if not self.candidates:
            reasons = "; ".join(f"{e.drone_id} — {e.reason}" for e in self.excluded)
            raise PlanningError(
                "ни один борт не может выполнить съёмку этого типа в заданных условиях"
                + (f": {reasons}" if reasons else "")
            )

    # ------------------------------------------------------------------ борта
    def _candidates(self) -> list[Candidate]:
        out: list[Candidate] = []
        req = self.req
        for inst in req.drones:
            drone = self.fleet.drones.get(inst.model)
            if drone is None:
                self.excluded.append(ExcludedDrone(drone_id=inst.id, reason=f"неизвестная модель {inst.model}"))
                continue
            if inst.base_id and inst.base_id not in self.bases:
                self.excluded.append(ExcludedDrone(drone_id=inst.id, reason=f"неизвестная база {inst.base_id}"))
                continue
            options = [inst.payload] if inst.payload else drone.payloads
            payload = next(
                (
                    self.fleet.payloads[p]
                    for p in options
                    if p in drone.payloads and req.survey_type in self.fleet.payloads[p].survey_types
                ),
                None,
            )
            if payload is None:
                self.excluded.append(
                    ExcludedDrone(drone_id=inst.id, reason=f"{drone.name}: нет нагрузки для съёмки «{req.survey_type}»")
                )
                continue
            try:
                # высота съёмки от скорости не зависит — по ней пересчитываем ветер на рабочую высоту
                h = survey_params(drone, payload, req.requirements).altitude_agl_m
                wind = req.wind.at(h)
                # против ветра нужна путевая скорость ≥ ~2 м/с
                params = survey_params(drone, payload, req.requirements, min_speed=wind.speed_ms + 2.0)
            except SurveyInfeasible as e:
                self.excluded.append(ExcludedDrone(drone_id=inst.id, reason=str(e)))
                continue
            at_h = f" (на высоте {h:.0f} м)" if req.wind.ref_height_m is not None else ""
            wind_problem = check_wind(drone, wind)
            if wind_problem:
                self.excluded.append(ExcludedDrone(drone_id=inst.id, reason=wind_problem + at_h))
                continue
            # против ветра и поперёк него линию пути держать можно только при V > W
            if params.speed_ms <= wind.speed_ms + 0.5:
                self.excluded.append(ExcludedDrone(
                    drone_id=inst.id,
                    reason=f"{drone.name}: скорость съёмки {params.speed_ms:.1f} м/с не выше ветра "
                           f"{wind.speed_ms:.1f} м/с{at_h} (увеличьте GSD или снизьте перекрытие)",
                ))
                continue
            if params.photo_base_m and payload.min_trigger_interval_s:
                # на попутном галсе путевая скорость больше — кадры чаще; камера может не успеть
                v_ok = params.photo_base_m / payload.min_trigger_interval_s - wind.speed_ms
                if (drone.type == "multirotor" and v_ok < params.speed_ms and v_ok >= wind.speed_ms + 2.0):
                    # мультиротор может лететь медленнее: камера успевает и по ветру, линия пути держится
                    params = params.model_copy(update={
                        "speed_ms": v_ok,
                        "trigger_interval_s": params.photo_base_m / v_ok,
                        "notes": params.notes + [f"скорость снижена до {v_ok:.1f} м/с: камера успевает и на попутном галсе"],
                    })
                gs_tail = params.speed_ms + wind.speed_ms
                if params.photo_base_m / gs_tail < payload.min_trigger_interval_s - 1e-9:
                    along = params.photo_base_m / (1 - req.requirements.front_overlap)
                    real = max(0.0, 1 - payload.min_trigger_interval_s * gs_tail / along)
                    self.warnings.append(
                        f"{inst.id}: на попутном галсе камера не успевает (интервал "
                        f"{params.photo_base_m / gs_tail:.2f} с при минимуме {payload.min_trigger_interval_s:.1f} с) — "
                        f"продольное перекрытие упадёт до {100 * real:.0f} % вместо "
                        f"{100 * req.requirements.front_overlap:.0f} %; увеличьте GSD или снизьте перекрытие"
                    )
            budget = usable_flight_time_s(drone, wind, req.reserve, payload.endurance_factor)
            overhead = params.altitude_agl_m / drone.climb_rate_ms * 2 + (180 if drone.type == "fixed_wing" else 0)
            availability = max(budget - overhead, 1.0) / (budget + drone.swap_time_min * 60)
            productivity = params.speed_ms * params.line_spacing_m * availability
            # радиус действия: туда-обратно при худшем направлении ветра, с запасом, и радиоканал
            range_m = energy_range_m(drone, params, budget, wind)
            reach = min(range_m * (1 - REACH_MARGIN), drone.radio_range_km * 1000)
            # борт, который не долетает ни до одной точки области, работать не может
            gap = self.area.distance(Point(self.bases[inst.base_id or self._default_base(self.area)]))
            if gap >= reach:
                self.excluded.append(ExcludedDrone(
                    drone_id=inst.id,
                    reason=f"{drone.name}: не долетает до области: ближайшая точка {gap / 1000:.1f} км, "
                           f"радиус действия {reach / 1000:.1f} км",
                ))
                continue
            out.append(Candidate(inst.id, drone, payload, params, budget, inst.base_id, productivity, reach,
                                 min(reach, range_m * PROFITABLE_TRANSIT_SHARE), wind))
        return out

    # ------------------------------------------------------------- оценка
    def _reaches(self, geom: BaseGeometry, cand: Candidate) -> bool:
        """Единая проверка досягаемости участка: вся геометрия в радиусе действия борта."""
        return self._far_point(geom, cand) <= cand.reach_m

    def _default_base(self, region: BaseGeometry) -> str:
        """Ближайшая к центру участка база (для пустого участка — к центру всей области)."""
        c = (region if not region.is_empty else self.area).centroid
        return min(self.bases, key=lambda b: math.dist(self.bases[b], (c.x, c.y)))

    def _reassign_turn_gaps(
        self, cands: list[Candidate], regions: list[BaseGeometry], n_active: int | None = None
    ) -> tuple[list[BaseGeometry], float]:
        """Полосы у NFZ, где самолёт не может развернуться, отдаём ближайшему мультиротору.

        Первые n_active бортов работают по плану, остальные (с пустыми участками) — резерв:
        кусок уходит резервному мультиротору, только если ни один работающий до него
        не долетает. Иначе при весе «налёт» = 1 весь план достаётся самолёту и кольца
        у NFZ остаются неснятыми."""
        n_active = len(cands) if n_active is None else n_active
        multi = [i for i, c in enumerate(cands) if c.drone.type == "multirotor"]
        regions = list(regions)
        leftover = 0.0
        for i, c in enumerate(cands):
            if c.drone.type != "fixed_wing" or regions[i].is_empty:
                continue
            room = self._turn_room(regions[i], c)
            gap = _polys(regions[i].difference(room))
            if gap.is_empty or gap.area < 1.0:
                continue
            regions[i] = room
            if not multi:
                leftover += gap.area
                continue
            for piece in getattr(gap, "geoms", [gap]):
                if piece.area < 1.0:
                    continue
                # только мультироторы, которые долетают до всего куска
                able = [k for k in multi if self._reaches(piece, cands[k])]
                if not able:
                    leftover += piece.area
                    continue
                working = [k for k in able if k < n_active or not regions[k].is_empty]
                able = working or able
                j = min(able, key=lambda k: self._gap_cost(cands[k], regions[k], piece))
                regions[j] = _polys(unary_union([regions[j], piece]))
        return regions, leftover

    def _gap_cost(self, cand: Candidate, region: BaseGeometry, piece: BaseGeometry) -> float:
        """Оценка времени, к которому мультиротор закончит, если кусок у NFZ отдать ему:
        своя работа плюс съёмка куска плюс перелёты к нему. Без оценки загрузки кольца
        у NFZ уходили ближайшему борту, и один борт с далёкой базой затягивал все работы."""
        speed = max(cand.params.speed_ms, 1.0)
        prod = max(cand.productivity, 1e-6)
        # перелёты: туда-обратно до дальней точки куска, по числу вылетов на него
        trips = max(math.ceil(piece.area / prod / max(cand.budget_s, 1.0)), 1)
        transit = 2 * trips * self._far_point(piece, cand) / speed
        return (region.area + piece.area) / prod + transit

    def _far_point(self, geom: BaseGeometry, cand: Candidate) -> float:
        """Расстояние от базы борта до самой дальней точки геометрии."""
        b = self.bases[cand.base_id or self._default_base(self.area)]
        return max(
            (math.dist(b, c) for g in getattr(geom, "geoms", [geom]) for c in g.exterior.coords),
            default=0.0,
        )

    def _turn_room(self, region: BaseGeometry, cand: Candidate) -> BaseGeometry:
        """Самолёту нужен запас для разворота за концом галса: отступаем от NFZ и от границы
        разрешённой зоны на вынос петли разворота."""
        over = turn_overshoot(cand.drone, cand.params.line_spacing_m)
        if over <= 0:
            return region
        r = region
        if self.nfz:
            r = r.difference(unary_union([z.buffer(self.req.nfz_buffer_m + over) for z in self.nfz]))
        if self.allowed is not None:
            r = r.intersection(self.allowed.buffer(-over))
        return _polys(r)

    def evaluate(self, cands: list[Candidate], fractions: list[float], angle: float) -> Evaluation:
        active = [(c, f) for c, f in zip(cands, fractions) if f > 1e-6]
        unreachable = 0.0
        if self.mode == "grid":
            regions, rest = grid_partition(
                self.area, angle, [f for _, f in active],
                [self.bases[c.base_id or self._default_base(self.area)] for c, _ in active],
                [c.reach_m for c, _ in active],
                work_reach=[c.work_reach_m for c, _ in active],
            )
            unreachable = rest.area
        else:
            regions = split_by_fractions(self.area, angle, [f for _, f in active])
            for (c, _), r in zip(active, regions):
                if not r.is_empty and not self._reaches(r, c):
                    # полоса за радиусом действия — переходим к разбиению с учётом дальности
                    raise ValueError(f"{c.instance_id}: полоса за пределами радиуса действия")
        # резерв для полос у NFZ — мультироторы с нулевой долей
        spare = [c for c, f in zip(cands, fractions) if f <= 1e-6 and c.drone.type == "multirotor"]
        workers = [c for c, _ in active] + spare
        regions, leftover = self._reassign_turn_gaps(
            workers, list(regions) + [Polygon()] * len(spare), n_active=len(active)
        )
        results: list[DroneResult] = []
        work = [(cand, region) for k, (cand, region) in enumerate(zip(workers, regions))
                if not (k >= len(active) and region.is_empty)]
        built: list[tuple[Candidate, BaseGeometry, str, list, float]] = []
        for cand, region in work:
            base_id = cand.base_id or self._default_base(region)
            base = self.bases[base_id]
            p = cand.params
            passes = sweep_passes(region, angle, p.line_spacing_m) if not region.is_empty else []
            route = order_passes(passes, base)
            if p.tie_line_spacing_m and not region.is_empty:
                # секущие маршруты поперёк основных галсов (геофизика)
                ties = sweep_passes(region, angle + math.pi / 2, p.tie_line_spacing_m)
                route += order_passes(ties, route[-1].b if route else base, kind="tie")
            builder = SortieBuilder(cand.drone, p.speed_ms, p.altitude_agl_m, cand.wind or self.req.wind, cand.budget_s, self.router)
            sorties = builder.build(route, base, base_id)
            # длительность работы борта без ожидания очереди на старт
            built.append((cand, region, base_id, sorties, schedule(sorties, cand.drone.swap_time_min * 60)))
        # Очередь стартов на каждой ВПП. Борт занимает полосу на LAUNCH_INTERVAL_S, а дальше
        # работает сам: это задача 1||Lmax, где «хвост» работы q = длительность − интервал.
        # Её оптимально решает правило Джексона — первым стартует борт с самым длинным хвостом
        # (Jackson, 1955). Порядок результатов при этом не меняется: балансировка сопоставляет
        # его с долями.
        launch: dict[str, float] = {}
        next_launch: dict[str, float] = {}
        for cand, _, base_id, _, dur in sorted(
            built, key=lambda b: -(b[4] - LAUNCH_INTERVAL_S[b[0].drone.type])
        ):
            launch[cand.instance_id] = next_launch.get(base_id, 0.0)
            next_launch[base_id] = launch[cand.instance_id] + LAUNCH_INTERVAL_S[cand.drone.type]
        for cand, region, base_id, sorties, _ in built:
            finish = schedule(sorties, cand.drone.swap_time_min * 60, launch[cand.instance_id])
            results.append(DroneResult(cand, region, base_id, sorties, finish))
        return Evaluation(fractions=list(fractions), results=results, leftover_m2=leftover, unreachable_m2=unreachable)

    # ---------------------------------------------------------- оптимизация
    def solve(self) -> tuple[Evaluation, float]:
        """Мультистарт по направлению галсов. Оценка направления по одному борту
        (best_direction) не видит разбиения на полосы, перелётов и деления на вылеты, поэтому
        углы сравниваются по планам. Опорный план — полный, с лучшим по оценке углом ведущего
        борта. Остальные углы проходят дешёвый отсев (несколько итераций балансировки, при
        w < 1 — ещё планы «всё одному борту»). Быстрый план — допустимый, полный план при
        том же угле не хуже него, поэтому угол, чей быстрый план уже лучше опорного больше чем
        на MULTISTART_MIN_GAIN, достраивается полностью и заменяет опорный, если не уменьшает
        покрытие."""
        angles = self.directions()
        ref: _Run | None = None
        error: ValueError | None = None
        while angles and ref is None:
            try:
                ref = self._full(angles.pop(0))
            except ValueError as e:
                error = error or e
        if ref is None:
            raise error or PlanningError("не удалось построить план ни для одного направления галсов")
        for alt in self._promising(ref, angles):
            try:
                run = self._full(alt)
            except ValueError:
                continue
            if self._objective_vs(run, ref) < self._objective_vs(ref, ref) and self._covers(run, ref):
                ref = run
                break
        self.mode = ref.mode
        self.warnings.extend(ref.warnings)
        return ref.ev, ref.angle

    def _full(self, angle: float) -> _Run:
        """Полный план при заданном угле; режим разбиения и предупреждения — в результате."""
        self.mode = "strips"
        n_warn = len(self.warnings)
        try:
            ev = self._solve_at(angle)
        finally:
            added = self.warnings[n_warn:]
            del self.warnings[n_warn:]
        return _Run(angle, ev, self.mode, added)

    def _objective_vs(self, e: Evaluation | _Run, ref: _Run) -> float:
        """Целевая функция, нормированная на опорный план."""
        ev = e.ev if isinstance(e, _Run) else e
        return self._objective(ev, ref.ev.makespan, ref.ev.total, self.req.time_weight, self.req.makespan_cap_s)

    def _covers(self, run: _Run, ref: _Run) -> bool:
        """Покрытие планом run не меньше, чем опорным (с допуском COVERAGE_TOL)."""
        return self._coverage(run.ev.results) >= self._coverage(ref.ev.results) - COVERAGE_TOL * self.area.area

    def _promising(self, ref: _Run, angles: list[float]) -> list[float]:
        """Углы, чей быстрый план лучше опорного больше чем на MULTISTART_MIN_GAIN,
        от лучшего к худшему."""
        limit = self._objective_vs(ref, ref) * (1 - MULTISTART_MIN_GAIN)
        # если опорному плану не хватило полос (радиус действия), сразу берём сетку
        modes = ("grid",) if ref.mode == "grid" else ("strips", "grid")
        scored = []
        for a in angles:
            evs = self._screen(a, modes)
            if evs:
                j = min(self._objective_vs(e, ref) for e in evs)
                if j < limit:
                    scored.append((j, a))
        return [a for _, a in sorted(scored)]

    def _screen(self, angle: float, modes: tuple[str, ...] = ("strips", "grid")) -> list[Evaluation]:
        """Быстрые планы для отсева угла: балансировка с малым числом итераций; при w < 1
        или ε-ограничении — ещё планы, где вся работа у одного борта."""
        cands = self._order(angle)
        fr = [c.productivity for c in cands]
        out: list[Evaluation] = []
        for mode in modes:
            self.mode = mode
            try:
                out.append(self._balance_from(cands, fr, angle, SCREEN_ITERS))
                break
            except (ValueError, GEOSException):
                continue
        if not out:
            return []
        if (self.req.time_weight < 1.0 or self.req.makespan_cap_s is not None) and len(cands) > 1:
            for i in range(len(cands)):
                try:
                    e = self.evaluate(cands, [1.0 if k == i else 0.0 for k in range(len(cands))], angle)
                except (ValueError, GEOSException):
                    continue
                if e.unreachable_m2 <= out[0].unreachable_m2 + 1.0:
                    out.append(e)
        return out

    @staticmethod
    def _objective(e: Evaluation, t_ref: float, s_ref: float, w: float, cap: float | None) -> float:
        """J = w·T_max/T_ref + (1 − w)·ΣT/ΣT_ref; при ε-ограничении — min ΣT со штрафом за T_max > ε.
        При w = 0 время работ всё же входит с весом MIN_TIME_WEIGHT: иначе план с налётом меньше
        на 1 % может оказаться вдвое дольше (всё — одному борту)."""
        if cap is not None:
            w = 0.0
        else:
            w = max(w, MIN_TIME_WEIGHT)
        j = w * e.makespan / t_ref + (1 - w) * e.total / s_ref
        if cap is not None and e.makespan > cap:
            j += 10.0 * (e.makespan - cap) / t_ref  # штраф за нарушение ε-ограничения
        return j

    def directions(self) -> list[float]:
        """Углы галсов для мультистарта, в порядке приоритета: по два лучших по оценке времени
        съёмки для каждого набора параметров бортов (первым — ведущий борт), вдоль и поперёк
        ветра, длинная ось минимального прямоугольника. Близкие (до 5°) углы не повторяются.
        Для больших областей план строится долго — только два лучших угла ведущего борта."""
        lead = max(self.candidates, key=lambda c: c.productivity)
        large = self.area.area > MULTISTART_LARGE_AREA_M2
        specs: dict[tuple, tuple[float, DroneModel, float]] = {}
        for c in [lead] + ([] if large else [c for c in self.candidates if c is not lead]):
            key = (c.drone.id, round(c.params.line_spacing_m, 1), round(c.params.speed_ms, 2))
            specs.setdefault(key, (c.params.line_spacing_m, c.drone, c.params.speed_ms))
        ranked = ranked_directions(self.area, list(specs.values()), lead.wind or self.req.wind)
        if not ranked[0]:
            raise PlanningError(f"{lead.drone.name}: ветер не позволяет выполнить съёмку ни в одном направлении")
        angles: list[float] = []
        for rk in ranked:
            angles += distinct_directions([a for a, _ in rk], 2)
        if not large:
            angles += wind_axes(self.req.wind) + rectangle_axis(self.area)
        return distinct_directions(angles, MULTISTART_K_LARGE if large else MULTISTART_K)

    def _solve_at(self, angle: float) -> Evaluation:
        """План при заданном направлении галсов."""
        cands = self._order(angle)
        fr = [c.productivity for c in cands]
        # Полосы дают лучшую геометрию. Если какой-то борт не достаёт до своей полосы,
        # переходим к разбиению сеткой с учётом радиуса действия.
        try:
            best = self._balance(cands, fr, angle)
        except ValueError:
            self.mode = "grid"
            self.warnings.append(
                "область больше радиуса действия части бортов — использовано разбиение с учётом дальности"
            )
            best = self._balance(cands, fr, angle)
        w = self.req.time_weight
        cap = self.req.makespan_cap_s
        if cap is not None and len(cands) > 1:
            best = self._local_search(cands, best, angle, 0.0, cap=cap)
        elif w < 1.0 and len(cands) > 1:
            best = self._local_search(cands, best, angle, w)
        elif len(cands) > 1 and self.mode != "grid":
            # w = 1: сначала порядок бортов одной базы, затем доводка долей.
            # В сеточном режиме участки растут от баз и от порядка не зависят, а одна оценка
            # стоит ~0,3 с; на large_mixed_fleet доводка ничего не дала — там её не делаем.
            cands, best = self._order_search(cands, best, angle)
        return best

    def prepare(self) -> tuple[list[Candidate], float]:
        """Направление галсов по оценке для самого производительного борта и порядок бортов
        (одностартовый вариант, используется бенчмарком для простых baseline)."""
        lead = max(self.candidates, key=lambda c: c.productivity)
        angle, _, _ = best_direction(self.area, lead.params.line_spacing_m, lead.drone, lead.params.speed_ms,
                                     lead.wind or self.req.wind)
        return self._order(angle), angle

    def _order(self, angle: float) -> list[Candidate]:
        """Порядок полос ↔ положение баз вдоль оси, перпендикулярной галсам."""
        def axis_key(c: Candidate) -> float:
            b = self.bases[c.base_id or self._default_base(self.area)]
            return strip_axis_position(b, angle)

        return sorted(self.candidates, key=axis_key)

    def _base_of(self, c: Candidate) -> str:
        return c.base_id or self._default_base(self.area)

    def _orders(self, cands: list[Candidate], limit: int = 24) -> list[list[Candidate]]:
        """Другие порядки полос, не меняющие положения баз вдоль оси: перестановки внутри
        групп бортов одной базы. Одинаковые борта (модель + нагрузка) не различаются.
        Если перестановок больше limit — только обмены соседей внутри группы (2-opt)."""
        groups: list[list[Candidate]] = []
        for c in cands:
            if groups and self._base_of(groups[-1][-1]) == self._base_of(c):
                groups[-1].append(c)
            else:
                groups.append([c])
        total = math.prod(math.factorial(len(g)) for g in groups)
        if total <= limit:
            variants = [list(itertools.permutations(g)) for g in groups]
            orders = [[c for g in combo for c in g] for combo in itertools.product(*variants)]
        else:
            orders = []
            for k in range(len(cands) - 1):
                if self._base_of(cands[k]) == self._base_of(cands[k + 1]):
                    o = list(cands)
                    o[k], o[k + 1] = o[k + 1], o[k]
                    orders.append(o)

        def key(order: list[Candidate]) -> tuple:
            return tuple((c.drone.id, c.payload.id) for c in order)

        seen = {key(cands)}
        out = []
        for o in orders:
            if key(o) not in seen:
                seen.add(key(o))
                out.append(o)
        return out

    @staticmethod
    def _lexi_better(a: Evaluation, b: Evaluation, tol: float = 0.005) -> bool:
        """Лексикографическое сравнение для w = 1: главное — T_max, налёт — вторичный критерий.
        План a лучше b, если T_max меньше более чем на tol (0,5 %), либо a доминирует b по Парето
        (T_max не больше и ΣT не больше, хотя бы одно строго). Допуск не даёт выиграть секунды
        T_max ценой минут налёта: такая разница меньше точности модели времени."""
        if a.makespan < b.makespan * (1 - tol):
            return True
        return (a.makespan <= b.makespan and a.total <= b.total
                and (a.makespan < b.makespan - 1.0 or a.total < b.total - 1.0))

    def _order_search(
        self, cands: list[Candidate], best: Evaluation, angle: float
    ) -> tuple[list[Candidate], Evaluation]:
        """Перебор порядка полос внутри групп бортов одной базы (только полосное разбиение):
        от порядка зависит, чья полоса ближе к базе, а значит и перелёты. Каждый порядок
        проходит ту же балансировку и доводку, что и исходный: после точной нарезки (Split)
        время борта меняется ступенями, и одной балансировки для сравнения порядков мало."""
        start = best
        best = self._local_search(cands, best, angle, 1.0, quick=True)
        orders = self._orders(cands)
        if len(cands) > MAX_ORDER_SEARCH_DRONES:
            orders = orders[:MAX_ORDERS_LARGE_FLEET]  # большой парк: только несколько обменов
        for order in orders:
            try:
                e = self._balance_from(order, [c.productivity for c in order], angle, 12)
                e = self._local_search(order, e, angle, 1.0, quick=True)
            except (ValueError, GEOSException):
                continue
            if e.unreachable_m2 > best.unreachable_m2 + 1.0 or e.leftover_m2 > best.leftover_m2 + 1.0:
                continue
            if self._lexi_better(e, best):
                cands, best, start = order, e, e
        # широкий поиск — только для лучшего порядка
        return cands, self._local_search(cands, start if start is not best else best, angle, 1.0)

    def _balance(self, cands: list[Candidate], fr: list[float], angle: float, iters: int = 12) -> Evaluation:
        """Выравнивание времени окончания работ: доли корректируются по фактическому времени.
        Мультистарт: доли по производительности и равные доли, берётся лучший результат."""
        results = []
        for k, start in enumerate((fr, [1.0] * len(fr))):
            # сетка дорогая: второй старт (равные доли) — только одна оценка, без итераций
            n_it = 0 if (self.mode == "grid" and k == 1) else iters
            try:
                results.append(self._balance_from(cands, start, angle, n_it))
            except ValueError:
                if not results and start is not fr:
                    raise
        if not results:
            raise ValueError("не удалось построить план ни из одной стартовой точки")
        # покрытие важнее времени: сначала планы без лишней неснятой площади
        min_unc = min(e.uncovered_m2 for e in results)
        return min(results, key=lambda e: (e.uncovered_m2 > min_unc + UNCOVERED_TOL_M2, e.makespan))

    def _balance_from(self, cands: list[Candidate], fr: list[float], angle: float, iters: int) -> Evaluation:
        best = self.evaluate(cands, fr, angle)
        cur = best
        for it in range(iters):
            if self.mode == "grid" and it >= 4:
                break  # сеточное разбиение дороже, хватает нескольких итераций
            times = [r.finish_s for r in cur.results]
            if not times or len(times) != len(cur.fractions):
                break  # есть борта без участка — доли и времена не сопоставить
            mean = sum(times) / len(times)
            if max(times) - min(times) < 0.02 * mean:
                break
            fr = [f * (mean / max(t, 1.0)) ** 0.8 for f, t in zip(cur.fractions, times)]
            s = sum(fr)
            if s <= 0:
                break
            fr = [f / s for f in fr]
            try:
                cur = self.evaluate(cands, fr, angle)
            except ValueError:
                break
            if cur.makespan < best.makespan and cur.uncovered_m2 <= best.uncovered_m2 + UNCOVERED_TOL_M2:
                best = cur
        return best

    def _local_search(
        self, cands: list[Candidate], start: Evaluation, angle: float, w: float, cap: float | None = None,
        quick: bool = False,
    ) -> Evaluation:
        """quick — короткий проход (шаг 0,2, до 12 оценок) для сравнения вариантов между собой."""
        t_ref, s_ref = start.makespan, start.total
        area = self.area.area
        lexi = w >= 1.0 and cap is None

        def score(e: Evaluation) -> float:
            # полнота съёмки — жёсткое требование: доля неснятой площади штрафуется сильнее,
            # чем любое реальное изменение времени (метод штрафных функций, Coello, 2002)
            return self._objective(e, t_ref, s_ref, w, cap) + UNCOVERED_PENALTY * e.uncovered_m2 / area

        def loses_coverage(a: Evaluation, b: Evaluation) -> bool:
            return a.uncovered_m2 > b.uncovered_m2 + UNCOVERED_TOL_M2

        def better(a: Evaluation, b: Evaluation) -> bool:
            # полнота съёмки сравнивается первой, затем критерий
            if loses_coverage(a, b):
                return False
            if lexi:
                return loses_coverage(b, a) or self._lexi_better(a, b)
            return score(a) < score(b) - 1e-6

        best = start
        if lexi and not quick:
            # ход «убрать борт»: борт, который в основном летает к области и обратно
            e = self._drop_overhead_drone(cands, best, angle, better)
            if e is not None and better(e, best):
                best = e
        if not quick:
            # дополнительные стартовые точки: вся работа одному борту. И при w = 1 это бывает
            # быстрее: у остальных далёкая база или ждать очереди на старт дольше, чем снимать
            for i in range(len(cands)):
                fr = [1.0 if k == i else 0.0 for k in range(len(cands))]
                try:
                    e = self.evaluate(cands, fr, angle)
                except (ValueError, GEOSException):
                    continue
                if better(e, best):  # один борт, не снимающий всю область, сюда не пройдёт
                    best = e
        n = len(cands)
        step = 0.2 if quick else 0.5
        evals = 0
        if quick:
            max_evals = 12
        elif self.mode == "grid":
            max_evals = 16
        else:
            # при w = 1 лексикографика принимает и мелкие выигрыши по налёту — ограничиваем
            max_evals = 40 if lexi else 200
        while step > 0.05 and evals < max_evals:
            improved = False
            for i in range(n):
                for j in range(n):
                    if evals >= max_evals:
                        break  # лимит оценок — и внутри прохода (при многих бортах пар сотни)
                    if i == j or best.fractions[i] <= 1e-6:
                        continue
                    fr = list(best.fractions)
                    moved = fr[i] * step
                    fr[i] -= moved
                    fr[j] += moved
                    if fr[i] < 0.02 * sum(fr):  # слишком маленький кусок — отдаём целиком
                        fr[j] += fr[i]
                        fr[i] = 0.0
                    try:
                        e = self.evaluate(cands, fr, angle)
                    except (ValueError, GEOSException):
                        continue
                    evals += 1
                    if better(e, best):
                        best, improved = e, True
            if not improved:
                step /= 2
        return best

    def _drop_overhead_drone(self, cands: list[Candidate], cur: Evaluation, angle: float, better) -> Evaluation | None:
        """Ход «убрать борт» — жадное исключение: пока план улучшается, пробуем убрать любой
        работающий борт (его участок уходит остальным, доли заново балансируются) и берём лучший
        вариант. Лишний борт удлиняет работу: ждёт очереди на старт с общей ВПП, летит к далёкому
        участку, с малой долей тратит время на взлёт и посадку. Первыми пробуем борта, у которых
        съёмка — меньшая часть времени. Возвращает лучший найденный план или None."""
        survey_kinds = ("survey", "tie")
        iters = 2 if self.mode == "grid" else 6
        best: Evaluation | None = None
        base = cur
        for _ in range(max(len(cands) - 1, 0)):
            if len(base.results) < 2:
                break
            by_id = {r.cand.instance_id: r for r in base.results}

            def share(i: int) -> float:
                r = by_id.get(cands[i].instance_id)
                if r is None or r.finish_s <= 0:
                    return 1.0
                return sum(l.duration_s for s in r.sorties for l in s.legs if l.kind in survey_kinds) / r.finish_s

            active = [i for i in range(len(cands)) if base.fractions[i] > 1e-6]
            round_best: Evaluation | None = None
            for i in sorted(active, key=share):
                keep = [k for k in active if k != i]
                sub = [cands[k] for k in keep]
                try:
                    e = self._balance_from(sub, [base.fractions[k] for k in keep], angle, iters)
                except (ValueError, GEOSException):
                    continue
                if e.unreachable_m2 > cur.unreachable_m2 + 1.0 or e.leftover_m2 > cur.leftover_m2 + 1.0:
                    continue
                full = [0.0] * len(cands)
                for k, f in zip(keep, e.fractions):
                    full[k] = f
                e.fractions = full
                if round_best is None or better(e, round_best):
                    round_best = e
            if round_best is None or not better(round_best, base):
                break
            best = base = round_best
        return best

    # ------------------------------------------------------------ вывод
    def _check(self, res: DroneResult) -> None:
        d = res.cand.drone
        p = res.cand.params
        sites = {**self.bases, **self.reserve_sites}
        wind = res.cand.wind or self.req.wind
        builder = SortieBuilder(d, p.speed_ms, p.altitude_agl_m, wind, res.cand.budget_s, self.router)
        # резерв заряда в секундах: то, что осталось сверх бюджета вылета
        reserve_s = usable_flight_time_s(d, wind, 0.0, res.cand.payload.endurance_factor) - res.cand.budget_s
        for s in res.sorties:
            builder.divert_check(s, sites)
            if s.max_divert_s > reserve_s:
                self.warnings.append(
                    f"{res.cand.instance_id}: в вылете {s.index + 1} до ближайшей площадки ({s.divert_site}) "
                    f"до {s.max_divert_s / 60:.1f} мин — больше резерва заряда {reserve_s / 60:.1f} мин; "
                    "добавьте резервную площадку ближе к области"
                )
        radio = d.radio_range_km * 1000
        base = self.bases[res.base_id]
        for s in res.sorties:
            far = max(math.dist(base, p) for leg in s.legs for p in leg.points)
            if far > radio:
                self.warnings.append(
                    f"{res.cand.instance_id}: вылет {s.index + 1} уходит на {far / 1000:.1f} км от базы, "
                    f"дальность радиоканала {d.radio_range_km:.0f} км"
                )
                break
        for s in res.sorties:
            for leg in s.legs:
                if leg.kind != "survey" and len(leg.points) >= 2:
                    line = LineString(leg.points)
                    if any(line.intersects(z) for z in self.nfz):
                        self.warnings.append(
                            f"{res.cand.instance_id}: вылет {s.index + 1} — перелёт пересекает запретную зону "
                            "(нет обходного пути: база или область внутри запрета)"
                        )
                        break

    def _coverage(self, results: list[DroneResult]) -> float:
        strips = []
        for r in results:
            half = r.cand.params.swath_m / 2
            for s in r.sorties:
                for leg in s.legs:
                    if leg.kind in ("survey", "tie") and leg.distance_m > 0:
                        strips.append(LineString(leg.points).buffer(half, cap_style="flat"))
        if not strips:
            return 0.0
        try:
            return unary_union(strips).intersection(self.area).area
        except GEOSException:
            # численная неустойчивость GEOS на почти совпадающих рёбрах — считаем на сетке 5 см
            return shapely.intersection(shapely.union_all(strips, grid_size=0.05), self.area, grid_size=0.05).area

    # ------------------------------------------------------------ рельеф
    def _terrain(self) -> Terrain | None:
        if not self.req.use_terrain:
            return None
        pts = [self.frame.xy_to_lonlat(*p) for p in list(self.bases.values()) + list(self.reserve_sites.values())]
        minx, miny, maxx, maxy = self.frame.to_wgs(self.area).bounds
        for lon, lat in pts:
            minx, miny, maxx, maxy = min(minx, lon), min(miny, lat), max(maxx, lon), max(maxy, lat)
        t = Terrain((minx - 0.02, miny - 0.02, maxx + 0.02, maxy + 0.02))
        if not t.available:
            self.warnings.append(
                "рельеф недоступен (нет сети и тайлов Copernicus DEM в кэше) — высоты заданы относительно точки старта"
            )
            return None
        return t

    def _dense(self, pts: list[tuple[float, float]], step: float) -> list[tuple[float, float]]:
        out = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            n = max(1, math.ceil(math.dist(a, b) / step))
            out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
        return out

    def _ground(self, terrain: Terrain, pts: list[tuple[float, float]]) -> list[float]:
        h = terrain.heights([self.frame.xy_to_lonlat(x, y) for x, y in pts])
        return [float(v) if v == v else 0.0 for v in h]  # NaN → 0

    def _apply_terrain(self, terrain: Terrain, r: DroneResult) -> dict[tuple[int, int], tuple[list, list]]:
        """Абсолютные высоты участков. Мультиротор — огибание рельефа на галсах, постоянный безопасный
        уровень на перелётах; самолёт — постоянный эшелон на весь вылет (не ниже точки старта)."""
        agl = r.cand.params.altitude_agl_m
        step = 50.0 if r.cand.params.tie_line_spacing_m else 100.0
        out: dict[tuple[int, int], tuple[list, list]] = {}
        base_ground = self._ground(terrain, [self.bases[r.base_id]])[0]
        worst_agl = 0.0
        for s in r.sorties:
            if r.cand.drone.type == "fixed_wing":
                all_pts = [p for leg in s.legs for p in self._dense(leg.points, 100.0)]
                level = max(self._ground(terrain, all_pts) + [base_ground]) + agl
                survey_pts = [p for leg in s.legs if leg.kind in ("survey", "tie") for p in self._dense(leg.points, 100.0)]
                if survey_pts:
                    worst_agl = max(worst_agl, level - min(self._ground(terrain, survey_pts)))
                for i, leg in enumerate(s.legs):
                    alt = []
                    for k, pt in enumerate(leg.points):
                        if _on_ground(leg.kind, k, len(leg.points)):
                            alt.append(base_ground)
                        elif leg.point_alts and leg.point_alts[k] < leg.alt_agl:
                            # набор после катапульты, раскрытие парашюта — над своим рельефом
                            alt.append(self._ground(terrain, [pt])[0] + leg.point_alts[k])
                        else:
                            alt.append(level)
                    out[(s.index, i)] = (leg.points, alt)
                continue
            for i, leg in enumerate(s.legs):
                if leg.kind in ("survey", "tie"):
                    pts = self._dense(leg.points, step)
                    out[(s.index, i)] = (pts, [g + agl for g in self._ground(terrain, pts)])
                elif leg.kind in ("takeoff", "landing"):
                    g = self._ground(terrain, leg.points[:1])[0]
                    n = len(leg.points)
                    out[(s.index, i)] = (leg.points, [g if _on_ground(leg.kind, k, n) else g + agl for k in range(n)])
                else:
                    level = max(self._ground(terrain, self._dense(leg.points, 100.0))) + agl
                    out[(s.index, i)] = (leg.points, [level] * len(leg.points))
        if worst_agl > agl * 1.1:
            # самолёт держит эшелон над высшей точкой рельефа — над низинами он выше заданной высоты
            p = r.cand.params
            msg = f"{r.cand.instance_id}: над низинами высота до {worst_agl:.0f} м над землёй вместо {agl:.0f} м"
            if p.gsd_cm:
                msg += f", GSD до {p.gsd_cm * worst_agl / agl:.1f} см вместо {p.gsd_cm:.1f}"
            ceiling = self.req.requirements.altitude_ceiling_m
            if ceiling is not None and worst_agl > ceiling:
                msg += f"; выше потолка {ceiling:.0f} м"
            self.warnings.append(msg + " — разбейте область по высотам рельефа или используйте мультиротор")
        return out

    def run(self) -> PlanResponse:
        ev, angle = self.solve()
        f = self.frame
        terrain = self._terrain()
        drones_out: list[DronePlanOut] = []
        levels = self._transit_levels(ev.results)
        for r in ev.results:
            self._check(r)
            amsl = self._apply_terrain(terrain, r) if terrain else {}
            off = levels[r.cand.instance_id]
            sorties_out = []
            for s in r.sorties:
                legs = []
                for li, l in enumerate(s.legs):
                    pts, alts = amsl.get((s.index, li), (l.points, None))
                    # эшелон перелёта: взлёт и посадка — до него, транзит и возврат — на нём
                    lift = off if l.kind in TRANSIT_KINDS + ("takeoff", "landing") else 0.0

                    def agl(i: int, l=l, n=len(pts)) -> tuple[float, float]:
                        """Высота точки AGL и подъём на эшелон перелёта (только для точек на эшелоне)."""
                        if _on_ground(l.kind, i, n):
                            return 0.0, 0.0
                        if l.point_alts and l.point_alts[i] < l.alt_agl:
                            return l.point_alts[i], 0.0
                        return l.alt_agl + lift, lift

                    legs.append(LegOut(
                        kind=l.kind,
                        coordinates=[[*f.xy_to_lonlat(x, y), agl(i)[0]] for i, (x, y) in enumerate(pts)],
                        alt_amsl=[round(a + agl(i)[1], 1) for i, a in enumerate(alts)] if alts else None,
                        duration_s=round(l.duration_s, 1),
                        distance_m=round(l.distance_m, 1),
                        speed_ms=round(l.speed_ms, 2),
                    ))
                sorties_out.append(
                    SortieOut(index=s.index, base_id=s.base_id, start_s=round(s.start_s, 1),
                          max_divert_s=round(s.max_divert_s, 1), divert_site=s.divert_site,
                              duration_s=round(s.duration_s, 1), survey_length_m=round(s.survey_length_m, 1), legs=legs)
                )
            drones_out.append(
                DronePlanOut(
                    drone_id=r.cand.instance_id,
                    model=r.cand.drone.id,
                    model_name=r.cand.drone.name,
                    payload=r.cand.payload.id,
                    params=r.cand.params,
                    sweep_angle_deg=round(math.degrees(angle) % 180, 1),
                    region=mapping(f.to_wgs(r.region)),
                    area_km2=round(r.region.area / 1e6, 4),
                    sorties=sorties_out,
                    flight_time_s=round(r.flight_s, 1),
                    finish_s=round(r.finish_s, 1),
                    transit_alt_agl_m=round(r.cand.params.altitude_agl_m + off, 1),
                )
            )
        unused = {c.instance_id for c in self.candidates} - {r.cand.instance_id for r in ev.results}
        for u in sorted(unused):
            self.excluded.append(ExcludedDrone(drone_id=u, reason="не задействован: так выгоднее по выбранному критерию"))
        if ev.leftover_m2 > 100:
            has_multi = any(c.drone.type == "multirotor" for c in self.candidates)
            advice = ("мультироторы парка не долетают до этих полос — поставьте ВПП ближе к зонам"
                      if has_multi else "добавьте в парк мультиротор")
            self.warnings.append(
                f"{ev.leftover_m2 / 1e6:.3f} км² у запретных зон не снято: самолёту не хватает места для "
                f"разворота; {advice} или увеличьте запас вокруг зон"
            )
        if ev.unreachable_m2 > 100:
            self.warnings.append(
                f"{ev.unreachable_m2 / 1e6:.3f} км² вне радиуса действия всех бортов (заряд или радиоканал) — "
                "добавьте ВПП ближе к области или борт с большей дальностью"
            )
        sep = closest_approach(
            [track(r.cand.instance_id, r.sorties,
                   lambda leg, off=levels[r.cand.instance_id]: leg.alt_agl + (off if leg.kind in TRANSIT_KINDS else 0.0))
             for r in ev.results],
            list(self.bases.values()),
            ev.makespan,
        )
        if sep is not None and sep.min_h_m < H_MIN_M:
            self.warnings.append(
                f"сближение бортов {sep.pair[0]} и {sep.pair[1]} до {sep.min_h_m:.0f} м на одной высоте "
                f"(через {sep.t_s / 60:.0f} мин от начала работ; моментов с конфликтом: {sep.conflicts}) — "
                "разнесите их по времени или по высоте"
            )
        covered = self._coverage(ev.results)
        area = self.area.area
        summary = Summary(
            makespan_s=round(ev.makespan, 1),
            total_flight_s=round(ev.total, 1),
            sorties=sum(len(r.sorties) for r in ev.results),
            area_km2=round(area / 1e6, 4),
            covered_km2=round(covered / 1e6, 4),
            coverage_pct=round(100 * covered / area, 2),
            drones_used=len(ev.results),
            min_separation_m=round(sep.min_h_m, 1) if sep is not None else None,
            separation_pair=list(sep.pair) if sep is not None else [],
            separation_conflicts=sep.conflicts if sep is not None else 0,
        )
        return PlanResponse(
            summary=summary,
            drones=drones_out,
            excluded=self.excluded,
            warnings=self.warnings,
            working_area=mapping(f.to_wgs(self.area)),
            time_weight=self.req.time_weight,
            reserve_sites=self.req.reserve_sites,
            bases=self.req.bases,
            no_fly_zones=self.req.no_fly_zones,
            allowed_area=self.req.allowed_area,
            terrain=self._terrain_info(terrain),
        )

    def _transit_levels(self, results: list[DroneResult]) -> dict[str, float]:
        """Эшелоны перелёта: k-й борт на k·TRANSIT_LEVEL_STEP_M выше высоты съёмки, но не выше
        потолка разрешённого объёма (если задан)."""
        ceiling = self.req.requirements.altitude_ceiling_m
        out = {}
        for k, r in enumerate(results):
            room = math.inf if ceiling is None else max(ceiling - r.cand.params.altitude_agl_m, 0.0)
            out[r.cand.instance_id] = min(TRANSIT_LEVEL_STEP_M * k, room)
        return out

    def _terrain_info(self, terrain: Terrain | None) -> TerrainInfo | None:
        if terrain is None:
            return None
        g = self._ground(terrain, self._dense(list(self.area.exterior.coords) if isinstance(self.area, Polygon)
                                              else [c for p in self.area.geoms for c in p.exterior.coords], 200.0))
        return TerrainInfo(source=TERRAIN_ATTRIBUTION, ground_min_m=round(min(g), 1),
                           ground_max_m=round(max(g), 1))


def _on_ground(kind: str, k: int, n: int) -> bool:
    """Точка участка на земле: начало взлёта (земля → высота) и конец посадки (высота → земля)."""
    return (kind == "takeoff" and k == 0) or (kind == "landing" and k == n - 1)


def plan(req: PlanRequest) -> PlanResponse:
    return Planner(req).run()
