"""Оркестрация планирования для парка БВС.

1. Рабочая область = (область съёмки ∩ разрешённая зона) − buffer(NFZ).
2. Для каждого борта: выбор нагрузки, параметры съёмки, бюджет вылета; неподходящие исключаются.
3. Общее направление галсов, полосы с долями площади по производительности бортов.
4. Балансировка долей по фактическому времени (критерий «время работ»).
5. Для time_weight < 1 — локальный поиск по долям, минимизирующий
   J = w·T_max/T_ref + (1 − w)·ΣT/ΣT_ref.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from shapely.geometry import LineString, MultiPolygon, Point, Polygon, mapping, shape
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union
from shapely.errors import GEOSException
from shapely.validation import make_valid

from .avoid import Router
from .coverage import best_direction, sweep_passes
from .energy import check_wind, usable_flight_time_s
from .fleet import DroneModel, Payload, load_fleet
from .geo import LocalFrame
from .mission import Sortie, SortieBuilder, order_passes, schedule
from .terrain import Terrain
from .turns import turn_overshoot
from .grid_partition import grid_partition
from .mission import FIXED_WING_LANDING_S, FIXED_WING_TAKEOFF_S, MULTIROTOR_DESCENT_MS
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


class PlanningError(ValueError):
    pass


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
        self.reserve_sites = {r.id: f.lonlat_to_xy(r.lon, r.lat) for r in req.reserve_sites}
        if not self.bases:
            raise PlanningError("не задано ни одного взлётно-посадочного пункта")
        self.candidates = self._candidates()
        if not self.candidates:
            raise PlanningError("ни один борт не может выполнить съёмку этого типа в заданных условиях")

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
            wind_problem = check_wind(drone, req.wind)
            if wind_problem:
                self.excluded.append(ExcludedDrone(drone_id=inst.id, reason=wind_problem))
                continue
            try:
                # против ветра нужна путевая скорость ≥ ~2 м/с
                params = survey_params(drone, payload, req.requirements, min_speed=req.wind.speed_ms + 2.0)
            except SurveyInfeasible as e:
                self.excluded.append(ExcludedDrone(drone_id=inst.id, reason=str(e)))
                continue
            # против ветра и поперёк него линию пути держать можно только при V > W
            if params.speed_ms <= req.wind.speed_ms + 0.5:
                self.excluded.append(ExcludedDrone(
                    drone_id=inst.id,
                    reason=f"{drone.name}: скорость съёмки {params.speed_ms:.1f} м/с не выше ветра "
                           f"{req.wind.speed_ms:.1f} м/с (увеличьте GSD или снизьте перекрытие)",
                ))
                continue
            budget = usable_flight_time_s(drone, req.wind, req.reserve)
            overhead = params.altitude_agl_m / drone.climb_rate_ms * 2 + (180 if drone.type == "fixed_wing" else 0)
            availability = max(budget - overhead, 1.0) / (budget + drone.swap_time_min * 60)
            productivity = params.speed_ms * params.line_spacing_m * availability
            fw = drone.type == "fixed_wing"
            t_ops = params.altitude_agl_m / drone.climb_rate_ms + (
                FIXED_WING_TAKEOFF_S + FIXED_WING_LANDING_S if fw else params.altitude_agl_m / MULTIROTOR_DESCENT_MS
            )
            v_back = max(max(params.speed_ms, drone.cruise_speed_ms) - req.wind.speed_ms, 0.5)
            reach = min(0.5 * max(budget - t_ops, 0.0) * v_back * 0.85, drone.radio_range_km * 1000)
            out.append(Candidate(inst.id, drone, payload, params, budget, inst.base_id, productivity, reach))
        return out

    # ------------------------------------------------------------- оценка
    def _default_base(self, region: BaseGeometry) -> str:
        c = region.centroid
        return min(self.bases, key=lambda b: math.dist(self.bases[b], (c.x, c.y)))

    def _reassign_turn_gaps(
        self, cands: list[Candidate], regions: list[BaseGeometry]
    ) -> tuple[list[BaseGeometry], float]:
        """Полосы у NFZ, где самолёт не может развернуться, отдаём ближайшему мультиротору."""
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
                able = [
                    k for k in multi
                    if self._far_point(piece, cands[k]) <= cands[k].reach_m
                ]
                if not able:
                    leftover += piece.area
                    continue
                j = min(able, key=lambda k: regions[k].distance(piece) if not regions[k].is_empty
                        else self._far_point(piece, cands[k]))
                regions[j] = _polys(unary_union([regions[j], piece]))
        return regions, leftover

    def _far_point(self, geom: BaseGeometry, cand: Candidate) -> float:
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
            )
            unreachable = rest.area
        else:
            regions = split_by_fractions(self.area, angle, [f for _, f in active])
        regions, leftover = self._reassign_turn_gaps([c for c, _ in active], regions)
        results: list[DroneResult] = []
        for (cand, _), region in zip(active, regions):
            base_id = cand.base_id or self._default_base(region)
            base = self.bases[base_id]
            p = cand.params
            passes = sweep_passes(region, angle, p.line_spacing_m) if not region.is_empty else []
            route = order_passes(passes, base)
            if p.tie_line_spacing_m and not region.is_empty:
                # секущие маршруты поперёк основных галсов (геофизика)
                ties = sweep_passes(region, angle + math.pi / 2, p.tie_line_spacing_m)
                route += order_passes(ties, route[-1].b if route else base, kind="tie")
            builder = SortieBuilder(cand.drone, p.speed_ms, p.altitude_agl_m, self.req.wind, cand.budget_s, self.router)
            sorties = builder.build(route, base, base_id)
            finish = schedule(sorties, cand.drone.swap_time_min * 60)
            results.append(DroneResult(cand, region, base_id, sorties, finish))
        return Evaluation(fractions=list(fractions), results=results, leftover_m2=leftover, unreachable_m2=unreachable)

    # ---------------------------------------------------------- оптимизация
    def solve(self) -> tuple[Evaluation, float]:
        cands, angle = self.prepare()
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
        return best, angle

    def prepare(self) -> tuple[list[Candidate], float]:
        """Общее направление галсов и порядок бортов вдоль оси разбиения."""
        cands = self.candidates
        # направление галсов — по самому производительному борту
        lead = max(cands, key=lambda c: c.productivity)
        angle, _, _ = best_direction(self.area, lead.params.line_spacing_m, lead.drone, lead.params.speed_ms, self.req.wind)

        # порядок полос ↔ положение баз вдоль оси, перпендикулярной галсам
        def axis_key(c: Candidate) -> float:
            b = self.bases[c.base_id or self._default_base(self.area)]
            return strip_axis_position(b, angle)

        cands.sort(key=axis_key)
        return cands, angle

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
        return min(results, key=lambda e: e.makespan)

    def _balance_from(self, cands: list[Candidate], fr: list[float], angle: float, iters: int) -> Evaluation:
        best = self.evaluate(cands, fr, angle)
        cur = best
        for it in range(iters):
            if self.mode == "grid" and it >= 4:
                break  # сеточное разбиение дороже, хватает нескольких итераций
            times = [r.finish_s for r in cur.results]
            mean = sum(times) / len(times)
            if max(times) - min(times) < 0.02 * mean:
                break
            fr = [f * (mean / max(t, 1.0)) ** 0.8 for f, t in zip(cur.fractions, times)]
            s = sum(fr)
            fr = [f / s for f in fr]
            try:
                cur = self.evaluate(cands, fr, angle)
            except ValueError:
                break
            if cur.makespan < best.makespan:
                best = cur
        return best

    def _local_search(
        self, cands: list[Candidate], start: Evaluation, angle: float, w: float, cap: float | None = None
    ) -> Evaluation:
        t_ref, s_ref = start.makespan, start.total

        def score(e: Evaluation) -> float:
            j = w * e.makespan / t_ref + (1 - w) * e.total / s_ref
            if cap is not None and e.makespan > cap:
                j += 10.0 * (e.makespan - cap) / t_ref  # штраф за нарушение ε-ограничения
            return j

        best, best_j = start, score(start)
        # дополнительные стартовые точки: вся работа одному борту
        for i in range(len(cands)):
            fr = [1.0 if k == i else 0.0 for k in range(len(cands))]
            try:
                e = self.evaluate(cands, fr, angle)
            except (ValueError, GEOSException):
                continue
            if e.unreachable_m2 > best.unreachable_m2 + 1.0:
                continue  # один борт не достаёт до всей области — не считаем решением
            if score(e) < best_j - 1e-6:
                best, best_j = e, score(e)
        n = len(cands)
        step = 0.5
        evals = 0
        max_evals = 16 if self.mode == "grid" else 200
        while step > 0.05 and evals < max_evals:
            improved = False
            for i in range(n):
                for j in range(n):
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
                    j_val = score(e)
                    if j_val < best_j - 1e-6:
                        best, best_j, improved = e, j_val, True
            if not improved:
                step /= 2
        return best

    # ------------------------------------------------------------ вывод
    def _check(self, res: DroneResult) -> None:
        d = res.cand.drone
        p = res.cand.params
        sites = {**self.bases, **self.reserve_sites}
        builder = SortieBuilder(d, p.speed_ms, p.altitude_agl_m, self.req.wind, res.cand.budget_s, self.router)
        # резерв заряда в секундах: то, что осталось сверх бюджета вылета
        reserve_s = usable_flight_time_s(d, self.req.wind, 0.0) - res.cand.budget_s
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
        return unary_union(strips).intersection(self.area).area

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
        for s in r.sorties:
            if r.cand.drone.type == "fixed_wing":
                all_pts = [p for leg in s.legs for p in self._dense(leg.points, 100.0)]
                level = max(self._ground(terrain, all_pts) + [base_ground]) + agl
                for i, leg in enumerate(s.legs):
                    alt = [base_ground if leg.kind in ("takeoff", "landing") and k == 0 else level
                           for k in range(len(leg.points))]
                    out[(s.index, i)] = (leg.points, alt)
                continue
            for i, leg in enumerate(s.legs):
                if leg.kind in ("survey", "tie"):
                    pts = self._dense(leg.points, step)
                    out[(s.index, i)] = (pts, [g + agl for g in self._ground(terrain, pts)])
                elif leg.kind in ("takeoff", "landing"):
                    g = self._ground(terrain, leg.points[:1])[0]
                    out[(s.index, i)] = (leg.points, [g] + [g + agl] * (len(leg.points) - 1))
                else:
                    level = max(self._ground(terrain, self._dense(leg.points, 100.0))) + agl
                    out[(s.index, i)] = (leg.points, [level] * len(leg.points))
        return out

    def run(self) -> PlanResponse:
        ev, angle = self.solve()
        f = self.frame
        terrain = self._terrain()
        drones_out: list[DronePlanOut] = []
        for r in ev.results:
            self._check(r)
            amsl = self._apply_terrain(terrain, r) if terrain else {}
            sorties_out = []
            for s in r.sorties:
                legs = []
                for li, l in enumerate(s.legs):
                    pts, alts = amsl.get((s.index, li), (l.points, None))
                    legs.append(LegOut(
                        kind=l.kind,
                        coordinates=[[*f.xy_to_lonlat(x, y), 0.0 if l.kind in ("takeoff", "landing") and i == 0 else l.alt_agl]
                                     for i, (x, y) in enumerate(pts)],
                        alt_amsl=[round(a, 1) for a in alts] if alts else None,
                        duration_s=round(l.duration_s, 1),
                        distance_m=round(l.distance_m, 1),
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
                )
            )
        unused = {c.instance_id for c in self.candidates} - {r.cand.instance_id for r in ev.results}
        for u in sorted(unused):
            self.excluded.append(ExcludedDrone(drone_id=u, reason="не задействован: так выгоднее по выбранному критерию"))
        if ev.leftover_m2 > 100:
            self.warnings.append(
                f"{ev.leftover_m2 / 1e6:.3f} км² у запретных зон не снято: самолёту не хватает места для "
                "разворота. Добавьте в парк мультиротор или увеличьте запас вокруг зон"
            )
        if ev.unreachable_m2 > 100:
            self.warnings.append(
                f"{ev.unreachable_m2 / 1e6:.3f} км² вне радиуса действия всех бортов (заряд или радиоканал) — "
                "добавьте ВПП ближе к области или борт с большей дальностью"
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
            terrain=self._terrain_info(terrain),
        )

    def _terrain_info(self, terrain: Terrain | None) -> TerrainInfo | None:
        if terrain is None:
            return None
        g = self._ground(terrain, self._dense(list(self.area.exterior.coords) if isinstance(self.area, Polygon)
                                              else [c for p in self.area.geoms for c in p.exterior.coords], 200.0))
        return TerrainInfo(source="Copernicus DEM GLO-30 (DSM, EGM2008)", ground_min_m=round(min(g), 1),
                           ground_max_m=round(max(g), 1))


def plan(req: PlanRequest) -> PlanResponse:
    return Planner(req).run()
