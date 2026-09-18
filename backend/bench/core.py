"""Общие части бенчмарка: базовые подходы, метрики плана и прогон одного сценария.

Базовые подходы строятся тем же Planner.evaluate (те же модели полёта, вылетов, ветра),
но без оптимизации долей, порядка и угла — сравнивается именно распределение работы.
"""
from __future__ import annotations

import math
import time
import traceback
from dataclasses import dataclass, field

from planner.coverage import best_direction
from planner.planner import Evaluation, Planner, plan
from planner.schemas import PlanRequest, PlanResponse

from bench.lower_bound import lower_bound
from bench.random_scenarios import random_request

COVERAGE_OK = 99.5  # %, ниже — план считается неполным и не участвует в сравнении времени

GROUPS = {
    "survey": ("survey", "tie"),
    "turn": ("turn",),
    "transit": ("transit", "return"),
    "ground": ("takeoff", "landing"),
}


@dataclass
class PlanStats:
    label: str
    makespan_s: float = math.nan
    total_s: float = math.nan
    coverage_pct: float = 0.0
    sorties: int = 0
    drones_used: int = 0
    area_km2: float = 0.0
    time_s: dict[str, float] = field(default_factory=dict)   # налёт по группам участков
    dist_m: dict[str, float] = field(default_factory=dict)
    lines_x_spacing_m2: float = 0.0  # Σ длина основных галсов × шаг (проверка допущения границы)
    finish_s: list[float] = field(default_factory=list)  # окончание работ каждым задействованным бортом
    compute_s: float = 0.0
    mode: str = ""
    note: str = ""
    error: str = ""

    @property
    def ok(self) -> bool:
        return not self.error and math.isfinite(self.makespan_s)

    @property
    def full(self) -> bool:
        """План построен и снимает не меньше COVERAGE_OK % рабочей области."""
        return self.ok and self.coverage_pct >= COVERAGE_OK

    def share(self, group: str) -> float:
        tot = sum(self.time_s.values())
        return self.time_s.get(group, 0.0) / tot if tot > 0 else math.nan

    @property
    def productivity_km2_h(self) -> float:
        return self.area_km2 / (self.makespan_s / 3600) if self.ok and self.makespan_s > 0 else math.nan

    @property
    def deadhead_km(self) -> float:
        return self.dist_m.get("transit", 0.0) / 1000


def _fill(st: PlanStats, drones: list[tuple[float, list]]) -> PlanStats:
    """drones — [(шаг галсов, вылеты)], вылет — объект с legs (kind, duration_s, distance_m)."""
    st.time_s = {g: 0.0 for g in GROUPS}
    st.dist_m = {g: 0.0 for g in GROUPS}
    kind_to_group = {k: g for g, ks in GROUPS.items() for k in ks}
    lines = 0.0
    for spacing, sorties in drones:
        for s in sorties:
            for leg in s.legs:
                g = kind_to_group.get(leg.kind)
                if g is None:
                    continue
                st.time_s[g] += leg.duration_s
                st.dist_m[g] += leg.distance_m
                if leg.kind == "survey":
                    lines += leg.distance_m * spacing
    st.lines_x_spacing_m2 = lines
    st.sorties = sum(len(s) for _, s in drones)
    st.drones_used = sum(1 for _, s in drones if s)
    return st


def stats_from_response(label: str, r: PlanResponse, compute_s: float) -> PlanStats:
    st = PlanStats(label, makespan_s=r.summary.makespan_s, total_s=r.summary.total_flight_s,
                   coverage_pct=r.summary.coverage_pct, area_km2=r.summary.area_km2, compute_s=compute_s,
                   mode="grid" if any("с учётом дальности" in w for w in r.warnings) else "strips")
    # планировщик сам сообщает о неснятой площади: вне радиуса действия или у запретных зон
    known = [w for w in r.warnings if "вне радиуса действия всех бортов" in w or "не снято" in w]
    st.note = "; ".join(known)
    st.finish_s = [d.finish_s for d in r.drones if d.sorties]
    return _fill(st, [(d.params.line_spacing_m, d.sorties) for d in r.drones])


def stats_from_eval(label: str, p: Planner, ev: Evaluation, compute_s: float) -> PlanStats:
    cov = 100 * p._coverage(ev.results) / p.area.area
    st = PlanStats(label, makespan_s=ev.makespan, total_s=ev.total, coverage_pct=cov, area_km2=p.area.area / 1e6,
                   compute_s=compute_s, mode=p.mode)
    st.finish_s = [r.finish_s for r in ev.results if r.sorties]
    return _fill(st, [(r.cand.params.line_spacing_m, r.sorties) for r in ev.results])


def error_text(e: BaseException) -> str:
    tb = traceback.extract_tb(e.__traceback__)
    where = f" [{tb[-1].filename.replace(chr(92), '/').split('/')[-1]}:{tb[-1].lineno}]" if tb else ""
    return f"{type(e).__name__}: {e}{where}"


def evaluate_fixed(p: Planner, cands, fractions: list[float], angle: float) -> Evaluation:
    """Одна оценка с заданными долями: полосами, а если полоса вне радиуса действия — сеткой."""
    p.mode = "strips"
    try:
        return p.evaluate(cands, fractions, angle)
    except ValueError:
        p.mode = "grid"
        return p.evaluate(cands, fractions, angle)


def baseline_fixed(p: Planner, label: str, weights) -> PlanStats:
    """Доли по весам weights(кандидат) при угле галсов ведущего борта (Planner.prepare)."""
    t = time.perf_counter()
    try:
        cands, angle = p.prepare()
        ev = evaluate_fixed(p, cands, [weights(c) for c in cands], angle)
    except Exception as e:  # noqa: BLE001 — базовый подход не должен ронять бенчмарк
        return PlanStats(label, error=error_text(e))
    return stats_from_eval(label, p, ev, time.perf_counter() - t)


def baseline_single(p: Planner) -> PlanStats:
    """Вся область одному борту; каждый борт — со своим лучшим углом галсов. Берётся самый
    быстрый из бортов, снимающих ≥ COVERAGE_OK %, если таких нет — с наибольшим покрытием."""
    t = time.perf_counter()
    best: PlanStats | None = None
    errors = []
    for c in p.candidates:
        try:
            angle, _, _ = best_direction(p.area, c.params.line_spacing_m, c.drone, c.params.speed_ms, p.req.wind)
            cands = p._order(angle)
            ev = evaluate_fixed(p, cands, [1.0 if x is c else 0.0 for x in cands], angle)
        except Exception as e:  # noqa: BLE001
            errors.append(f"{c.instance_id}: {error_text(e)}")
            continue
        st = stats_from_eval(f"один лучший борт ({c.drone.name})", p, ev, 0.0)
        if st.drones_used > 1:
            st.note = "остатки у запретных зон снимает мультиротор"
        key = (not st.full, -st.coverage_pct if not st.full else 0.0, st.makespan_s)
        if best is None or key < (not best.full, -best.coverage_pct if not best.full else 0.0, best.makespan_s):
            best = st
    if best is None:
        return PlanStats("один лучший борт", error="; ".join(errors) or "нет бортов")
    best.compute_s = time.perf_counter() - t
    return best


@dataclass
class CaseResult:
    name: str
    area_km2: float = 0.0
    n_drones: int = 0
    wind_ms: float = 0.0
    rows: dict[str, PlanStats] = field(default_factory=dict)
    lb_cont_s: float = math.nan
    lb_int_s: float = math.nan       # целые вылеты + очередь стартов — основная граница
    lb_noqueue_s: float = math.nan   # целые вылеты без очереди стартов
    lb_error: str = ""
    # параметры границы по бортам и базам: (id, база, B, O, R, t_swap, площадь за полный вылет, м²)
    lb_drones: list[tuple] = field(default_factory=list)
    error: str = ""          # не удалось даже подготовить задачу (Planner)
    runtime_s: float = 0.0

    def ratio(self, base: str, ours: str = "w1", strict: bool = True) -> float:
        """Во сколько раз базовый подход дольше нашего плана (T_base / T_our); NaN, если
        любой из двух планов не построен или (при strict) снимает меньше COVERAGE_OK %."""
        a, b = self.rows.get(base), self.rows.get(ours)
        if a is None or b is None or not (a.ok and b.ok) or (strict and not (a.full and b.full)):
            return math.nan
        return a.makespan_s / b.makespan_s

    def gap(self, ours: str = "w1") -> float:
        b = self.rows.get(ours)
        if b is None or not b.full or not math.isfinite(self.lb_int_s) or self.lb_int_s <= 0:
            return math.nan
        return b.makespan_s / self.lb_int_s


ROW_ORDER = ("single", "equal", "operator", "w1", "w0")


def run_case(name: str, req_data: dict, with_baselines: bool = True) -> CaseResult:
    """Все планы для одного сценария. Не бросает исключений: ошибки — в полях error."""
    t0 = time.perf_counter()
    res = CaseResult(name)
    data = {**req_data, "use_terrain": False}
    try:
        req = PlanRequest(**{**data, "time_weight": 1.0})
        p = Planner(req)
    except Exception as e:  # noqa: BLE001
        res.error = error_text(e)
        res.runtime_s = time.perf_counter() - t0
        return res
    res.area_km2 = p.area.area / 1e6
    res.n_drones = len(req.drones)
    res.wind_ms = req.wind.speed_ms
    try:
        lb = lower_bound(p)
        res.lb_cont_s, res.lb_int_s, res.lb_noqueue_s = lb.continuous_s, lb.integer_s, lb.integer_noqueue_s
        res.lb_drones = [(d.instance_id, d.base_id, d.budget_s, d.overhead_s, d.reach_s, d.swap_s,
                          d.area_per_sortie_m2)
                         for d in lb.drones]
    except Exception as e:  # noqa: BLE001
        res.lb_error = error_text(e)
    if with_baselines:
        res.rows["single"] = baseline_single(p)
        res.rows["equal"] = baseline_fixed(p, "поровну без учёта ТТХ", lambda c: 1.0)
        res.rows["operator"] = baseline_fixed(p, "оператор: доли ∝ скорость × шаг галсов",
                                              lambda c: c.params.speed_ms * c.params.line_spacing_m)
    for key, w, label in (("w1", 1.0, "наш план: время работ (w = 1)"), ("w0", 0.0, "наш план: налёт (w = 0)")):
        t = time.perf_counter()
        try:
            r = plan(PlanRequest(**{**data, "time_weight": w}))
        except Exception as e:  # noqa: BLE001
            res.rows[key] = PlanStats(label, error=error_text(e), compute_s=time.perf_counter() - t)
            continue
        res.rows[key] = stats_from_response(label, r, time.perf_counter() - t)
    res.runtime_s = time.perf_counter() - t0
    return res


def run_random_case(seed: int) -> tuple[int, dict, CaseResult]:
    """Случайный сценарий по seed (для пула процессов: функция уровня модуля)."""
    data = random_request(seed)
    return seed, data, run_case(f"seed {seed}", data)
