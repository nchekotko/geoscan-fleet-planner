"""Фронт Парето по двум критериям: время выполнения работ (T_max) и суммарный налёт (ΣT).

Точки получаем, решая задачу для набора весов w ∈ [0, 1] в целевой функции
J = w·T_max/T_ref + (1 − w)·ΣT/ΣT_ref (метод взвешенной суммы), и оставляем недоминируемые.
Расчёты для разных весов независимы и идут параллельно в отдельных процессах.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor

from shapely.geometry import shape

from .geo import LocalFrame
from .planner import plan
from .schemas import PlanRequest, PlanResponse

DEFAULT_WEIGHTS = (1.0, 0.9, 0.75, 0.6, 0.45, 0.3, 0.15, 0.0)
LARGE_AREA_KM2 = 20.0


def _area_km2(req: PlanRequest) -> float:
    g = shape(req.survey_area)
    return LocalFrame.around(g).to_local(g).area / 1e6


def _solve(args: tuple[dict, float, float | None]) -> PlanResponse | None:
    data, w, cap = args
    req = PlanRequest(**data)
    req.time_weight = w
    req.makespan_cap_s = cap
    try:
        return plan(req)
    except ValueError:
        return None


def non_dominated(plans: list[PlanResponse]) -> list[PlanResponse]:
    pts = sorted(plans, key=lambda p: (p.summary.makespan_s, p.summary.total_flight_s))
    front: list[PlanResponse] = []
    best_total = float("inf")
    for p in pts:
        if p.summary.total_flight_s < best_total - 1.0:  # допуск 1 с
            front.append(p)
            best_total = p.summary.total_flight_s
    return front


def _run(jobs, workers):
    if workers == 1:
        return [_solve(j) for j in jobs]
    with ProcessPoolExecutor(max_workers=workers or min(len(jobs), 6)) as ex:
        return list(ex.map(_solve, jobs))


def pareto_front(
    req: PlanRequest, weights=DEFAULT_WEIGHTS, workers: int | None = None, eps_points: int = 5
) -> list[PlanResponse]:
    """Взвешенная сумма даёт выпуклую часть фронта; метод ε-ограничений (Mavrotas, 2009)
    добирает точки между крайними планами: min ΣT при T_max ≤ ε."""
    data = req.model_dump()
    if _area_km2(req) > LARGE_AREA_KM2 and weights is DEFAULT_WEIGHTS:
        # большие области считаются в сеточном режиме дольше — берём меньше точек
        weights, eps_points = (1.0, 0.5, 0.0), min(eps_points, 2)
    results = _run([(data, w, None) for w in weights], workers)
    ok = [r for r in results if r is not None]
    if len(ok) >= 2 and eps_points > 0:
        t_lo = min(r.summary.makespan_s for r in ok)
        t_hi = max(r.summary.makespan_s for r in ok)
        if t_hi > t_lo * 1.02:
            caps = [t_lo + (t_hi - t_lo) * k / (eps_points + 1) for k in range(1, eps_points + 1)]
            ok += [r for r in _run([(data, 1.0, c) for c in caps], workers) if r is not None]
    return non_dominated(ok)
