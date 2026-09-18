"""Фронт Парето по двум критериям: время выполнения работ (T_max) и суммарный налёт (ΣT).

Точки получаем, решая задачу для набора весов w ∈ [0, 1] в целевой функции
J = w·T_max/T_ref + (1 − w)·ΣT/ΣT_ref (метод взвешенной суммы), и оставляем недоминируемые.
Расчёты для разных весов независимы и идут параллельно в отдельных процессах.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor

from .planner import plan
from .schemas import PlanRequest, PlanResponse

DEFAULT_WEIGHTS = (1.0, 0.9, 0.75, 0.6, 0.45, 0.3, 0.15, 0.0)


def _solve(args: tuple[dict, float]) -> PlanResponse | None:
    data, w = args
    req = PlanRequest(**data)
    req.time_weight = w
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


def pareto_front(req: PlanRequest, weights=DEFAULT_WEIGHTS, workers: int | None = None) -> list[PlanResponse]:
    data = req.model_dump()
    jobs = [(data, w) for w in weights]
    if workers == 1:
        results = [_solve(j) for j in jobs]
    else:
        with ProcessPoolExecutor(max_workers=workers or min(len(jobs), 6)) as ex:
            results = list(ex.map(_solve, jobs))
    return non_dominated([r for r in results if r is not None])
