"""Советник: ответы на вопросы «выполнимо ли», «сколько бортов нужно к сроку»,
«сколько времени займут работы этим числом бортов».

Заказчик сформулировал это так: пользователь задаёт ограничение (число бортов или срок), а
сервис подсказывает, выполнимо ли задание, сколько бортов потребуется, чтобы уложиться в срок,
и сколько времени займёт работа заданным числом бортов.

Как считаем: строим план для 1, 2, … N бортов и получаем кривую «время работ от числа бортов».
Порядок, в котором борта добавляются, — по убыванию производительности с учётом удаления базы
от области: первым берётся тот, кто снимет больше за час работы (жадная эвристика; перебор всех
подмножеств — 2^N планов, для интерактивного ответа это слишком долго). Планы для разного числа
бортов независимы и считаются параллельно.

Время работ не обязано монотонно падать с числом бортов: лишний борт с далёкой базой может
затянуть работы. Поэтому в ответе и точное время для каждого числа бортов, и «лучшее к этому
числу» (running min) — по нему и отвечаем на вопрос о сроке.
"""
from __future__ import annotations

import math

from shapely.geometry import Point, shape

from .geo import LocalFrame
from .pareto import _run
from .planner import PlanningError, Planner
from .schemas import AdviceOption, AdviceResponse, PlanRequest, PlanResponse

# сколько планов считать: больше 12 точек кривой для подсказки не нужно
MAX_POINTS = 12
# набор бортов, который не снимает область целиком, срок не выполняет (обычно не хватает
# мультиротора на кольца у запретных зон)
COVERAGE_MIN_PCT = 99.5


def _fleet_order(req: PlanRequest) -> list[str]:
    """Порядок добавления бортов: по убыванию «полезной производительности» — площади съёмки
    за час работ с поправкой на перелёт от своей базы до области."""
    try:
        pl = Planner(req)
    except PlanningError:
        return [d.id for d in req.drones]
    g = shape(req.survey_area)
    frame = LocalFrame.around(g)
    area = frame.to_local(g)
    score: dict[str, float] = {}
    for c in pl.candidates:
        base = pl.bases[c.base_id or pl._default_base(area)]
        transit_s = 2 * area.distance(Point(base)) / max(c.params.speed_ms, 1.0)
        # доля вылета, которая уходит на съёмку, а не на дорогу
        useful = max(c.budget_s - transit_s, 0.0) / max(c.budget_s, 1.0)
        score[c.instance_id] = c.productivity * useful
    order = sorted(score, key=lambda i: -score[i])
    # борта, которые вообще не могут работать (исключённые), — в конец
    return order + [d.id for d in req.drones if d.id not in score]


def _subsets(req: PlanRequest, order: list[str]) -> list[list[str]]:
    """Наборы бортов для кривой: префиксы порядка. При большом парке — с шагом, чтобы
    уложиться в MAX_POINTS планов."""
    n = len(order)
    if n <= MAX_POINTS:
        counts = list(range(1, n + 1))
    else:
        step = math.ceil(n / MAX_POINTS)
        counts = sorted({*range(1, n + 1, step), n})
    return [order[:k] for k in counts]


def advise(
    req: PlanRequest,
    deadline_s: float | None = None,
    max_drones: int | None = None,
    workers: int | None = None,
) -> AdviceResponse:
    """Кривая «время работ от числа бортов» и подсказки по сроку и числу бортов."""
    order = _fleet_order(req)
    if max_drones is not None:
        order = order[:max_drones]
    if not order:
        raise PlanningError("в парке нет бортов")
    data = req.model_dump()
    jobs = []
    for ids in _subsets(req, order):
        sub = dict(data, drones=[d for d in data["drones"] if d["id"] in set(ids)])
        jobs.append((sub, req.time_weight if req.makespan_cap_s is None else 1.0, req.makespan_cap_s))
    results: list[PlanResponse | None] = _run(jobs, workers)

    options: list[AdviceOption] = []
    best_so_far = math.inf
    for ids, res in zip(_subsets(req, order), results):
        if res is None:
            options.append(AdviceOption(drones=len(ids), drone_ids=ids, feasible=False,
                                        reason="план не строится этим набором бортов"))
            continue
        s = res.summary
        covers = s.coverage_pct >= COVERAGE_MIN_PCT
        if covers:
            best_so_far = min(best_so_far, s.makespan_s)
        options.append(AdviceOption(
            drones=s.drones_used, drone_ids=[d.drone_id for d in res.drones if d.sorties],
            makespan_s=s.makespan_s, total_flight_s=s.total_flight_s, sorties=s.sorties,
            coverage_pct=s.coverage_pct,
            best_makespan_s=None if best_so_far == math.inf else best_so_far,
            feasible=covers and (deadline_s is None or s.makespan_s <= deadline_s),
            reason="" if covers else f"область снята на {s.coverage_pct:.1f} % — этому набору бортов "
                                     f"её целиком не покрыть",
        ))
    done = [o for o in options if o.makespan_s is not None and not o.reason]
    if not done:
        short = next((o for o in options if o.makespan_s is not None), None)
        raise PlanningError(
            "ни один набор бортов не снимает область целиком"
            + (f" (лучшее покрытие {short.coverage_pct:.1f} %)" if short and short.coverage_pct else "")
        )

    best = min(done, key=lambda o: o.makespan_s or math.inf)
    advice = AdviceResponse(
        options=options,
        deadline_s=deadline_s,
        best_makespan_s=best.makespan_s,
        best_drones=best.drones,
        best_drone_ids=best.drone_ids,
        min_total_flight_s=min((o.total_flight_s for o in done if o.total_flight_s is not None), default=None),
    )
    if deadline_s is not None:
        fit = next((o for o in done if (o.makespan_s or math.inf) <= deadline_s), None)
        advice.feasible = fit is not None
        if fit is not None:
            advice.drones_needed = fit.drones
            advice.drones_needed_ids = fit.drone_ids
            advice.message = (
                f"Уложиться в срок {_hm(deadline_s)} можно: нужно {fit.drones} "
                f"{_plural(fit.drones)} ({', '.join(fit.drone_ids)}), работы займут {_hm(fit.makespan_s or 0)}."
            )
        else:
            advice.message = (
                f"В срок {_hm(deadline_s)} задание не выполнить: всем парком "
                f"({best.drones} {_plural(best.drones)}) работы займут {_hm(best.makespan_s or 0)}. "
                "Нужны дополнительные борта, ВПП ближе к области или менее жёсткие требования к съёмке."
            )
    else:
        advice.message = (
            f"Быстрее всего — {best.drones} {_plural(best.drones)}: {_hm(best.makespan_s or 0)}. "
            f"Наименьшее число бортов, которым область снимается целиком — {done[0].drones} "
            f"({_hm(done[0].makespan_s or 0)})."
        )
    return advice


def _plural(n: int) -> str:
    if n % 10 == 1 and n % 100 != 11:
        return "борт"
    if n % 10 in (2, 3, 4) and n % 100 not in (12, 13, 14):
        return "борта"
    return "бортов"


def _hm(seconds: float) -> str:
    h, m = divmod(int(round(seconds / 60)), 60)
    return f"{h} ч {m:02d} мин" if h else f"{m} мин"
