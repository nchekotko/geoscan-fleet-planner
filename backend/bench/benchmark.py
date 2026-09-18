"""Бенчмарк планировщика: базовые подходы, нижняя граница, потери времени, случайный набор,
масштабируемость.

Базовые подходы (та же модель полёта, но без оптимизации распределения работы):
  single   — вся область одному борту (лучшему из тех, кто снимает её полностью);
  equal    — область делится поровну без учёта ТТХ;
  operator — полосы пропорционально скорости × шагу галсов (ручное деление оператором).
Наши планы: критерий «время работ» (w = 1) и «суммарный налёт» (w = 0).

Запуск из каталога backend:
    python bench/benchmark.py                       # N = 30, seed = 1 → docs/benchmark.md
    python bench/benchmark.py --random 100 --seed 7 --out /tmp/b.md
Сценарии, на которых планировщик упал или дал заведомо плохой план, сохраняются
в data/scenarios_failures/ (их можно открыть в интерфейсе или передать в /api/plan).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from bench.core import COVERAGE_OK, ROW_ORDER, CaseResult, PlanStats, run_case, run_random_case  # noqa: E402
from bench.lower_bound import MIN_LAUNCH_INTERVAL_S, lower_bound  # noqa: E402
from bench.random_scenarios import case_seeds  # noqa: E402
from planner.planner import Planner, plan  # noqa: E402
from planner.schemas import PlanRequest  # noqa: E402

SC = ROOT / "data" / "scenarios"
FAIL_DIR = ROOT / "data" / "scenarios_failures"
DOCS = ROOT.parent / "docs" / "benchmark.md"
SCENARIOS = ("demo_basic", "geophysics_401", "lidar_401", "strong_wind", "large_mixed_fleet")
BASELINES = (("single", "один лучший борт"), ("equal", "поровну без учёта ТТХ"), ("operator", "оператор"))
TIE = 0.005  # ±0,5 % по времени работ — ничья (точность модели времени)
GAP_SUSPICIOUS = 2.5  # T / LB больше — план проверяется вручную (сохраняется в находки)
SCALE_AREAS = (1, 4, 16, 64, 144, 400)
SCALE_FLEETS = (2, 4, 8, 16)
SCALE_BUDGET_S = 60.0  # строка 400 км² считается, если по прогнозу укладывается в это время


# ---------------------------------------------------------------- форматирование
def num(x: float, nd: int = 1) -> str:
    return "—" if x is None or not math.isfinite(x) else f"{x:.{nd}f}".replace(".", ",")


def mins(s: float) -> str:
    return num(s / 60, 1)


def times(x: float) -> str:
    return "—" if not math.isfinite(x) else f"{num(x, 2)}×"


def pct(x: float, nd: int = 0) -> str:
    return "—" if not math.isfinite(x) else f"{num(100 * x, nd)} %"


def cov(st: PlanStats) -> str:
    return "—" if not st.ok else num(st.coverage_pct, 2) + ("" if st.full else " †")


def quart(xs: list[float]) -> tuple[float, float, float]:
    xs = sorted(x for x in xs if math.isfinite(x))
    if not xs:
        return math.nan, math.nan, math.nan
    if len(xs) == 1:
        return xs[0], xs[0], xs[0]
    q1, q2, q3 = statistics.quantiles(xs, n=4, method="inclusive")
    return q1, q2, q3


def median(xs: list[float]) -> float:
    xs = [x for x in xs if math.isfinite(x)]
    return statistics.median(xs) if xs else math.nan


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


# ---------------------------------------------------------------- проверки
def lb_violations(cases: list[CaseResult]) -> tuple[int, int, list[str]]:
    """Сколько полных планов (любых) быстрее нижней границы: должно быть 0."""
    checked, bad = 0, []
    for c in cases:
        if not math.isfinite(c.lb_int_s):
            continue
        for key, st in c.rows.items():
            if st.full:
                checked += 1
                if st.makespan_s < c.lb_int_s - 1.0:
                    bad.append(f"{c.name}/{key}: T = {mins(st.makespan_s)} мин < LB = {mins(c.lb_int_s)} мин")
    return checked, len(bad), bad


def findings(c: CaseResult) -> list[str]:
    """Ошибки и заведомо плохие планы нашего планировщика в этом сценарии."""
    out = []
    if c.error:
        out.append(f"исключение при подготовке задачи: {c.error}")
    for key in ("w1", "w0"):
        st = c.rows.get(key)
        if st is None:
            continue
        if st.error:
            out.append(f"исключение в плане {key}: {st.error}")
        elif not st.full and not st.note:
            out.append(f"план {key} снимает {num(st.coverage_pct, 2)} % без предупреждения о неснятой площади")
    w1, w0 = c.rows.get("w1"), c.rows.get("w0")
    if w1 and w0 and w1.full and w0.full and w0.makespan_s < w1.makespan_s * (1 - TIE):
        out.append(f"план w=1 ({mins(w1.makespan_s)} мин) медленнее плана w=0 ({mins(w0.makespan_s)} мин)")
    for key, name in BASELINES:
        r = c.ratio(key)
        if math.isfinite(r) and r < 1 - TIE:
            out.append(f"план w=1 ({mins(w1.makespan_s)} мин) медленнее базового «{name}» "
                       f"({mins(c.rows[key].makespan_s)} мин)")
    g = c.gap()
    if math.isfinite(g) and g > GAP_SUSPICIOUS:
        f = w1.finish_s
        out.append(f"T / LB = {num(g, 2)}: борта заканчивают работу через {mins(min(f))}–{mins(max(f))} мин "
                   "(проверить балансировку)")
    return out


# ---------------------------------------------------------------- 1. сценарии
def scenario_cases() -> list[CaseResult]:
    out = []
    for n in SCENARIOS:
        log(f"… сценарий {n}")
        out.append(run_case(n, json.loads((SC / f"{n}.json").read_text(encoding="utf-8"))))
    return out


def section_scenarios(cases: list[CaseResult]) -> list[str]:
    L = ["## 1. Пять сценариев из data/scenarios", "",
         "### Сводка: во сколько раз базовый подход дольше нашего плана (w = 1)", "",
         "| Сценарий | Площадь, км² | Бортов | Ветер, м/с | T наш, мин | Один лучший борт | Поровну | Оператор | T / LB |",
         "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for c in cases:
        w1 = c.rows["w1"]
        cells = []
        for key, _ in BASELINES:
            r_all = c.ratio(key, strict=False)
            cells.append(times(r_all) + ("" if math.isfinite(c.ratio(key)) or not math.isfinite(r_all) else " †"))
        L.append(f"| {c.name} | {num(c.area_km2, 2)} | {c.n_drones} | {num(c.wind_ms, 0)} | {mins(w1.makespan_s)} | "
                 + " | ".join(cells) + f" | {num(c.gap(), 2)} |")
    for strict, label in ((True, "Медиана, только полные планы"), (False, "Медиана, все планы")):
        cells = [times(median([c.ratio(k, strict=strict) for c in cases])) for k, _ in BASELINES]
        n = [sum(math.isfinite(c.ratio(k, strict=strict)) for c in cases) for k, _ in BASELINES]
        cells = [f"{x} (n = {m})" for x, m in zip(cells, n)]
        L.append(f"| **{label}** | | | | | " + " | ".join(cells) + f" | {num(median([c.gap() for c in cases]), 2)} |")
    L += ["", "† — хотя бы один из двух планов снимает меньше 99,5 % рабочей области: отношение показано, "
          "но в медиану «только полные планы» не входит.", ""]
    L += ["### Подробно", ""]
    for c in cases:
        w1 = c.rows["w1"]
        L += [f"#### {c.name}", "",
              "| Подход | Время работ, мин | Налёт, мин | Покрытие, % | Бортов / вылетов | T / T_наш | Разбиение |",
              "|---|---:|---:|---:|---:|---:|---|"]
        for key in ROW_ORDER:
            st = c.rows.get(key)
            if st is None:
                continue
            if not st.ok:
                L.append(f"| {st.label} | не построен | — | — | — | — | {st.error[:80]} |")
                continue
            r = st.makespan_s / w1.makespan_s if w1.ok else math.nan
            mode = "полосы" if st.mode == "strips" else "сетка (радиус действия)"
            note = f"; {st.note}" if st.note and key == "single" else ""
            L.append(f"| {st.label} | {mins(st.makespan_s)} | {mins(st.total_s)} | {cov(st)} | "
                     f"{st.drones_used} / {st.sorties} | {times(r)} | {mode}{note} |")
        L.append("")
    return L


# ---------------------------------------------------------------- 2. граница
def section_bound(cases: list[CaseResult], checked: int, violations: int, bad: list[str]) -> list[str]:
    L = ["## 2. Нижняя граница времени работ и разрыв до неё", "",
         "Граница (LB) — релаксация модели планировщика: ни один план, который может построить эта модель "
         "(не только наш), не завершит работы быстрее LB. Поэтому T / LB — верхняя оценка того, насколько "
         "наш план может быть хуже оптимального; сам оптимум лежит между LB и T. Код — `bench/lower_bound.py`.",
         "",
         "**Что взято из модели без изменений.** Вылет борта i не длиннее бюджета B_i (паспортная "
         "продолжительность × (1 − резерв) × поправка на ветер); между вылетами — смена АКБ t_swap,i; в каждом "
         "вылете взлёт и посадка O_i (набор высоты, у самолёта ещё катапульта и парашют — `SortieBuilder`); "
         "галсы одного борта параллельны, идут с шагом s_i и воздушной скоростью V_i, путевая скорость — "
         "по треугольнику скоростей; чтобы снять площадь a, нужно пройти не меньше a / s_i метров галсов "
         "(у геофизики ещё a / s_tie,i секущих); вылет начинается и заканчивается на базе; с одной ВПП борта "
         "стартуют по очереди.",
         "",
         "**Что ослаблено (граница от этого только ниже).** Развороты бесплатны; перелёты прямые, без обхода "
         "запретных зон; каждый борт может выбрать свой угол галсов и любую доступную базу; интервал между "
         f"стартами — наименьший в модели ({num(MIN_LAUNCH_INTERVAL_S / 60, 0)} мин), порядок стартов любой; радиус действия (заряд, "
         "радиоканал) ограничивает только перелёт до ближайшей точки области, а не то, какую часть области борт "
         "может снять; геометрия разбиения не учитывается.",
         "",
         "**Формула без ветра.** За полный вылет борт снимает не больше",
         "",
         "    a_i = s_i · V_i · (B_i − O_i − R_i),   R_i = 2·d_i / V_тр,i,",
         "",
         "где d_i — расстояние от базы до ближайшей точки области, V_тр,i — транзитная скорость (у геофизики "
         "a_i ещё делится на 1 + s_i / s_tie,i). Последний вылет не требует смены АКБ после себя, поэтому за "
         "время T борт снимет не больше a_i · (T + t_swap,i) / (B_i + t_swap,i). Сумма по бортам должна "
         "покрыть площадь A:",
         "",
         "    T_непр = (A − Σ q_i · t_swap,i) / Σ q_i,   q_i = a_i / (B_i + t_swap,i).",
         "",
         "**Ветер.** Галсы по ветру быстрее, против — медленнее, но вылет замкнут: смещение Δ = S₊ − S₋ "
         "(разность длин галсов в двух направлениях) должно вернуть остальное горизонтальное время T_o "
         "(перелёты, развороты): T_o ≥ |Δ| / ρ, где ρ — наибольшая проекция путевой скорости на обратное "
         "направление (V_тр − w·e; у самолёта не меньше V — в модели время его разворота от ветра не зависит). "
         "Кроме того, T_o ≥ R_i — время прямого перелёта до ближайшей по времени точки области и обратно "
         "по треугольнику скоростей. Время вылета f(S) = min по Δ [S₊/g₊ + S₋/g₋ + max(|Δ|/ρ, R_i)] + O_i "
         "— минимум кусочно-линейной выпуклой функции, он достигается при Δ ∈ {0, min(ρR_i, S), S}. "
         "Наибольшая длина галсов S при f(S) ≤ B_i, умноженная на s_i, даёт a_i. Если галсы идут парами "
         "туда-обратно (Δ = 0), средняя путевая скорость на паре не выше sqrt(V² − w²) < V, так что ветер "
         "уменьшает a_i; перекос Δ > 0 выгоден, только если перелёт на транзитной скорости возвращает смещение "
         "быстрее, чем галс против ветра, — граница учитывает и такой план.",
         "",
         "**Целые вылеты.** Отношение S(H)/H не убывает, и при фиксированном угле S(H) вогнута, "
         "поэтому из n вылетов выгоднее всего равные: cap_i(t) = max_θ max_n n · s_i · S_θ(min(Y_n / n, B_i − O_i)), "
         "Y_n = t − n·O_i − (n − 1)·t_swap,i. Граница «без очереди» — наименьшее T, при котором Σ cap_i(T) ≥ A "
         "(бисекция). Она не меньше непрерывной и заметно точнее, когда вылетов мало (самолёт с 2,4-часовым "
         "вылетом на задаче на час).",
         "",
         "**Очередь стартов (столбец «LB», основная граница).** С одной ВПП борта взлетают по очереди, k-й — "
         f"не раньше k · Δ (Δ = {num(MIN_LAUNCH_INTERVAL_S, 0)} с), и работает T − k · Δ. Какие борта с какой базы и в каком порядке "
         "стартуют — задача о назначениях «борт → место в очереди» с весом cap_i(T − k · Δ); её точный максимум "
         "(венгерский алгоритм) должен покрыть A. На больших парках с одной ВПП это главный вклад в границу.",
         "",
         "Угол θ перебирается с шагом 0,25° (плюс оси ветра), на погрешность перебора ёмкость увеличена на 0,5 %; "
         "R_i ищется по точкам границы области через 10 м с поправкой на шаг — граница остаётся нижней.",
         "",
         "| Сценарий | T наш (w = 1), мин | LB непрерывная, мин | LB целые вылеты без очереди, мин | LB, мин | "
         "T / LB | Σ L·s / A |",
         "|---|---:|---:|---:|---:|---:|---:|"]
    for c in cases:
        w1 = c.rows["w1"]
        ls = w1.lines_x_spacing_m2 / (c.area_km2 * 1e6) if w1.ok and c.area_km2 else math.nan
        L.append(f"| {c.name} | {mins(w1.makespan_s)} | {mins(c.lb_cont_s)} | {mins(c.lb_noqueue_s)} | "
                 f"{mins(c.lb_int_s)} | {num(c.gap(), 2)} | {num(ls, 3)} |")
    L += ["",
          "Σ L·s / A — длина основных галсов нашего плана × шаг / площадь: проверка допущения «площадь a требует "
          "a / s метров галсов» (≈ 1; больше 1 — перекрытие на стыках полос и у границы).",
          "",
          f"Проверка корректности: LB не больше времени работ ни одного из {checked} полных планов "
          f"(наших и базовых, во всех разделах бенчмарка); нарушений: **{violations}**."]
    L += [f"- {b}" for b in bad[:10]]
    demo = next((c for c in cases if c.name == "demo_basic"), None)
    if demo and demo.lb_drones:
        L += ["", "Параметры границы для demo_basic:", "",
              "| Борт | База | B, мин | O, с | R, с | t_swap, мин | Площадь за полный вылет, км² |",
              "|---|---|---:|---:|---:|---:|---:|"]
        for i, base, b, o, r, sw, a in demo.lb_drones:
            L.append(f"| {i} | {base} | {num(b / 60, 1)} | {num(o, 0)} | {num(r, 0)} | {num(sw / 60, 0)} | "
                     f"{num(a / 1e6, 2)} |")
    L += ["",
          "Откуда разрыв. Граница не платит за развороты (7–20 % налёта, см. раздел 3), за непрямые перелёты "
          "к дальним полосам и обходы зон, за полный интервал стартов самолёта (в модели он больше Δ), "
          "за то, что участок борта — полоса или клетки, а не «любая доля площади рядом с базой», "
          "и за неполный последний вылет. На малых областях доля этих постоянных потерь больше, отсюда и "
          "больший разрыв.", ""]
    return L


# ---------------------------------------------------------------- 3. потери
def section_waterfall(cases: list[CaseResult]) -> list[str]:
    L = ["## 3. Куда уходит время: доли налёта по видам участков", "",
         "Доли — от суммарного налёта всех бортов. «Перелёты» — транзит к полосам и возврат на базу, "
         "«холостой пробег» — их длина. Производительность парка — площадь / время работ.", "",
         "| Сценарий | План | Съёмка | Развороты | Перелёты | Взлёт и посадка | Налёт / время работ | км²/ч | "
         "Холостой пробег, км | Вылетов |",
         "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for c in cases:
        for key, label in (("w1", "наш, w = 1"), ("w0", "наш, w = 0"), ("operator", "оператор")):
            st = c.rows.get(key)
            if st is None or not st.ok:
                continue
            L.append(f"| {c.name} | {label} | {pct(st.share('survey'))} | {pct(st.share('turn'))} | "
                     f"{pct(st.share('transit'))} | {pct(st.share('ground'))} | {num(st.total_s / st.makespan_s, 2)} | "
                     f"{num(st.productivity_km2_h, 2)} | {num(st.deadhead_km, 1)} | {st.sorties} |")
    return L + [""]


# ---------------------------------------------------------------- 4. случайный набор
def run_random(n: int, seed: int, jobs: int) -> tuple[list[tuple[int, dict, CaseResult]], float]:
    seeds = case_seeds(seed, n)
    t = time.perf_counter()
    if jobs > 1 and n > 1:
        out = []
        with ProcessPoolExecutor(max_workers=min(jobs, n)) as ex:
            for k, r in enumerate(ex.map(run_random_case, seeds), 1):
                out.append(r)
                log(f"… случайный сценарий {k}/{n} (seed {r[0]}): {r[2].runtime_s:.1f} с")
    else:
        out = []
        for k, s in enumerate(seeds, 1):
            out.append(run_random_case(s))
            log(f"… случайный сценарий {k}/{n} (seed {s}): {out[-1][2].runtime_s:.1f} с")
    return out, time.perf_counter() - t


def save_findings(items: list[tuple[int, dict, CaseResult]], seed: int, fail_dir: Path) -> list[tuple[int, str, str]]:
    """Сценарии с ошибками или заведомо плохими планами → data/scenarios_failures/*.json."""
    saved = []
    for s, data, c in items:
        f = findings(c)
        if not f:
            continue
        fail_dir.mkdir(parents=True, exist_ok=True)
        path = fail_dir / f"random_seed{seed}_{s}.json"
        body = {**data, "_comment": data.get("_comment", "") + " | находки бенчмарка: " + "; ".join(f)}
        path.write_text(json.dumps(body, ensure_ascii=False, indent=1), encoding="utf-8")
        try:
            shown = path.resolve().relative_to(ROOT).as_posix()
        except ValueError:
            shown = path.as_posix()
        saved.append((s, "; ".join(f), shown))
    return saved


def section_random(items: list[tuple[int, dict, CaseResult]], n: int, seed: int, wall_s: float, jobs: int,
                   saved: list[tuple[int, str, str]]) -> list[str]:
    cases = [c for _, _, c in items]
    built = [c for c in cases if not c.error]
    L = [f"## 4. Случайный набор: N = {n}, seed = {seed}", "",
         "Генератор — `bench/random_scenarios.py`: звёздный (невыпуклый) или выпуклый многоугольник 2–30 км² "
         "в Московском регионе (долгота 37–38°, широта 55–56°), иногда вытянутый до 2,5:1; 0–2 запретные "
         "зоны внутри области; 1–2 базы в 0,3–2 км от границы; 2–6 бортов из geoscan_201 / geoscan_gemini / "
         "geoscan_801 / geoscan_401 (70 % привязаны к базе, остальные — к ближайшей); RGB-съёмка с GSD "
         "3–5 см; ветер 0–8 м/с с любого направления. Каждый сценарий восстанавливается по своему seed "
         "(`random_request(seed)`), список seed'ов — в таблице ниже.", "",
         f"Время прогона набора: {num(wall_s / 60, 1)} мин (пул из {jobs} процессов; сумма по сценариям "
         f"{num(sum(c.runtime_s for c in cases) / 60, 1)} мин процессорного времени). Медиана времени расчёта "
         f"нашего плана w = 1 на сценарий — {num(median([c.rows['w1'].compute_s for c in built if 'w1' in c.rows]), 1)} с "
         "(в пуле, под нагрузкой).", "",
         "### Отношения времени работ", "",
         "| Показатель | Медиана | Q1–Q3 | Мин–макс | Сценариев |", "|---|---:|---:|---:|---:|"]

    def row(label: str, xs: list[float]) -> str:
        xs = [x for x in xs if math.isfinite(x)]
        q1, q2, q3 = quart(xs)
        mm = f"{num(min(xs), 2)}–{num(max(xs), 2)}" if xs else "—"
        return f"| {label} | {num(q2, 2)} | {num(q1, 2)}–{num(q3, 2)} | {mm} | {len(xs)} |"

    for key, name in BASELINES:
        L.append(row(f"T «{name}» / T наш (w = 1)", [c.ratio(key) for c in built]))
    L.append(row("T наш (w = 1) / LB", [c.gap() for c in built if c.rows.get("w1") and c.rows["w1"].full]))
    L.append(row("налёт «один лучший борт» / налёт наш (w = 0)",
                 [c.rows["single"].total_s / c.rows["w0"].total_s for c in built
                  if c.rows.get("single") and c.rows.get("w0") and c.rows["single"].full and c.rows["w0"].full]))
    L += ["", "Отношение > 1 — наш план быстрее (или, в последней строке, с меньшим налётом). В сравнение входят "
          "только пары, где оба плана построены и снимают ≥ 99,5 % области.", "",
          "### Победы", "",
          "| Против | Наш план быстрее | Ничья (±0,5 %) | Наш план медленнее | Не сравнить |", "|---|---:|---:|---:|---:|"]
    for key, name in BASELINES:
        rs = [c.ratio(key) for c in built]
        ok = [r for r in rs if math.isfinite(r)]
        win = sum(r > 1 + TIE for r in ok)
        tie = sum(abs(r - 1) <= TIE for r in ok)
        loss = sum(r < 1 - TIE for r in ok)
        m = max(len(ok), 1)
        L.append(f"| {name} | {win} ({pct(win / m)}) | {tie} ({pct(tie / m)}) | {loss} ({pct(loss / m)}) | "
                 f"{len(cases) - len(ok)} |")
    L += ["", "«Не сравнить» — один из планов не построен или снимает меньше 99,5 %. Генератор не проверяет, "
          "что парк вообще может снять область: при сильном ветре и далёких базах часть области бывает вне "
          "радиуса действия всех бортов — такой сценарий невыполним любым планом, планировщик сообщает об этом "
          "предупреждением (столбец «Неполных, о которых предупредил планировщик»).", "",
          "### Покрытие и отказы", "",
          "| План | Построен | Покрытие ≥ 99,5 % | Неполных, о которых предупредил планировщик | Медиана покрытия, % |",
          "|---|---:|---:|---:|---:|"]
    for key in ROW_ORDER:
        sts = [c.rows[key] for c in built if key in c.rows]
        okk = [s for s in sts if s.ok]
        full = [s for s in okk if s.full]
        label = sts[0].label if sts else key
        if key == "single":
            label = "один лучший борт"
        warned = sum(1 for s in okk if not s.full and s.note) if key in ("w1", "w0") else math.nan
        L.append(f"| {label} | {len(okk)} из {len(cases)} | {len(full)} ({pct(len(full) / max(len(okk), 1))}) | "
                 f"{'—' if not math.isfinite(warned) else f'{warned} из {len(okk) - len(full)}'} | "
                 f"{num(median([s.coverage_pct for s in okk]), 2)} |")
    exc = [(s, c) for s, _, c in items if c.error or any(c.rows.get(k) and c.rows[k].error for k in ("w1", "w0"))]
    L += ["", f"Исключений в нашем планировщике: **{len(exc)}** из {len(cases)} сценариев."]
    for s, c in exc[:5]:
        msg = c.error or next(c.rows[k].error for k in ("w1", "w0") if c.rows.get(k) and c.rows[k].error)
        L.append(f"- seed {s}: `{msg[:200]}`")
    base_err = [(s, k, c.rows[k].error) for s, _, c in items for k, _ in BASELINES if c.rows.get(k) and c.rows[k].error]
    if base_err:
        L.append(f"\nБазовые подходы не построены в {len(base_err)} случаях, например: "
                 + "; ".join(f"seed {s} / {k}: `{e[:120]}`" for s, k, e in base_err[:3]))
    L += ["", f"Находки (сценарий сохранён в `data/scenarios_failures/`): **{len(saved)}**."]
    for s, f, path in saved:
        L.append(f"- seed {s}: {f} — `{path}`")
    L += ["", "<details><summary>Все сценарии набора</summary>", "",
          "| seed | Площадь, км² | NFZ | Баз | Бортов | Ветер, м/с | T наш w=1, мин | Покр., % | "
          "Один лучший | Поровну | Оператор | T / LB | Расчёт w=1, с |",
          "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for s, data, c in items:
        if c.error:
            L.append(f"| {s} | — | — | — | — | — | ошибка: {c.error[:60]} | | | | | | |")
            continue
        w1 = c.rows.get("w1")
        cells = []
        for key, _ in BASELINES:
            r = c.ratio(key, strict=False)
            cells.append(times(r) + ("" if math.isfinite(c.ratio(key)) or not math.isfinite(r) else " †"))
        L.append(f"| {s} | {num(c.area_km2, 1)} | {len(data['no_fly_zones'])} | {len(data['bases'])} | {c.n_drones} | "
                 f"{num(c.wind_ms, 1)} | {mins(w1.makespan_s) if w1 and w1.ok else 'ошибка'} | "
                 f"{cov(w1) if w1 else '—'} | " + " | ".join(cells) + f" | {num(c.gap(), 2)} | "
                 f"{num(w1.compute_s, 1) if w1 else '—'} |")
    L += ["", "</details>", ""]
    return L


# ---------------------------------------------------------------- 5. масштабируемость
def scale_request(area_km2: float, n: int) -> dict:
    """Квадратная область, парк поровну из 201 и Gemini; до 16 км² — одна база, дальше — две
    (у южного и северного края), борта чередуются по базам."""
    lat0, lon0 = 55.61, 37.62
    side = math.sqrt(area_km2)
    k_lon = side / (111.32 * math.cos(math.radians(lat0))) / 2
    k_lat = side / 110.57 / 2
    area = {"type": "Polygon", "coordinates": [[
        [lon0 - k_lon, lat0 - k_lat], [lon0 + k_lon, lat0 - k_lat], [lon0 + k_lon, lat0 + k_lat],
        [lon0 - k_lon, lat0 + k_lat], [lon0 - k_lon, lat0 - k_lat]]]}
    bases = [{"id": "A", "lon": lon0, "lat": lat0 - k_lat - 0.005}]
    if area_km2 > 16:
        bases.append({"id": "B", "lon": lon0, "lat": lat0 + k_lat + 0.005})
    drones = [{"id": f"d{i}", "model": "geoscan_201" if i % 2 == 0 else "geoscan_gemini",
               "base_id": bases[(i // 2) % len(bases)]["id"]} for i in range(n)]
    return {"survey_area": area, "bases": bases, "drones": drones, "survey_type": "rgb",
            "requirements": {"gsd_cm": 4}, "wind": {"speed_ms": 4, "from_deg": 270},
            "time_weight": 1.0, "use_terrain": False}


def run_scale(max_area: float) -> tuple[list[dict], float, str]:
    rows, note = [], ""
    t0 = time.perf_counter()
    row_time: dict[float, float] = {}
    for a in SCALE_AREAS:
        if a > max_area:
            continue
        if a == 400 and 144 in row_time:
            est = row_time[144] * 400 / 144
            if est > SCALE_BUDGET_S:
                note = (f"Строка 400 км² пропущена: по времени строки 144 км² ({num(row_time[144], 1)} с) "
                        f"прогноз {num(est, 0)} с — больше {num(SCALE_BUDGET_S, 0)} с.")
                continue
        for n in SCALE_FLEETS:
            data = scale_request(a, n)
            req = PlanRequest(**data)
            t = time.perf_counter()
            r = plan(req)
            dt = time.perf_counter() - t
            row_time[a] = row_time.get(a, 0.0) + dt
            lb = lower_bound(Planner(req))
            rows.append({"area": a, "n": n, "bases": len(data["bases"]), "compute_s": dt,
                         "makespan_s": r.summary.makespan_s, "sorties": r.summary.sorties,
                         "used": r.summary.drones_used, "coverage": r.summary.coverage_pct,
                         "grid": any("с учётом дальности" in w for w in r.warnings),
                         "gap": r.summary.makespan_s / lb.integer_s})
            log(f"… масштабируемость {a} км², {n} бортов: {dt:.1f} с")
    return rows, time.perf_counter() - t0, note


def section_scale(rows: list[dict], total_s: float, note: str) -> list[str]:
    L = ["## 5. Масштабируемость", "",
         f"Квадратная область, парк поровну из Геоскан 201 и Gemini, ветер 4 м/с, GSD 4 см, w = 1; до 16 км² — "
         f"одна база у южного края, дальше — две (южный и северный край). Время расчёта — `plan()` целиком "
         f"(мультистарт по углу, балансировка, перебор порядка, проверки), без рельефа, считается "
         f"последовательно в одном процессе; машина — {os.cpu_count()} логических ядер (планировщик использует "
         f"одно, пул процессов нужен только фронту Парето). Весь раздел — {num(total_s, 0)} с.", "",
         "| Площадь, км² | Бортов | Баз | Задействовано | Разбиение | Вылетов | Время работ, ч | Покрытие, % | "
         "T / LB | Время расчёта, с |",
         "|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|"]
    for r in rows:
        L.append(f"| {r['area']} | {r['n']} | {r['bases']} | {r['used']} | {'сетка' if r['grid'] else 'полосы'} | "
                 f"{r['sorties']} | {num(r['makespan_s'] / 3600, 2)} | {num(r['coverage'], 1)} | {num(r['gap'], 2)} | "
                 f"{num(r['compute_s'], 2)} |")
    # план с большим парком не должен быть медленнее плана с его частью: лишние борта можно не брать
    worse = []
    by = {(r["area"], r["n"]): r for r in rows}
    for (a, n), r in sorted(by.items()):
        prev = by.get((a, n // 2))
        if prev and r["makespan_s"] > prev["makespan_s"] * (1 + TIE):
            worse.append(f"{a} км²: {n} бортов — {num(r['makespan_s'] / 3600, 2)} ч, {n // 2} бортов — "
                         f"{num(prev['makespan_s'] / 3600, 2)} ч")
    if worse:
        L += ["", "Недостаток планировщика: парк вдвое больше (он включает меньший) даёт план медленнее — "
              "лишние борта можно было не задействовать: " + "; ".join(worse) + "."]
    if note:
        L += ["", note]
    return L + [""]


# ---------------------------------------------------------------- итоги
def section_summary(sc: list[CaseResult], items, rows: list[dict], total_s: float) -> list[str]:
    rnd = [c for _, _, c in items if not c.error]
    L = ["## Итоги", ""]
    parts = []
    for key, name in BASELINES:
        rs = [c.ratio(key) for c in sc]
        parts.append(f"«{name}» — {times(median(rs))} (сравнимых сценариев {sum(math.isfinite(r) for r in rs)} из {len(sc)})")
    L.append("- Пять сценариев, медиана T базового подхода / T нашего плана w = 1: " + "; ".join(parts) + ".")
    if rnd:
        op = [c.ratio("operator") for c in rnd]
        sg = [c.ratio("single") for c in rnd]
        q_op, q_sg = quart(op), quart(sg)
        ok_op = [r for r in op if math.isfinite(r)]
        L.append(f"- Случайный набор ({len(rnd)} сценариев): наш план w = 1 быстрее «оператора» в "
                 f"{num(q_op[1], 2)} раза по медиане (Q1–Q3 {num(q_op[0], 2)}–{num(q_op[2], 2)}), быстрее более чем на "
                 f"0,5 % — в {pct(sum(r > 1 + TIE for r in ok_op) / max(len(ok_op), 1))} сравнимых сценариев; "
                 f"быстрее «одного лучшего борта» в {num(q_sg[1], 2)} раза (Q1–Q3 {num(q_sg[0], 2)}–{num(q_sg[2], 2)}).")
        full = sum(1 for c in rnd if c.rows.get("w1") and c.rows["w1"].full)
        warned = sum(1 for c in rnd if c.rows.get("w1") and c.rows["w1"].ok and not c.rows["w1"].full
                     and c.rows["w1"].note)
        L.append(f"- Покрытие ≥ 99,5 % у плана w = 1 — в {full} из {len(items)} случайных сценариев; ещё в {warned} "
                 "часть области не снята, и планировщик об этом предупреждает (вне радиуса действия всего парка "
                 "или у запретных зон, где самолёту негде развернуться).")
        gq = quart([c.gap() for c in rnd if c.rows.get("w1") and c.rows["w1"].full])
        L.append(f"- Разрыв до нижней границы T / LB: медиана {num(gq[1], 2)} (Q1–Q3 {num(gq[0], 2)}–{num(gq[2], 2)}) "
                 "на случайном наборе, " + ", ".join(f"{c.name} {num(c.gap(), 2)}" for c in sc) + " на сценариях.")
    if rows:
        big = max(rows, key=lambda r: (r["area"], r["n"]))
        slow = max(rows, key=lambda r: r["compute_s"])
        L.append(f"- Масштабируемость: самая большая задача ({big['area']} км², {big['n']} бортов) — "
                 f"{num(big['compute_s'], 1)} с; самая долгая в сетке — {num(slow['compute_s'], 1)} с "
                 f"({slow['area']} км², {slow['n']} бортов).")
    L.append(f"- Полный прогон бенчмарка — {num(total_s / 60, 1)} мин.")
    return L + [""]


def main() -> None:
    ap = argparse.ArgumentParser(description="Бенчмарк планировщика → docs/benchmark.md")
    ap.add_argument("--random", type=int, default=30, help="число случайных сценариев (0 — без набора)")
    ap.add_argument("--seed", type=int, default=1, help="seed случайного набора")
    ap.add_argument("--jobs", type=int, default=min(8, os.cpu_count() or 1), help="процессов для случайного набора")
    ap.add_argument("--max-area", type=float, default=400, help="наибольшая площадь в сетке масштабируемости, км²")
    ap.add_argument("--no-scale", action="store_true", help="без раздела масштабируемости")
    ap.add_argument("--out", type=Path, default=DOCS, help="куда записать Markdown")
    ap.add_argument("--failures-dir", type=Path, default=FAIL_DIR, help="куда сохранять сценарии с находками")
    args = ap.parse_args()

    t0 = time.perf_counter()
    sc = scenario_cases()
    items, wall = run_random(args.random, args.seed, args.jobs) if args.random > 0 else ([], 0.0)
    saved = save_findings(items, args.seed, args.failures_dir) if items else []
    scale_rows, scale_s, scale_note = ([], 0.0, "") if args.no_scale else run_scale(args.max_area)
    total = time.perf_counter() - t0

    checked, n_bad, bad = lb_violations(sc + [c for _, _, c in items])
    # масштабируемость: T / LB ≥ 1 проверяется там же
    checked += len(scale_rows)
    n_bad += sum(1 for r in scale_rows if r["gap"] < 1 - 1e-6)

    cmd = "python bench/benchmark.py" + ("" if (args.random, args.seed) == (30, 1) else
                                         f" --random {args.random} --seed {args.seed}")
    lines = ["# Бенчмарк", "",
             f"Сгенерировано `{cmd}` из каталога `backend`. Рельеф отключён (`use_terrain = false`), чтобы "
             f"результаты не зависели от сети и кэша DEM. Полный прогон — {num(total / 60, 1)} мин на машине "
             f"с {os.cpu_count()} логическими ядрами.", "",
             "## Как читать", "",
             "- Все планы — наши и базовые — считает одна и та же модель: полёт по треугольнику скоростей, "
             "развороты, нарезка на вылеты по заряду, смена АКБ, очередь стартов на ВПП. Базовые подходы "
             "отличаются только распределением работы: без балансировки долей, перебора угла галсов и порядка "
             "полос.",
             "- **Один лучший борт** — вся область одному борту; каждый борт пробуется со своим лучшим "
             "направлением галсов, берётся самый быстрый из снимающих ≥ 99,5 % области. Полосы у запретных зон, "
             "где самолёт не может развернуться, модель отдаёт свободному мультиротору — такие строки отмечены.",
             "- **Поровну без учёта ТТХ** — область режется на равные полосы по числу бортов (угол и порядок "
             "полос — как у «оператора»).",
             "- **Оператор** — полосы пропорциональны «скорость × шаг галсов» каждого борта: так делит область "
             "опытный оператор вручную в QGC или Geoscan Planner. Угол галсов — лучший для самого "
             "производительного борта (с него же начинает мультистарт нашего планировщика), порядок полос — "
             "по положению баз. Одна оценка, без балансировки. Если полоса выходит за радиус действия борта, "
             "то же деление делается сеткой с учётом дальности.",
             "- **Наш план** — `time_weight = 1` (минимум времени работ) и `time_weight = 0` (минимум "
             "суммарного налёта).",
             "- **Покрытие — ограничение, а не метрика.** План, снимающий меньше 99,5 % рабочей области, помечен †; "
             "в медианы и доли побед такие пары не входят.",
             "- **T / T_наш** — во сколько раз время работ подхода больше, чем у нашего плана w = 1 "
             "(> 1 — наш план быстрее). **T / LB** — во сколько раз наш план дольше нижней границы (раздел 2).",
             ""]
    lines += section_summary(sc, items, scale_rows, total)
    lines += section_scenarios(sc)
    lines += section_bound(sc, checked, n_bad, bad)
    lines += section_waterfall(sc)
    if items:
        lines += section_random(items, args.random, args.seed, wall, args.jobs, saved)
    if scale_rows:
        lines += section_scale(scale_rows, scale_s, scale_note)
    text = "\n".join(lines)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8")
    print(text)
    log(f"готово за {total / 60:.1f} мин → {args.out}; нарушений LB: {n_bad} из {checked}")


if __name__ == "__main__":
    main()
