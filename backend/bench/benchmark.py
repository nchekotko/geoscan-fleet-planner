"""Бенчмарк: сравнение с базовыми подходами и масштабируемость.

Базовые подходы:
  single — вся область одному самому производительному борту;
  equal  — область делится поровну между всеми бортами без учёта их ТТХ.
Наши планы: критерий «время работ» (w=1) и «суммарный налёт» (w=0).

Запуск: python bench/benchmark.py  → печатает таблицы в Markdown (docs/benchmark.md).
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from planner.planner import Planner, plan  # noqa: E402
from planner.schemas import PlanRequest  # noqa: E402

SC = Path(__file__).resolve().parent.parent / "data" / "scenarios"


def load(name: str, **over) -> PlanRequest:
    data = json.loads((SC / f"{name}.json").read_text(encoding="utf-8"))
    data.update(over)
    data["use_terrain"] = False
    return PlanRequest(**data)


def evaluate_fixed(req: PlanRequest, fractions_fn) -> tuple[float, float, float]:
    """План с заданными долями, без оптимизации. Возвращает (T_max, ΣT, покрытие %)."""
    p = Planner(req)
    cands, angle = p.prepare()
    fr = fractions_fn(cands)
    try:
        ev = p.evaluate(cands, fr, angle)
    except ValueError:
        p.mode = "grid"
        ev = p.evaluate(cands, fr, angle)
    cov = 100 * p._coverage(ev.results) / p.area.area
    return ev.makespan, ev.total, cov


def baselines(name: str) -> list[str]:
    req = load(name)
    rows = []

    def single(cands):
        best = max(range(len(cands)), key=lambda i: cands[i].productivity)
        return [1.0 if i == best else 0.0 for i in range(len(cands))]

    for label, fn in [("один лучший борт", single), ("поровну без учёта ТТХ", lambda c: [1.0] * len(c))]:
        try:
            t, s, c = evaluate_fixed(req, fn)
            rows.append((label, t, s, c))
        except ValueError as e:
            rows.append((label, math.nan, math.nan, 0.0))
            print(f"  {name}/{label}: {e}", file=sys.stderr)
    for label, w in [("наш план: время работ", 1.0), ("наш план: суммарный налёт", 0.0)]:
        r = plan(load(name, time_weight=w))
        rows.append((label, r.summary.makespan_s, r.summary.total_flight_s, r.summary.coverage_pct))
    t_best = rows[2][1]
    out = [f"### {name}", "", "| Подход | Время работ, мин | Суммарный налёт, мин | Покрытие, % | Время работ к нашему |",
           "|---|---:|---:|---:|---:|"]
    for label, t, s, c in rows:
        ratio = "—" if math.isnan(t) else f"{t / t_best:.2f}×"
        tt = "не выполним" if math.isnan(t) else f"{t / 60:.1f}"
        ss = "—" if math.isnan(s) else f"{s / 60:.1f}"
        out.append(f"| {label} | {tt} | {ss} | {c:.1f} | {ratio} |")
    return out + [""]


def scalability() -> list[str]:
    """Квадратная область растущей площади, парк из n бортов (201 + Gemini поровну)."""
    base = json.loads((SC / "strong_wind.json").read_text(encoding="utf-8"))
    out = ["### Масштабируемость", "", "| Площадь, км² | Бортов | Вылетов | Время расчёта, с |", "|---:|---:|---:|---:|"]
    lat0, lon0 = 55.61, 37.62
    for side_km in (1, 2, 4, 8):
        for n in (2, 4, 8):
            k_lon = side_km / (111.32 * math.cos(math.radians(lat0))) / 2
            k_lat = side_km / 110.57 / 2
            area = {"type": "Polygon", "coordinates": [[
                [lon0 - k_lon, lat0 - k_lat], [lon0 + k_lon, lat0 - k_lat], [lon0 + k_lon, lat0 + k_lat],
                [lon0 - k_lon, lat0 + k_lat], [lon0 - k_lon, lat0 - k_lat]]]}
            drones = [{"id": f"d{i}", "model": "geoscan_201" if i % 2 == 0 else "geoscan_gemini"} for i in range(n)]
            req = PlanRequest(**{**base, "survey_area": area, "drones": drones,
                                 "bases": [{"id": "A", "lon": lon0, "lat": lat0 - k_lat - 0.005}],
                                 "wind": {"speed_ms": 4, "from_deg": 270}, "use_terrain": False})
            t = time.perf_counter()
            r = plan(req)
            dt = time.perf_counter() - t
            out.append(f"| {side_km * side_km} | {n} | {r.summary.sorties} | {dt:.2f} |")
    return out + [""]


if __name__ == "__main__":
    lines = ["# Бенчмарк", "",
             "Сгенерировано `python bench/benchmark.py`. Рельеф отключён, чтобы результаты не зависели от сети.", ""]
    lines += ["## Сравнение с базовыми подходами", ""]
    for n in ("demo_basic", "geophysics_401", "lidar_401", "strong_wind", "large_mixed_fleet"):
        print("…", n, file=sys.stderr)
        lines += baselines(n)
    print("… масштабируемость", file=sys.stderr)
    lines += ["## Масштабируемость", ""] + scalability()[2:]
    text = "\n".join(lines)
    out = Path(__file__).resolve().parent.parent.parent / "docs" / "benchmark.md"
    out.parent.mkdir(exist_ok=True)
    out.write_text(text, encoding="utf-8")
    print(text)
