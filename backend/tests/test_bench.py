"""Бенчмарк: нижняя граница времени работ и генератор случайных сценариев."""
import json
import math
from pathlib import Path

import pytest

from bench.lower_bound import lower_bound
from bench.random_scenarios import case_seeds, random_request
from planner.planner import Planner, plan
from planner.schemas import PlanRequest

SC = Path(__file__).resolve().parent.parent / "data" / "scenarios"


def _req(name: str, **over) -> PlanRequest:
    data = json.loads((SC / f"{name}.json").read_text(encoding="utf-8"))
    data.update(over)
    data["use_terrain"] = False
    return PlanRequest(**data)


@pytest.mark.parametrize("name", ["strong_wind", "geophysics_401", "lidar_401"])
def test_lower_bound_not_above_plan(name):
    """Граница — релаксация модели: не больше времени работ реального плана и не меньше
    непрерывной версии."""
    req = _req(name)
    lb = lower_bound(Planner(req))
    r = plan(req)
    assert 0 < lb.continuous_s <= lb.integer_noqueue_s + 1.0 <= lb.integer_s + 2.0
    assert lb.integer_s <= r.summary.makespan_s


def test_assignment_is_optimal():
    """Венгерский алгоритм границы совпадает с перебором на малой матрице."""
    import itertools

    import numpy as np

    from bench.lower_bound import _assign_max

    rng = np.random.default_rng(3)
    for _ in range(20):
        w = rng.uniform(0, 10, size=(3, 5))
        brute = max(sum(w[i, c] for i, c in enumerate(cols)) for cols in itertools.permutations(range(5), 3))
        assert _assign_max(w) == pytest.approx(brute)


def test_lower_bound_no_wind_is_rate_times_useful_time():
    """Без ветра площадь за вылет — s·V·(B − O − R), R = 2·d / V_тр."""
    req = _req("strong_wind", wind={"speed_ms": 0, "from_deg": 0})
    p = Planner(req)
    lb = lower_bound(p)
    from shapely.geometry import Point

    cands = {c.instance_id: c for c in p.candidates}
    assert {d.instance_id for d in lb.drones} == set(cands)
    for d in lb.drones:
        c = cands[d.instance_id]
        par = c.params
        v_tr = max(par.speed_ms, c.drone.cruise_speed_ms)
        dist = p.area.distance(Point(p.bases[d.base_id]))
        assert d.reach_s == pytest.approx(2 * dist / v_tr, abs=1.0)
        expected = par.line_spacing_m * par.speed_ms * (d.budget_s - d.overhead_s - d.reach_s)
        assert d.area_per_sortie_m2 == pytest.approx(expected, rel=0.01)
    assert math.isfinite(lb.integer_s)


def test_random_scenarios_reproducible_and_valid():
    for s in case_seeds(1, 5):
        data = random_request(s)
        assert data == random_request(s)
        req = PlanRequest(**data)
        assert 2 <= len(req.drones) <= 6 and 1 <= len(req.bases) <= 2
