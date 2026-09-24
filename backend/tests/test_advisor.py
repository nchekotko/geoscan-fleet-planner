"""Советник (сколько бортов нужно к сроку, сколько времени займут работы) и учёт ТО."""
import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from api.main import app
from planner.advisor import advise
from planner.planner import plan
from planner.schemas import PlanRequest

SCENARIO = Path(__file__).resolve().parents[1] / "data" / "scenarios" / "demo_basic.json"


def load(**over):
    data = json.loads(SCENARIO.read_text(encoding="utf-8"))
    data["use_terrain"] = False
    data.update(over)
    return PlanRequest(**data)


@pytest.fixture(scope="module")
def curve():
    """Кривая «время работ от числа бортов» (считается один раз на модуль)."""
    return advise(load(), workers=1)


def test_advice_curve_is_built_for_every_fleet_size(curve):
    assert [o.drones for o in curve.options] == [1, 2, 3, 4]
    done = [o for o in curve.options if o.makespan_s]
    assert len(done) == 4
    # больше бортов — не дольше по времени работ, но больше суммарный налёт
    assert done[-1].makespan_s < done[0].makespan_s
    assert done[-1].total_flight_s > done[0].total_flight_s


def test_advice_marks_incomplete_coverage(curve):
    """Один самолёт не снимает кольца у запретной зоны: такой вариант срок не выполняет."""
    one = curve.options[0]
    assert one.coverage_pct < 99.5
    assert one.feasible is False and "целиком не покрыть" in one.reason


def test_advice_answers_deadline_question():
    tight = advise(load(), deadline_s=45 * 60, workers=1)
    assert tight.feasible is False and tight.drones_needed is None
    assert "не выполнить" in tight.message
    loose = advise(load(), deadline_s=70 * 60, workers=1)
    assert loose.feasible is True
    assert loose.drones_needed and loose.drones_needed <= 4
    assert loose.drones_needed_ids and len(loose.drones_needed_ids) == loose.drones_needed


def test_advice_respects_max_drones():
    a = advise(load(), max_drones=2, workers=1)
    assert [o.drones for o in a.options] == [1, 2]
    assert a.best_drones <= 2


def test_advice_endpoint():
    c = TestClient(app)
    body = json.loads(SCENARIO.read_text(encoding="utf-8"))
    body["use_terrain"] = False
    r = c.post("/api/advise", json={**body, "deadline_s": 70 * 60})
    assert r.status_code == 200
    a = r.json()
    assert a["feasible"] is True and a["drones_needed"] >= 1 and a["message"]
    assert len(a["options"]) == len(body["drones"])


def test_geodata_endpoints():
    c = TestClient(app)
    items = c.get("/api/geodata").json()
    assert any(i["name"] == "zones_moscow" and i["features"] == 341 for i in items)
    fc = c.get("/api/geodata/zones_moscow").json()
    assert fc["type"] == "FeatureCollection" and len(fc["features"]) == 341
    assert c.get("/api/geodata/nonexistent").status_code == 404


# ------------------------------------------------------------------------ ТО
def test_maintenance_counts_flights_and_hours():
    res = plan(load())
    by_id = {d.drone_id: d.maintenance for d in res.drones}
    m201 = by_id["201-1"]
    assert m201.interval_flights == 80 and m201.flights_after == len(
        next(d for d in res.drones if d.drone_id == "201-1").sorties)
    assert m201.remaining_flights == 80 - m201.flights_after
    m801 = by_id["801-1"]
    assert m801.interval_hours == 160.0 and m801.hours_after > 0
    assert m801.remaining_hours == pytest.approx(160.0 - m801.hours_after, abs=0.01)


def test_maintenance_due_warning():
    """Борт с почти выработанным ресурсом до ТО: предупреждение, что ресурс кончится в задании."""
    data = json.loads(SCENARIO.read_text(encoding="utf-8"))
    data["use_terrain"] = False
    for d in data["drones"]:
        if d["id"] == "201-1":
            d["flights_done"] = 80
    res = plan(PlanRequest(**data))
    m = next(d.maintenance for d in res.drones if d.drone_id == "201-1")
    assert m.due is True and m.remaining_flights < 0
    assert any("ресурс до ТО" in w and "201-1" in w for w in res.warnings)
