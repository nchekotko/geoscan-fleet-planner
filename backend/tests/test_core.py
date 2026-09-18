import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from shapely.geometry import LineString, Polygon, box, shape

from api.main import app
from planner.coverage import best_direction, distinct_directions, sweep_passes
from planner.dubins import shortest_path
from planner.energy import usable_flight_time_s
from planner.fleet import load_fleet
from planner.mission import SortieBuilder, order_passes
from planner.planner import Planner, plan
from planner.schemas import PlanRequest
from planner.sensors import SurveyRequirements, survey_params
from planner.wind import Wind, ground_speed

FLEET = load_fleet()
SCENARIO = Path(__file__).resolve().parent.parent / "data" / "scenarios" / "demo_basic.json"


def load_req(**over) -> PlanRequest:
    data = json.loads(SCENARIO.read_text(encoding="utf-8"))
    data.update(over)
    return PlanRequest(**data)


# --- сенсоры ------------------------------------------------------------------
def test_gemini_pf1b_gsd_example_from_plan():
    """PLAN.md 1.2: Gemini + PF1B на 100 м → GSD ≈ 1,96 см, кадр 117,5 × 78,3 м, шаг 35 м при 70 %."""
    p = survey_params(
        FLEET.drones["geoscan_gemini"], FLEET.payloads["pf1b"],
        SurveyRequirements(altitude_m=100, side_overlap=0.7),
    )
    assert p.gsd_cm == pytest.approx(1.958, abs=0.01)
    assert p.swath_m == pytest.approx(117.5, abs=0.2)
    assert p.line_spacing_m == pytest.approx(35.25, abs=0.2)


def test_altitude_clamped_to_ceiling_and_min():
    d = FLEET.drones["geoscan_201"]
    p = survey_params(d, FLEET.payloads["sony_rx1rm2"], SurveyRequirements(gsd_cm=1.0))
    assert p.altitude_agl_m == pytest.approx(100)  # мин. безопасная высота 201
    assert p.notes


def test_lidar_density_limits_speed():
    d = FLEET.drones["geoscan_401"]
    p = survey_params(d, FLEET.payloads["agm_ms3"], SurveyRequirements(lidar_density_pts_m2=500))
    assert p.lidar_density_pts_m2 >= 500 - 1e-6
    assert p.speed_ms < d.cruise_speed_ms


# --- ветер и развороты ---------------------------------------------------------
def test_ground_speed_head_and_tail_wind():
    w = Wind(speed_ms=5, from_deg=270)  # западный ветер дует на восток
    assert ground_speed(20, 0.0, w) == pytest.approx(25)
    assert ground_speed(20, math.pi, w) == pytest.approx(15)
    assert ground_speed(4, math.pi / 2, w) is None  # боковой ветер сильнее воздушной скорости


def test_dubins_reaches_goal():
    random.seed(0)
    for _ in range(500):
        s = (random.uniform(-300, 300), random.uniform(-300, 300), random.uniform(0, 6.28))
        e = (random.uniform(-300, 300), random.uniform(-300, 300), random.uniform(0, 6.28))
        x, y, _ = shortest_path(s, e, 50).sample(2.0)[-1]
        assert math.dist((x, y), e[:2]) < 1e-6


def test_wind_reduces_multirotor_budget():
    d = FLEET.drones["geoscan_gemini"]
    assert usable_flight_time_s(d, Wind(speed_ms=8), 0.2) < usable_flight_time_s(d, Wind(), 0.2)


# --- покрытие -----------------------------------------------------------------
def test_passes_cover_polygon_with_hole():
    outer = box(0, 0, 1000, 600)
    area = outer.difference(box(400, 200, 600, 400))
    passes = sweep_passes(area, 0.0, 50)
    for p in passes:
        assert not LineString([p.a, p.b]).crosses(box(400, 200, 600, 400).buffer(-1))
    covered = LineString([passes[0].a, passes[0].b]).buffer(25, cap_style="flat")
    for p in passes[1:]:
        covered = covered.union(LineString([p.a, p.b]).buffer(25, cap_style="flat"))
    assert covered.intersection(area).area / area.area > 0.99


def test_direction_follows_long_side():
    area = box(0, 0, 3000, 300)
    ang, _, _ = best_direction(area, 50, FLEET.drones["geoscan_gemini"], 10, Wind())
    assert min(ang % math.pi, math.pi - ang % math.pi) < 0.05


# --- нарезка на вылеты -----------------------------------------------------------
def test_split_is_feasible_and_not_worse_than_greedy():
    """Точная нарезка (Split): все вылеты в бюджете, галсы сняты целиком и по порядку,
    время работы борта не больше, чем у жадной нарезки."""
    d = FLEET.drones["geoscan_gemini"]
    area = box(2000, -1500, 4500, 1500)
    route = order_passes(sweep_passes(area, 0.3, 60), (0.0, 0.0))
    b = SortieBuilder(d, 10.0, 100.0, Wind(speed_ms=5, from_deg=200), 25 * 60)
    split = b._build_split(route, (0.0, 0.0), "A")
    greedy = b._build_greedy(route, (0.0, 0.0), "A")
    assert split is not None and len(split) > 1
    assert all(s.duration_s <= b.budget + 1e-6 for s in split)
    flown = [leg.points for s in split for leg in s.legs if leg.kind == "survey"]
    assert flown == [[dp.a, dp.b] for dp in route]
    assert b._finish(split) <= b._finish(greedy) + 1e-6
    assert b._finish(b.build(route, (0.0, 0.0), "A")) == min(b._finish(split), b._finish(greedy))


def test_distinct_directions_modulo_pi():
    """Углы, отличающиеся на π или меньше чем на 5°, считаются одним направлением галсов."""
    a = [0.0, math.pi - 0.01, math.radians(3), math.radians(30), math.pi + math.radians(30.5), 1.0]
    assert distinct_directions(a, 5) == [0.0, math.radians(30), 1.0]
    assert distinct_directions(a, 1) == [0.0]


@pytest.mark.parametrize("w", [1.0, 0.0])
def test_angle_multistart_not_worse_than_single(w):
    """Мультистарт по углу галсов не хуже плана с одним углом (по оценке ведущего борта)
    и не уменьшает покрытие. На lidar_401 при критерии «налёт» оценка угла ошибается — план
    заметно лучше; по времени работ выигрыш уже забирает точная нарезка на вылеты (Split)."""
    data = json.loads((SCENARIO.parent / "lidar_401.json").read_text(encoding="utf-8"))
    req = PlanRequest(**{**data, "use_terrain": False, "time_weight": w})
    multi = Planner(req)
    ev_m, _ = multi.solve()
    single = Planner(req)
    single.directions = lambda: Planner.directions(single)[:1]
    ev_s, _ = single.solve()
    key = (lambda e: e.makespan) if w == 1.0 else (lambda e: e.total)
    assert key(ev_m) <= key(ev_s) + 1e-6
    if w == 0.0:
        assert key(ev_m) < key(ev_s) - 60  # по налёту — строго лучше (минута и более)
    assert multi._coverage(ev_m.results) >= single._coverage(ev_s.results) - 1.0


# --- планировщик ----------------------------------------------------------------
@pytest.mark.parametrize("w", [1.0, 0.0])
def test_plan_respects_battery_and_covers(w):
    req = load_req(time_weight=w)
    res = plan(req)
    if any(d.model != "geoscan_201" for d in res.drones):
        assert res.summary.coverage_pct > 99  # мультироторы добирают полосы у NFZ
    else:
        assert res.summary.coverage_pct > 95
        assert any("не хватает места для разворота" in w_ for w_ in res.warnings)
    for d in res.drones:
        budget = usable_flight_time_s(FLEET.drones[d.model], req.wind, req.reserve)
        for s in d.sorties:
            assert s.duration_s <= budget + 1


def test_time_criterion_beats_single_best_drone():
    fast = plan(load_req(time_weight=1.0)).summary
    lean = plan(load_req(time_weight=0.0)).summary
    assert fast.makespan_s < lean.makespan_s
    assert lean.total_flight_s <= fast.total_flight_s


def test_survey_legs_avoid_nfz():
    req = load_req()
    res = plan(req)
    nfz = [shape(z) for z in req.no_fly_zones]
    for d in res.drones:
        for s in d.sorties:
            for leg in s.legs:
                if leg.kind == "survey":
                    line = LineString([c[:2] for c in leg.coordinates])
                    assert not any(line.crosses(z) or line.within(z) for z in nfz)


def test_unsupported_survey_type_excludes_drones():
    res = plan(load_req(survey_type="thermal", requirements={"gsd_cm": 10, "front_overlap": 0.8, "side_overlap": 0.7}))
    assert [d.model for d in res.drones] == ["geoscan_801"]
    assert any("нет нагрузки" in e.reason for e in res.excluded)


# --- API и экспорт ---------------------------------------------------------------
def test_api_plan_and_exports():
    c = TestClient(app)
    body = json.loads(SCENARIO.read_text(encoding="utf-8"))
    r = c.post("/api/plan", json=body)
    assert r.status_code == 200, r.text
    data = r.json()
    pid, drone = data["plan_id"], data["drones"][0]["drone_id"]
    gj = c.get(f"/api/plan/{pid}/export/{drone}.geojson").json()
    assert gj["type"] == "FeatureCollection"
    for f in gj["features"]:
        for lon, lat, *_ in (f["geometry"]["coordinates"] if f["geometry"]["type"] == "LineString"
                             else [f["geometry"]["coordinates"]] if f["geometry"]["type"] == "Point" else []):
            assert -180 <= lon <= 180 and -90 <= lat <= 90
    kml = c.get(f"/api/plan/{pid}/export/{drone}.kml").text
    root = ET.fromstring(kml.encode("utf-8"))
    assert root.tag.endswith("kml")
    assert c.get(f"/api/plan/{pid}/export.zip").status_code == 200


def test_api_rejects_empty_area():
    c = TestClient(app)
    body = json.loads(SCENARIO.read_text(encoding="utf-8"))
    body["no_fly_zones"] = [body["survey_area"]]
    assert c.post("/api/plan", json=body).status_code == 422


def _demo_body(**over) -> dict:
    body = json.loads(SCENARIO.read_text(encoding="utf-8"))
    body["use_terrain"] = False
    body.update(over)
    return body


def _far_body(dlat: float = 0.072) -> dict:
    """demo_basic с областью, сдвинутой на север (0,072° ≈ 8 км): мультироторы не долетают."""
    body = _demo_body(no_fly_zones=[])
    ring = body["survey_area"]["coordinates"][0]
    body["survey_area"]["coordinates"][0] = [[lon, lat + dlat] for lon, lat in ring]
    return body


def _bad_inputs() -> list[tuple[str, dict]]:
    def req(overlap=None, **over):
        b = _demo_body(**over)
        if overlap is not None:
            b["requirements"] = {**b["requirements"], **overlap}
        return b

    area = _demo_body()["survey_area"]
    bow = {"type": "Polygon", "coordinates": [[[37.60, 55.60], [37.64, 55.62], [37.64, 55.60], [37.60, 55.62], [37.60, 55.60]]]}
    return [
        ("overlap=1", req({"side_overlap": 1.0})),
        ("front_overlap=1", req({"front_overlap": 1.0})),
        ("gsd=0", req({"gsd_cm": 0})),
        ("NaN в ветре", req(wind={"speed_ms": float("nan"), "from_deg": 0})),
        ("ветер 100 м/с", req(wind={"speed_ms": 100, "from_deg": 0})),
        ("lat=95", req(bases=[{"id": "A", "lon": 37.6, "lat": 95}])),
        ("самопересечение", req(survey_area=bow, no_fly_zones=[])),
        ("пустой полигон", req(survey_area={"type": "Polygon", "coordinates": []})),
        ("вырожденный полигон", req(survey_area={"type": "Polygon", "coordinates": [[[37.6, 55.6], [37.61, 55.6], [37.6, 55.6], [37.6, 55.6]]]})),
        ("не полигон", req(survey_area={"type": "Point", "coordinates": [37.6, 55.6]})),
        ("NFZ-линия", req(no_fly_zones=[{"type": "LineString", "coordinates": [[37.6, 55.6], [37.61, 55.61]]}])),
        ("повтор id бортов", req(drones=[{"id": "x", "model": "geoscan_201"}, {"id": "x", "model": "geoscan_gemini"}])),
        ("повтор id баз", req(bases=[{"id": "A", "lon": 37.605, "lat": 55.595}, {"id": "A", "lon": 37.65, "lat": 55.61}])),
        ("отрицательный буфер", req(nfz_buffer_m=-50)),
        ("неизвестная модель", req(drones=[{"id": "x", "model": "nope"}])),
        ("неизвестная база", req(drones=[{"id": "x", "model": "geoscan_201", "base_id": "Z"}])),
        ("нет бортов", req(drones=[])),
        ("нет баз", req(bases=[])),
        ("reserve=0.9", req(reserve=0.9)),
        ("область на другом краю света", req(survey_area={**area, "coordinates": [[[lon - 150, lat] for lon, lat in area["coordinates"][0]]]}, no_fly_zones=[])),
        ("далёкая область", _far_body(0.072)),
        ("очень далёкая область", _far_body(1.0)),
    ]


@pytest.mark.parametrize("name,body", _bad_inputs(), ids=[n for n, _ in _bad_inputs()])
def test_api_never_500_on_bad_input(name, body):
    c = TestClient(app, raise_server_exceptions=False)
    r = c.post("/api/plan", content=json.dumps(body), headers={"Content-Type": "application/json"})
    assert r.status_code in (200, 422), f"{name}: {r.status_code} {r.text[:300]}"
    if r.status_code == 422:
        assert isinstance(r.json()["detail"], str) and r.json()["detail"]


def test_far_area_fixed_wing_only():
    """Область в ~8 км от баз: мультироторы исключены с указанием расстояния, снимает самолёт."""
    c = TestClient(app, raise_server_exceptions=False)
    r = c.post("/api/plan", json=_far_body(0.072))
    assert r.status_code == 200, r.text
    data = r.json()
    assert {d["model"] for d in data["drones"]} == {"geoscan_201"}
    assert data["summary"]["coverage_pct"] > 99
    far = [e for e in data["excluded"] if "не долетает до области" in e["reason"]]
    assert {e["drone_id"] for e in far} == {"gemini-1", "gemini-2", "801-1"}
    assert all("км" in e["reason"] for e in far)
    # никто не долетает — 422 с причинами по каждому борту
    r = c.post("/api/plan", json=_far_body(1.0))
    assert r.status_code == 422
    assert "не долетает до области" in r.json()["detail"] and "201-1" in r.json()["detail"]


def test_self_intersecting_area_is_repaired_or_rejected():
    body = _demo_body(no_fly_zones=[], survey_area={
        "type": "Polygon",
        "coordinates": [[[37.60, 55.60], [37.64, 55.62], [37.64, 55.60], [37.60, 55.62], [37.60, 55.60]]],
    })
    req = PlanRequest(**body)
    assert shape(req.survey_area).is_valid and shape(req.survey_area).area > 0


def test_unexpected_error_is_json_500(monkeypatch):
    import api.main as api_main

    def boom(req):
        raise RuntimeError("сбой")

    monkeypatch.setattr(api_main, "plan", boom)
    r = TestClient(app, raise_server_exceptions=False).post("/api/plan", json=_demo_body())
    assert r.status_code == 500
    assert "внутренняя ошибка" in r.json()["detail"]


def test_export_cyrillic_drone_id():
    c = TestClient(app)
    body = _demo_body(drones=[{"id": "Борт-1", "model": "geoscan_201", "base_id": "A"}])
    r = c.post("/api/plan", json=body)
    assert r.status_code == 200, r.text
    pid = r.json()["plan_id"]
    for fmt in ("geojson", "kml"):
        e = c.get(f"/api/plan/{pid}/export/Борт-1.{fmt}")
        assert e.status_code == 200
        cd = e.headers["content-disposition"]
        cd.encode("latin-1")  # заголовок передаётся без ошибок кодировки
        assert f"filename*=UTF-8''%D0%91%D0%BE%D1%80%D1%82-1.{fmt}" in cd
        assert f'filename="____-1.{fmt}"' in cd


def test_slow_survey_in_strong_wind_is_excluded_not_crashing():
    res_req = load_req(survey_type="thermal", drones=[{"id": "a", "model": "geoscan_801"}, {"id": "b", "model": "geoscan_gemini"}])
    with pytest.raises(ValueError, match="ни один борт"):
        plan(res_req)  # тепловизор при GSD 3 см требует скорости < ветра


@pytest.mark.parametrize("w", [1.0, 0.0])
def test_no_leg_enters_nfz(w):
    """Все участки (транзит, развороты, возврат) обходят запретные зоны."""
    req = load_req(time_weight=w)
    res = plan(req)
    nfz = [shape(z) for z in req.no_fly_zones]
    for d in res.drones:
        for s in d.sorties:
            for leg in s.legs:
                if len(leg.coordinates) >= 2:
                    line = LineString([c[:2] for c in leg.coordinates])
                    assert not any(line.intersects(z) for z in nfz), (d.drone_id, s.index, leg.kind)
    assert not any("запретную зону" in w_ for w_ in res.warnings)


def test_router_goes_around_obstacle():
    from planner.avoid import Router, length
    r = Router([box(-50, -50, 50, 50)], margin=10)
    pts = r.route((-200, 0), (200, 0))
    assert len(pts) > 2
    assert not LineString(pts).intersects(box(-50, -50, 50, 50))
    assert 400 < length(pts) < 520


def test_cells_split_around_hole():
    from planner.coverage import boustrophedon_cells
    area = box(0, 0, 1000, 600).difference(box(400, 200, 600, 400))
    cells = boustrophedon_cells(sweep_passes(area, 0.0, 50))
    # под дырой, слева, справа, над дырой
    assert len(cells) == 4
    assert sum(len(c) for c in cells) == len(sweep_passes(area, 0.0, 50))


def test_route_around_hole_has_few_detours():
    """После декомпозиции переходов через дыру почти нет: число транзитов внутри вылета мало."""
    res = plan(load_req(time_weight=0.0))
    d = res.drones[0]
    transits = sum(1 for s in d.sorties for l in s.legs if l.kind == "transit") - len(d.sorties)
    assert transits <= 3  # 4 ячейки → 3 перехода между ними


def test_fixed_wing_turns_never_replaced_by_straight_hops():
    """Самолёт не «перепрыгивает» между галсами по прямой: все развороты — пути Дубинса."""
    res = plan(load_req(time_weight=0.0))
    d = res.drones[0]
    assert d.model == "geoscan_201"
    hops = [l for s in d.sorties for l in s.legs if l.kind == "transit" and l.distance_m < 200]
    assert not hops


# --- дополнительные сценарии ------------------------------------------------------
def scenario(name: str, **over) -> PlanRequest:
    data = json.loads((SCENARIO.parent / f"{name}.json").read_text(encoding="utf-8"))
    data.update(over)
    return PlanRequest(**data)


def _assert_budget(req, res):
    for d in res.drones:
        budget = usable_flight_time_s(FLEET.drones[d.model], req.wind, req.reserve)
        for s in d.sorties:
            assert s.duration_s <= budget + 1, (d.drone_id, s.index)


def test_geophysics_has_tie_lines():
    req = scenario("geophysics_401")
    res = plan(req)
    kinds = {l.kind for d in res.drones for s in d.sorties for l in s.legs}
    assert "tie" in kinds and "survey" in kinds
    assert all(d.params.altitude_agl_m == 30 for d in res.drones)
    _assert_budget(req, res)


def test_large_area_uses_reach_aware_partition():
    req = scenario("large_mixed_fleet")
    res = plan(req)
    assert any("с учётом дальности" in w for w in res.warnings)
    assert res.summary.coverage_pct > 99
    _assert_budget(req, res)
    # мультироторы работают у своих баз, основной объём — у самолётов
    by_model = {}
    for d in res.drones:
        by_model[d.model] = by_model.get(d.model, 0) + d.area_km2
    assert by_model["geoscan_201"] > 5 * by_model["geoscan_gemini"]


def test_strong_wind_raises_speed_and_excludes_weak():
    res = plan(scenario("strong_wind"))
    ids = {d.drone_id for d in res.drones}
    assert ids == {"201-1", "401-1"}
    d401 = next(d for d in res.drones if d.drone_id == "401-1")
    assert d401.params.speed_ms > 11


def test_divert_to_nearest_site_computed():
    res = plan(load_req())
    for d in res.drones:
        for s in d.sorties:
            assert s.max_divert_s > 0 and s.divert_site


def test_pareto_front_is_non_dominated():
    from planner.pareto import pareto_front
    front = pareto_front(load_req(), weights=(1.0, 0.5, 0.0), workers=1)
    assert len(front) >= 2
    for a, b in zip(front, front[1:]):
        assert a.summary.makespan_s <= b.summary.makespan_s
        assert a.summary.total_flight_s > b.summary.total_flight_s


def test_min_flight_time_keeps_full_coverage():
    """При весе «налёт» = 1 кольца у NFZ, где самолёт не разворачивается, добирает
    мультиротор, а во фронт Парето не попадают планы с неполным покрытием."""
    from planner.pareto import pareto_front
    res = plan(load_req(time_weight=0.0, use_terrain=False))
    assert res.summary.coverage_pct >= 99.9
    front = pareto_front(load_req(use_terrain=False), weights=(1.0, 0.5, 0.0), workers=1)
    assert front and all(p.summary.coverage_pct >= 99.5 for p in front)


def test_time_plan_not_dominated_by_pareto_front():
    """План при w = 1 не хуже ни одной точки фронта сразу по обоим критериям."""
    from planner.pareto import pareto_front
    req = load_req(use_terrain=False)
    t = plan(req.model_copy(update={"time_weight": 1.0})).summary
    # с очерёдностью стартов на ВПП (2 мин у мультиротора) план по времени — около 59 мин
    assert t.makespan_s <= 60.5 * 60
    for p in pareto_front(req, weights=(1.0, 0.5, 0.0), workers=1):
        s = p.summary
        assert not (s.makespan_s < t.makespan_s - 1.0 and s.total_flight_s < t.total_flight_s - 1.0)
        assert s.makespan_s >= t.makespan_s - 1.0  # быстрее плана «время работ» во фронте нет


def test_api_pareto():
    c = TestClient(app)
    body = json.loads((SCENARIO.parent / "strong_wind.json").read_text(encoding="utf-8"))
    r = c.post("/api/pareto", json=body)
    assert r.status_code == 200, r.text
    pts = r.json()
    assert pts and all("plan_id" in p for p in pts)


# --- рельеф --------------------------------------------------------------------
from planner.terrain import DEM_DIR, tile_name  # noqa: E402

HAS_MOSCOW_DEM = (DEM_DIR / f"{tile_name(55, 37)}.tif").exists()


@pytest.mark.skipif(not HAS_MOSCOW_DEM, reason="нет тайла Copernicus DEM в кэше")
def test_terrain_following_keeps_agl():
    req = scenario("geophysics_401")
    res = plan(req)
    assert res.terrain and 50 < res.terrain.ground_min_m < res.terrain.ground_max_m < 400
    for d in res.drones:
        for s in d.sorties:
            for leg in s.legs:
                if leg.kind in ("survey", "tie"):
                    assert leg.alt_amsl and len(leg.alt_amsl) == len(leg.coordinates)
                    assert len(leg.coordinates) >= 2


@pytest.mark.skipif(not HAS_MOSCOW_DEM, reason="нет тайла Copernicus DEM в кэше")
def test_fixed_wing_flies_constant_level_not_below_start():
    res = plan(load_req(time_weight=0.0))
    d = res.drones[0]
    assert d.model == "geoscan_201"
    for s in d.sorties:
        takeoff_ground = s.legs[0].alt_amsl[0]
        levels = {a for l in s.legs if l.kind not in ("takeoff", "landing") for a in l.alt_amsl}
        assert len(levels) == 1
        assert levels.pop() >= takeoff_ground + d.params.altitude_agl_m


def test_no_terrain_gives_warning(monkeypatch):
    import planner.terrain as t
    monkeypatch.setattr(t, "DEM_DIR", Path("nonexistent_dem_dir"))
    res = plan(load_req())
    assert res.terrain is None
    assert any("рельеф недоступен" in w for w in res.warnings)


# --- ключевые точки экспорта ------------------------------------------------------
from planner.export import drone_geojson, drone_kml, waypoints  # noqa: E402

KML_NS = "{http://www.opengis.net/kml/2.2}"


def test_landing_ends_on_ground():
    """Взлёт: земля → высота, посадка: высота → земля (AGL и абсолютные высоты)."""
    res = plan(load_req())
    for d in res.drones:
        for s in d.sorties:
            to, ld = s.legs[0], s.legs[-1]
            assert (to.kind, ld.kind) == ("takeoff", "landing")
            assert to.coordinates[0][2] == 0 and to.coordinates[-1][2] > 0
            assert ld.coordinates[0][2] > 0 and ld.coordinates[-1][2] == 0
            if ld.alt_amsl:
                assert ld.alt_amsl[-1] == to.alt_amsl[0] < ld.alt_amsl[0]


def test_leg_speeds():
    res = plan(load_req())
    for d in res.drones:
        for s in d.sorties:
            for leg in s.legs:
                if leg.kind in ("survey", "tie", "turn"):
                    assert leg.speed_ms == pytest.approx(d.params.speed_ms, abs=0.01)
                elif leg.kind in ("transit", "return"):
                    assert leg.speed_ms >= d.params.speed_ms - 0.01
                else:
                    assert 0 < leg.speed_ms < 10  # вертикальная скорость


@pytest.mark.parametrize("name", ["large_mixed_fleet", "demo_basic"])
def test_waypoints_avoid_nfz_and_start_end_on_ground(name):
    req = scenario(name)
    res = plan(req)
    nfz = [shape(z) for z in req.no_fly_zones]
    assert nfz and res.no_fly_zones == req.no_fly_zones and res.allowed_area == req.allowed_area
    for d in res.drones:
        wps = waypoints(d, res)
        assert [w["seq"] for w in wps] == list(range(1, len(wps) + 1))
        base = next(b for b in res.bases if b.id == d.sorties[0].base_id)
        for s in d.sorties:
            sw = [w for w in wps if w["sortie"] == s.index + 1]
            assert sw[0]["action"] == "takeoff" and sw[-1]["action"] == "land"
            for w in (sw[0], sw[-1]):
                assert w["altitude_agl_m"] == 0
                assert w["coord"][:2] == pytest.approx([base.lon, base.lat], abs=1e-7)
            # длительности участков округлены до 0,1 с
            assert sw[-1]["eta_s"] == pytest.approx(s.duration_s, abs=0.05 * len(s.legs) + 0.2)
            assert all(a["eta_s"] <= b["eta_s"] for a, b in zip(sw, sw[1:]))
            assert {"survey_start", "survey_end"} <= {w["action"] for w in sw}
            for a, b in zip(sw, sw[1:]):
                assert a["coord"] != b["coord"] or a["altitude_amsl_m"] != b["altitude_amsl_m"]
                seg = LineString([a["coord"][:2], b["coord"][:2]])
                assert not any(seg.intersects(z) for z in nfz), (d.drone_id, a["seq"])
            for w in sw:
                if w["phase"] in ("transit", "return"):
                    assert w["speed_ms"] >= d.params.speed_ms - 0.01


def test_exports_carry_keypoints_speeds_and_constraints():
    req = scenario("large_mixed_fleet")
    res = plan(req)
    d = res.drones[0]
    gj = drone_geojson(d, res)
    kinds = [f["properties"]["kind"] for f in gj["features"]]
    assert kinds.count("no_fly_zone") == len(req.no_fly_zones) and "allowed_area" in kinds
    legs = [f["properties"] for f in gj["features"] if f["properties"]["kind"] == "leg"]
    assert {"takeoff", "landing"} <= {p["phase"] for p in legs}
    assert all(p["speed_ms"] > 0 for p in legs)
    wp = [f["properties"] for f in gj["features"] if f["properties"]["kind"] == "waypoint"]
    assert all({"seq", "phase", "action", "speed_ms", "eta_s", "altitude_agl_m"} <= p.keys() for p in wp)
    root = ET.fromstring(drone_kml(d, res).encode("utf-8"))
    folders = {}
    for f in root.iter(f"{KML_NS}Folder"):
        folders.setdefault(f.findtext(f"{KML_NS}name"), []).append(f)
    assert "Ограничения" in folders and len(folders["Ключевые точки"]) == len(d.sorties)
    pts = [p for f in folders["Ключевые точки"] for p in f.findall(f"{KML_NS}Placemark")]
    assert len(pts) == len(wp)
    data = {e.get("name") for e in pts[0].iter(f"{KML_NS}Data")}
    assert {"phase", "speed_ms", "altitude_agl_m", "altitude_amsl_m", "eta_s"} <= data
    speeds = {e.findtext(f"{KML_NS}value") for f in root.iter(f"{KML_NS}Placemark")
              if f.find(f"{KML_NS}LineString") is not None for e in f.iter(f"{KML_NS}Data")
              if e.get("name") == "speed_ms"}
    assert speeds and all(float(v) > 0 for v in speeds)


@pytest.mark.parametrize("name", ["demo_basic", "lidar_401", "geophysics_401"])
def test_every_leg_has_speed(name):
    """У каждого участка своя скорость — и при жадной нарезке, и при точной (Split)."""
    res = plan(scenario(name, use_terrain=False))
    for d in res.drones:
        for s in d.sorties:
            for leg in s.legs:
                assert leg.speed_ms > 0, (d.drone_id, s.index, leg.kind)


def test_pareto_survives_broken_pool(monkeypatch):
    """Если пул процессов упал, фронт Парето досчитывается последовательно."""
    import planner.pareto as pr
    from concurrent.futures.process import BrokenProcessPool

    class Broken:
        def __init__(self, *a, **k):
            pass

        def __enter__(self):
            raise BrokenProcessPool("тест")

        def __exit__(self, *a):
            return False

    monkeypatch.setattr(pr, "ProcessPoolExecutor", Broken)
    front = pr.pareto_front(scenario("strong_wind", use_terrain=False), weights=(1.0, 0.0), eps_points=0)
    assert front


def test_multirotor_reach_in_wind_keeps_full_coverage():
    """Радиус действия по треугольнику скоростей: при ветре 7 м/с мультироторы долетают
    до полос у NFZ, и demo снимается полностью (раньше радиус был занижен, покрытие 95,7 %)."""
    res = plan(load_req(wind={"speed_ms": 7, "from_deg": 270}, use_terrain=False))
    assert res.summary.coverage_pct > 99.9


def test_energy_range_matches_round_trip():
    """Радиус по заряду: полёт туда-обратно на этот радиус при худшем направлении ветра
    занимает весь бюджет вылета без взлёта и посадки."""
    from planner.planner import energy_range_m, round_trip_s_per_m
    d = FLEET.drones["geoscan_gemini"]
    p = survey_params(d, FLEET.payloads["pf1b"], SurveyRequirements(altitude_m=100))
    w = Wind(speed_ms=7, from_deg=0)
    r = energy_range_m(d, p, 1800, w)
    t_ops = 100 / d.climb_rate_ms + 100 / 3.0
    assert r * round_trip_s_per_m(max(p.speed_ms, d.cruise_speed_ms), w) == pytest.approx(1800 - t_ops, rel=1e-6)
    # без ветра — просто V·t/2
    assert energy_range_m(d, p, 1800, Wind()) == pytest.approx(10 * (1800 - t_ops) / 2, rel=1e-3)



def test_launches_are_staggered_per_base():
    """С одной ВПП борта стартуют по очереди: самолёт первым, затем мультироторы с интервалом."""
    from planner.mission import LAUNCH_INTERVAL_S
    res = plan(load_req(use_terrain=False))
    by_base: dict[str, list[float]] = {}
    for d in res.drones:
        by_base.setdefault(d.sorties[0].base_id, []).append(d.sorties[0].start_s)
    for starts in by_base.values():
        starts.sort()
        assert starts[0] == 0.0
        for a, b in zip(starts, starts[1:]):
            assert b - a >= min(LAUNCH_INTERVAL_S.values()) - 1e-6


def test_transit_levels_and_separation_reported():
    """Эшелоны перелёта различаются на 20 м; в сводке — наименьшее сближение бортов."""
    res = plan(load_req(use_terrain=False))
    levels = sorted(d.transit_alt_agl_m - d.params.altitude_agl_m for d in res.drones)
    assert levels == pytest.approx([20.0 * k for k in range(len(levels))], abs=0.1)
    for d in res.drones:
        for s in d.sorties:
            for leg in s.legs:
                if leg.kind in ("transit", "return"):
                    assert all(c[2] == pytest.approx(d.transit_alt_agl_m, abs=0.1) for c in leg.coordinates)
    assert res.summary.min_separation_m is None or res.summary.min_separation_m > 50


def test_separation_detects_head_on():
    """Два трека навстречу друг другу на одной высоте — конфликт; разнесённые по высоте — нет."""
    from planner.mission import Leg, Sortie
    from planner.separation import closest_approach, track

    def sortie(a, b):
        s = Sortie(index=0, base=(0.0, 0.0), base_id="A")
        s.legs.append(Leg("survey", [a, b], 100.0, 100.0, 1000.0, 10.0))
        return s

    t1 = track("x", [sortie((0.0, 5000.0), (1000.0, 5000.0))], lambda l: 100.0)
    t2 = track("y", [sortie((1000.0, 5000.0), (0.0, 5000.0))], lambda l: 100.0)
    ap = closest_approach([t1, t2], [(0.0, 0.0)], 100.0)
    assert ap is not None and ap.min_h_m < 20 and ap.conflicts >= 1
    t3 = track("z", [sortie((1000.0, 5000.0), (0.0, 5000.0))], lambda l: 130.0)
    assert closest_approach([t1, t3], [(0.0, 0.0)], 100.0) is None


@pytest.mark.parametrize("gsd,passport_km2", [(2, 0.95), (3, 1.4), (5, 2.1)])
def test_geoscan_401_matches_passport_area_per_flight(gsd, passport_km2):
    """Калибровка по паспорту Геоскан 401 (руководство, с. 125): площадь фотосъёмки за полёт
    при 2/3/5 см/пикс — 0,95/1,4/2,1 км². Модель — в пределах 85–105 % паспорта."""
    d = {
        "survey_area": {"type": "Polygon", "coordinates": [[[37.60, 55.60], [37.70, 55.60], [37.70, 55.66],
                                                             [37.60, 55.66], [37.60, 55.60]]]},
        "bases": [{"id": "A", "lon": 37.65, "lat": 55.63}],
        "drones": [{"id": "401", "model": "geoscan_401", "payload": "sony_rx1rm3"}],
        "survey_type": "rgb",
        "requirements": {"gsd_cm": gsd, "front_overlap": 0.8, "side_overlap": 0.7},
        "use_terrain": False,
    }
    dr = plan(PlanRequest(**d)).drones[0]
    per = [s.survey_length_m * dr.params.line_spacing_m / 1e6 for s in dr.sorties][:-1]
    assert 0.85 <= sum(per) / len(per) / passport_km2 <= 1.05


def test_wind_profile_from_reference_height():
    """Ветер, заданный у земли (10 м), на рабочей высоте сильнее: v·(h/10)^0,14."""
    w = Wind(speed_ms=5, from_deg=270, ref_height_m=10)
    assert w.at(150).speed_ms == pytest.approx(5 * 15 ** 0.14)
    assert Wind(speed_ms=5).at(150).speed_ms == 5  # без высоты задания — ветер уже на рабочей высоте


def test_ground_wind_excludes_drone_over_limit():
    """7 м/с у земли на высоте Gemini (~200 м) — 10,7 м/с, больше его предела 10 м/с:
    Gemini исключается с указанием высоты, остальные работают."""
    res = plan(scenario("strong_wind", use_terrain=False,
                        wind={"speed_ms": 7, "from_deg": 0, "ref_height_m": 10}))
    gemini = [e.reason for e in res.excluded if e.drone_id == "gemini-1"]
    assert gemini and "на высоте" in gemini[0]
    assert res.drones


def test_lidar_uses_lidar_overlap():
    d = FLEET.drones["geoscan_401"]
    p20 = survey_params(d, FLEET.payloads["agm_ms3"], SurveyRequirements())
    p50 = survey_params(d, FLEET.payloads["agm_ms3"], SurveyRequirements(lidar_side_overlap=0.5))
    assert p20.line_spacing_m == pytest.approx(p20.swath_m * 0.8)
    assert p50.line_spacing_m < p20.line_spacing_m


def test_fixed_wing_launch_and_landing_into_wind():
    """Геоскан 201 при ветре: катапульта и заход на посадку против ветра, парашют раскрывается
    с наветренной стороны ВПП на величину сноса (руководство 201: запуск и посадка против ветра)."""
    from planner.geo import LocalFrame
    from planner.mission import LAUNCH_RUN_M, PARACHUTE_OPEN_AGL_M, PARACHUTE_SINK_MS
    req = load_req(time_weight=0.0, use_terrain=False)  # западный ветер 5 м/с — дует на восток
    res = plan(req)
    d = next(x for x in res.drones if x.model == "geoscan_201")
    base = next(b for b in req.bases if b.id == d.sorties[0].base_id)
    frame = LocalFrame(lon0=base.lon, lat0=base.lat)
    for s in d.sorties:
        take, land = s.legs[0], s.legs[-1]
        (x0, y0), (x1, y1) = (frame.lonlat_to_xy(*c[:2]) for c in take.coordinates)
        assert x1 - x0 == pytest.approx(-LAUNCH_RUN_M, abs=2) and abs(y1 - y0) < 2  # старт на запад
        assert take.coordinates[0][2] == 0.0
        pts = [frame.lonlat_to_xy(*c[:2]) for c in land.coordinates]
        drift = 5 * PARACHUTE_OPEN_AGL_M / PARACHUTE_SINK_MS
        assert pts[1][0] == pytest.approx(-drift, abs=2)  # раскрытие западнее ВПП на снос
        assert pts[0][0] > pts[1][0]  # заход с подветренной (восточной) стороны
        assert [c[2] for c in land.coordinates][1:] == [PARACHUTE_OPEN_AGL_M, 0.0]


def test_multirotor_takeoff_stays_vertical():
    res = plan(load_req(use_terrain=False))
    for d in res.drones:
        if d.model != "geoscan_201":
            for s in d.sorties:
                assert s.legs[0].coordinates[0][:2] == s.legs[0].coordinates[-1][:2]
