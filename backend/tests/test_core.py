import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from shapely.geometry import LineString, Polygon, box, shape

from api.main import app
from planner.coverage import best_direction, sweep_passes
from planner.dubins import shortest_path
from planner.energy import usable_flight_time_s
from planner.fleet import load_fleet
from planner.planner import plan
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
