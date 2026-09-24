"""Данные о воздушном пространстве: разбор файлов заказчика и их учёт в плане."""
import json
from datetime import datetime, timedelta
from pathlib import Path

import pytest

from planner.geodata import (
    AltBand,
    Restriction,
    TimeWindow,
    obstacle_from_feature,
    parse_alt_band,
    parse_obstacles_kml,
    parse_task_kml,
    parse_zones_kml,
    restriction_from_feature,
)
from planner.planner import plan
from planner.schemas import PlanRequest

DATA = Path(__file__).resolve().parents[1] / "data"
GEO = DATA / "geodata"
MOSCOW = DATA / "scenarios" / "moscow_real.json"


# --------------------------------------------------------------- разбор высот
@pytest.mark.parametrize(
    "text, lower_m, lower_ref, upper_m, upper_ref",
    [
        ("От земли до 500 м (1700 фут) AMSL", 0.0, "GND", 500.0, "AMSL"),
        ("От земли до 450 м (1500 фут) AGL", 0.0, "GND", 450.0, "AGL"),
        ("От FL280 до FL400", 8534.4, "AMSL", 12192.0, "AMSL"),
        ("От 800 м (2700 фут) AMSL до FL90", 800.0, "AMSL", 2743.2, "AMSL"),
        ("На всех высотах", 0.0, "GND", None, "AMSL"),
        ("", 0.0, "GND", None, "AMSL"),
    ],
)
def test_parse_alt_band(text, lower_m, lower_ref, upper_m, upper_ref):
    b = parse_alt_band(text)
    assert b.lower_m == pytest.approx(lower_m, abs=0.1)
    assert b.lower_ref == lower_ref
    assert (None if b.upper_m == float("inf") else pytest.approx(b.upper_m, abs=0.1)) == upper_m
    assert b.upper_ref == upper_ref
    assert b.raw == text.strip()


def test_parse_alt_band_notes_do_not_break_altitudes():
    """Оговорки заказчика («Некорректная геометрия!», примечания на второй строке) уходят
    в notes и не мешают разобрать высоты."""
    b = parse_alt_band("Некорректная геометрия! От 800 м (2700 фут) AMSL до FL90")
    assert (b.lower_m, b.upper_m) == pytest.approx((800.0, 2743.2), abs=0.1)
    assert "Некорректная геометрия" in b.notes
    b2 = parse_alt_band("От земли до 500 м (1700 фут) AMSL\n\nНе распространяется на воздушные суда")
    assert b2.upper_m == pytest.approx(500.0)
    assert "воздушные суда" in b2.notes


def test_alt_band_agl_span_uses_terrain_conservatively():
    """AMSL → над землёй: нижняя граница считается по самой высокой земле, верхняя — по самой
    низкой, чтобы зона скорее попала в расчёт, чем была пропущена."""
    b = parse_alt_band("От 300 м (1000 фут) AMSL до 800 м AMSL")
    lo, hi = b.agl_span(ground_min_m=100.0, ground_max_m=250.0)
    assert (lo, hi) == pytest.approx((50.0, 700.0))
    assert b.blocks(150.0, 100.0, 250.0) is True
    # та же зона над низкой землёй съёмке на 150 м уже не мешает
    assert b.blocks(150.0, 0.0, 0.0) is False


def test_all_altitudes_zone_blocks_any_flight():
    assert parse_alt_band("На всех высотах").blocks(150.0) is True


# --------------------------------------------------------------- время действия
def test_time_window_filters_by_mission_window():
    t0 = datetime(2026, 9, 29, 6, 0)
    w = TimeWindow(start=datetime(2026, 9, 30, 6, 0), end=datetime(2026, 9, 30, 18, 0))
    assert w.active(t0, t0 + timedelta(hours=12)) is False   # работы за сутки до ограничения
    assert w.active(t0, t0 + timedelta(hours=36)) is True
    # без времени работ ограничение считается действующим
    assert w.active(None, None) is True


def test_time_window_daily_interval():
    w = TimeWindow(daily_from_min=8 * 60, daily_to_min=20 * 60)
    day = datetime(2026, 9, 29, 5, 0)
    assert w.active(day, day + timedelta(hours=1)) is False   # 05:00–06:00 вне 08:00–20:00
    assert w.active(day, day + timedelta(hours=4)) is True    # захватывает 08:00
    night = TimeWindow(daily_from_min=22 * 60, daily_to_min=6 * 60)  # через полночь
    assert night.active(datetime(2026, 9, 29, 23, 0), datetime(2026, 9, 29, 23, 30)) is True


# --------------------------------------------------------------- разбор файлов
@pytest.mark.skipif(not (GEO / "zones_moscow.geojson").exists(), reason="нет выгрузки данных заказчика")
def test_zones_dataset_parsed():
    fc = json.loads((GEO / "zones_moscow.geojson").read_text(encoding="utf-8"))
    zones = fc["features"]
    assert len(zones) == 341
    types = {f["properties"]["zone_type"] for f in zones}
    assert {"prohibited", "permanent", "temporary"} <= types
    # у каждой зоны разобран диапазон высот
    assert all("alt" in f["properties"] for f in zones)
    r = restriction_from_feature(zones[0])
    assert isinstance(r, Restriction) and isinstance(r.band, AltBand)


@pytest.mark.skipif(not (GEO / "obstacles_moscow.geojson").exists(), reason="нет выгрузки данных заказчика")
def test_obstacles_dataset_parsed():
    fc = json.loads((GEO / "obstacles_moscow.geojson").read_text(encoding="utf-8"))
    obs = fc["features"]
    assert len(obs) == 5162
    o = obstacle_from_feature(obs[0])
    assert o.top_m > 0 and o.top_ref in ("AGL", "AMSL")
    # среди препятствий есть и линии (ЛЭП), и полигоны (мачты, здания)
    assert {"LineString", "Polygon"} <= {f["geometry"]["type"] for f in obs}
    assert any((f["properties"].get("obstacle_type") or "") == "COMMUNICATION_TOWER" for f in obs)


@pytest.mark.skipif(not (GEO / "task_moscow.geojson").exists(), reason="нет выгрузки данных заказчика")
def test_task_dataset_parsed():
    fc = json.loads((GEO / "task_moscow.geojson").read_text(encoding="utf-8"))
    assert len(fc["features"]) == 696
    assert all(f["geometry"]["type"] in ("Polygon", "MultiPolygon") for f in fc["features"])


@pytest.mark.skipif(not (Path("../sources/geodata/moscow_zone.kml")).exists(), reason="нет исходных KML")
def test_kml_parsers_on_source_files():
    """Разбор исходных KML заказчика (если они рядом): столько же объектов, что и в выгрузке."""
    src = Path("../sources/geodata")
    assert len(parse_zones_kml(str(src / "moscow_zone.kml"))) == 341
    assert len(parse_obstacles_kml(str(src / "obstacles_moscow.kml"))) == 5162
    assert len(parse_task_kml(str(src / "flight_boundaries.kml"))) == 696


# --------------------------------------------------------------- учёт в плане
@pytest.mark.skipif(not MOSCOW.exists(), reason="нет сценария на данных заказчика")
def test_moscow_scenario_uses_real_airspace():
    data = {k: v for k, v in json.loads(MOSCOW.read_text(encoding="utf-8")).items() if not k.startswith("_")}
    data["use_terrain"] = False
    res = plan(PlanRequest(**data))
    a = res.airspace
    assert a is not None and a.obstacles_total > 0 and a.restrictions_total > 0
    # препятствия ниже высоты съёмки с зазором не мешают, высокие — облетаются
    assert 0 <= a.obstacles_blocking <= a.obstacles_total
    assert res.summary.coverage_pct > 99.0
    assert len(res.obstacles) == a.obstacles_blocking
    assert any("высотные препятствия" in w for w in res.warnings)


def test_high_obstacle_becomes_keep_out_and_low_one_does_not():
    """Мачта выше высоты съёмки с зазором становится запретной зоной (в покрытии появляется
    дырка), а такая же ниже высоты съёмки — не мешает."""
    base = {
        "survey_area": {"type": "Polygon", "coordinates": [[[37.60, 55.60], [37.63, 55.60],
                                                            [37.63, 55.62], [37.60, 55.62], [37.60, 55.60]]]},
        "bases": [{"id": "A", "lon": 37.595, "lat": 55.595}],
        "drones": [{"id": "g1", "model": "geoscan_gemini"}],
        "requirements": {"gsd_cm": 4.0},
        "use_terrain": False,
        "obstacle_buffer_m": 100.0,
    }
    mast = {"type": "Feature",
            "geometry": {"type": "Polygon", "coordinates": [[[37.614, 55.609], [37.616, 55.609],
                                                             [37.616, 55.611], [37.614, 55.611], [37.614, 55.609]]]},
            "properties": {"kind": "obstacle", "id": "M1", "obstacle_type": "COMMUNICATION_TOWER",
                           "top_m": 300.0, "top_ref": "AGL"}}
    high = plan(PlanRequest(**dict(base, obstacles=[mast])))
    low = plan(PlanRequest(**dict(base, obstacles=[{**mast, "properties": {**mast["properties"], "top_m": 20.0}}])))
    assert high.airspace.obstacles_blocking == 1 and low.airspace.obstacles_blocking == 0
    assert high.summary.area_km2 < low.summary.area_km2  # вокруг мачты появилась запретная зона


def test_restriction_above_flight_band_is_reported_but_not_applied():
    """Зона «от 800 м AMSL до FL90» съёмке на 150 м не мешает: в расчёт не входит, но
    показывается с причиной."""
    zone = {"type": "Feature",
            "geometry": {"type": "Polygon", "coordinates": [[[37.60, 55.60], [37.64, 55.60],
                                                             [37.64, 55.63], [37.60, 55.63], [37.60, 55.60]]]},
            "properties": {"kind": "restriction", "id": "UUR999", "zone_type": "temporary",
                           "alt": {"lower_m": 800.0, "lower_ref": "AMSL", "upper_m": 2743.2,
                                   "upper_ref": "AMSL", "raw": "От 800 м (2700 фут) AMSL до FL90"}}}
    req = {
        "survey_area": {"type": "Polygon", "coordinates": [[[37.61, 55.61], [37.63, 55.61],
                                                            [37.63, 55.62], [37.61, 55.62], [37.61, 55.61]]]},
        "bases": [{"id": "A", "lon": 37.605, "lat": 55.605}],
        "drones": [{"id": "g1", "model": "geoscan_gemini"}],
        "use_terrain": False,
        "restrictions": [zone],
    }
    res = plan(PlanRequest(**req))
    assert res.airspace.restrictions_applied == 0
    assert res.restrictions[0]["properties"]["applies"] is False
    assert "выше работ" in res.restrictions[0]["properties"]["skip_reason"]
    # а та же зона от земли — вычитается из области
    ground = dict(zone["properties"]["alt"], lower_m=0.0, lower_ref="GND", upper_m=500.0)
    low = {**req, "restrictions": [{**zone, "properties": {**zone["properties"], "alt": ground}}]}
    from planner.planner import PlanningError
    with pytest.raises(PlanningError):  # зона накрывает всю область — работать негде
        plan(PlanRequest(**low))


def test_temporary_restriction_outside_window_is_skipped():
    zone = {"type": "Feature",
            "geometry": {"type": "Polygon", "coordinates": [[[37.60, 55.60], [37.64, 55.60],
                                                             [37.64, 55.63], [37.60, 55.63], [37.60, 55.60]]]},
            "properties": {"kind": "restriction", "id": "UUR998", "zone_type": "temporary",
                           "alt": {"lower_m": 0.0, "lower_ref": "GND", "upper_m": 500.0, "upper_ref": "AMSL"},
                           "active": {"from": "2026-10-05T00:00:00", "to": "2026-10-06T00:00:00"}}}
    req = {
        "survey_area": {"type": "Polygon", "coordinates": [[[37.61, 55.61], [37.63, 55.61],
                                                            [37.63, 55.62], [37.61, 55.62], [37.61, 55.61]]]},
        "bases": [{"id": "A", "lon": 37.605, "lat": 55.605}],
        "drones": [{"id": "g1", "model": "geoscan_gemini"}],
        "use_terrain": False,
        "restrictions": [zone],
        "mission_start": "2026-09-29T06:00:00",
        "mission_window_h": 12,
    }
    res = plan(PlanRequest(**req))  # ограничение начнётся через неделю — работам не мешает
    assert res.airspace.restrictions_applied == 0
    assert "не действует в окно работ" in res.restrictions[0]["properties"]["skip_reason"]
