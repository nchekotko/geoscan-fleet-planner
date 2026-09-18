"""Генератор тестовых сценариев (data/scenarios/*.json). Запуск: python data/make_scenarios.py"""
import json
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent / "scenarios"


def rect(lon, lat, w_km, h_km, rot_deg=0.0):
    """Прямоугольник w×h км с центром (lon, lat), повёрнутый на rot_deg."""
    k_lon = 1 / (111.32 * math.cos(math.radians(lat)))
    k_lat = 1 / 110.57
    a = math.radians(rot_deg)
    pts = []
    for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)]:
        x, y = sx * w_km / 2, sy * h_km / 2
        xr, yr = x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)
        pts.append([round(lon + xr * k_lon, 6), round(lat + yr * k_lat, 6)])
    return {"type": "Polygon", "coordinates": [pts]}


def poly(*pts):
    ring = [list(p) for p in pts] + [list(pts[0])]
    return {"type": "Polygon", "coordinates": [ring]}


SCENARIOS = {
    # Геофизика: магнитная съёмка двумя Геоскан 401, секущие маршруты
    "geophysics_401": {
        "survey_area": rect(37.95, 55.45, 1.6, 1.0, 15),
        "no_fly_zones": [rect(37.955, 55.452, 0.25, 0.2)],
        "bases": [{"id": "A", "name": "Лагерь", "lon": 37.935, "lat": 55.443}],
        "reserve_sites": [{"id": "R1", "lon": 37.965, "lat": 55.457}],
        "drones": [
            {"id": "401-mag-1", "model": "geoscan_401", "payload": "geoshark"},
            {"id": "401-mag-2", "model": "geoscan_401", "payload": "geoshark"},
        ],
        "survey_type": "geophysics",
        "requirements": {"line_spacing_m": 50, "altitude_m": 30},
        "wind": {"speed_ms": 4, "from_deg": 200},
        "time_weight": 1.0,
    },
    # LiDAR: 401 с AGM-MS3, плотность 100 т/м²
    "lidar_401": {
        "survey_area": poly((37.30, 55.80), (37.33, 55.795), (37.345, 55.81), (37.325, 55.822), (37.30, 55.815)),
        "no_fly_zones": [],
        "bases": [{"id": "A", "lon": 37.315, "lat": 55.79}],
        "reserve_sites": [],
        "drones": [
            {"id": "401-L1", "model": "geoscan_401", "payload": "agm_ms3"},
            {"id": "401-L2", "model": "geoscan_401", "payload": "alphaair_450"},
        ],
        "survey_type": "lidar",
        "requirements": {"lidar_density_pts_m2": 100, "lidar_side_overlap": 0.2},
        "wind": {"speed_ms": 3, "from_deg": 90},
        "time_weight": 1.0,
    },
    # Большая площадь, смешанный парк, две базы, разрешённая зона и два запрета
    "large_mixed_fleet": {
        "survey_area": poly((38.00, 55.30), (38.16, 55.29), (38.20, 55.36), (38.12, 55.40), (38.02, 55.38)),
        "allowed_area": poly((37.97, 55.27), (38.23, 55.27), (38.23, 55.39), (38.13, 55.42), (37.97, 55.41)),
        "no_fly_zones": [rect(38.08, 55.34, 1.5, 1.2), rect(38.15, 55.37, 1.0, 0.8, 30)],
        "bases": [
            {"id": "A", "name": "Юг", "lon": 38.05, "lat": 55.285},
            {"id": "B", "name": "Северо-восток", "lon": 38.19, "lat": 55.385},
        ],
        "reserve_sites": [{"id": "R1", "lon": 38.03, "lat": 55.36}, {"id": "R2", "lon": 38.17, "lat": 55.32}],
        "drones": [
            {"id": "201-1", "model": "geoscan_201", "base_id": "A"},
            {"id": "201-2", "model": "geoscan_201", "base_id": "B"},
            {"id": "gemini-1", "model": "geoscan_gemini", "base_id": "A"},
            {"id": "gemini-2", "model": "geoscan_gemini", "base_id": "B"},
            {"id": "401-1", "model": "geoscan_401", "base_id": "B"},
        ],
        "survey_type": "rgb",
        "requirements": {"gsd_cm": 5, "front_overlap": 0.75, "side_overlap": 0.6},
        "wind": {"speed_ms": 6, "from_deg": 240},
        "time_weight": 1.0,
    },
    # Сильный ветер 11 м/с: Gemini и 801 (предел 10 м/с) исключаются
    "strong_wind": {
        "survey_area": rect(37.62, 55.61, 2.5, 2.0),
        "no_fly_zones": [],
        "bases": [{"id": "A", "lon": 37.60, "lat": 55.595}],
        "reserve_sites": [],
        "drones": [
            {"id": "201-1", "model": "geoscan_201"},
            {"id": "gemini-1", "model": "geoscan_gemini"},
            {"id": "801-1", "model": "geoscan_801"},
            {"id": "401-1", "model": "geoscan_401"},
        ],
        "survey_type": "rgb",
        "requirements": {"gsd_cm": 4},
        "wind": {"speed_ms": 11, "from_deg": 0},
        "time_weight": 1.0,
    },
}

if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for name, sc in SCENARIOS.items():
        (OUT / f"{name}.json").write_text(json.dumps(sc, ensure_ascii=False, indent=1), encoding="utf-8")
        print("written", name)
