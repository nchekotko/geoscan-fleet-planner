"""Сценарий на данных заказчика: участок из «Границы полетов.kml», зоны ограничений
из «Московская зона.kml» и высотные препятствия из «obstacles_Московская область.kml».

Запуск (из каталога backend):
    python data/make_moscow_scenario.py
Читает data/geodata/*.geojson (их делает convert_geodata.py) и пишет
data/scenarios/moscow_real.json — самодостаточный сценарий с реальными ограничениями.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from shapely.geometry import Point, shape

HERE = Path(__file__).resolve().parent
GEO = HERE / "geodata"
OUT = HERE / "scenarios" / "moscow_real.json"

TASK_ID = "area-394"      # участок 11 км² под Егорьевском: рядом мачты и две зоны ограничений
OBSTACLE_RADIUS_KM = 4.0  # препятствия в этом радиусе от участка попадают в сценарий


def km_deg(lat: float) -> tuple[float, float]:
    return 1 / (111.32 * math.cos(math.radians(lat))), 1 / 110.57


def main() -> None:
    task = json.loads((GEO / "task_moscow.geojson").read_text(encoding="utf-8"))["features"]
    zones = json.loads((GEO / "zones_moscow.geojson").read_text(encoding="utf-8"))["features"]
    obstacles = json.loads((GEO / "obstacles_moscow.geojson").read_text(encoding="utf-8"))["features"]

    area_f = next(f for f in task if f["properties"]["id"] == TASK_ID)
    area = shape(area_f["geometry"])
    c = area.centroid
    k_lon, k_lat = km_deg(c.y)

    near = area.buffer(OBSTACLE_RADIUS_KM * k_lat)  # грубый радиус в градусах
    obs = [f for f in obstacles if shape(f["geometry"]).intersects(near)]
    zs = [f for f in zones if shape(f["geometry"]).intersects(area.buffer(2 * k_lat))]

    # ВПП — в стороне от участка и не внутри препятствий
    base = (round(c.x - 3 * k_lon, 6), round(c.y - 3.2 * k_lat, 6))
    reserve = (round(c.x + 2.5 * k_lon, 6), round(c.y + 1.5 * k_lat, 6))

    scenario = {
        "_note": "Реальные данные заказчика: задание «Границы полетов.kml», зоны «Московская зона.kml», "
                 "препятствия «obstacles_Московская область.kml»",
        "survey_area": area_f["geometry"],
        "bases": [{"id": "A", "name": "ВПП Егорьевск", "lon": base[0], "lat": base[1]}],
        "reserve_sites": [{"id": "R1", "name": "Резервная площадка", "lon": reserve[0], "lat": reserve[1]}],
        "drones": [
            {"id": "201-1", "model": "geoscan_201"},
            {"id": "201-2", "model": "geoscan_201"},
            {"id": "401-1", "model": "geoscan_401"},
            {"id": "gemini-1", "model": "geoscan_gemini"},
        ],
        "survey_type": "rgb",
        "requirements": {"gsd_cm": 5.0, "front_overlap": 0.7, "side_overlap": 0.6, "altitude_ceiling_m": 150},
        "wind": {"speed_ms": 5, "from_deg": 250, "ref_height_m": 10},
        "time_weight": 1.0,
        "mission_start": "2026-09-29T06:00:00+03:00",
        "mission_window_h": 12,
        "restrictions": zs,
        "obstacles": obs,
        "use_terrain": True,
    }
    OUT.write_text(json.dumps(scenario, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    km2 = area.area * 111.32 * 110.57 * math.cos(math.radians(c.y))
    tall = sum(1 for f in obs if (f["properties"].get("top_m") or 0) > 100)
    print(f"{OUT.name}: участок {km2:.2f} км², зон {len(zs)}, препятствий {len(obs)} (выше 100 м — {tall}), "
          f"{OUT.stat().st_size / 1e3:.0f} КБ")


if __name__ == "__main__":
    main()
