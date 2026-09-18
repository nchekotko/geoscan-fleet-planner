# API

Базовый адрес: `http://127.0.0.1:8000`. Интерактивная документация OpenAPI — `/docs`, схема — `/openapi.json`.
Все координаты — WGS84, порядок `[долгота, широта]` (GeoJSON, RFC 7946). Ключи доступа не нужны.

| Метод | Путь | Назначение |
|---|---|---|
| GET | `/api/health` | Проверка работоспособности |
| GET | `/api/fleet` | Датасет бортов и нагрузок (ТТХ, источники, допущения) |
| GET | `/api/scenarios` | Список тестовых сценариев |
| GET | `/api/scenarios/{name}` | Сценарий в формате запроса на планирование |
| POST | `/api/plan` | Рассчитать план |
| POST | `/api/pareto` | Рассчитать фронт Парето (набор недоминируемых планов) |
| GET | `/api/plan/{plan_id}/export/{drone_id}.geojson` | Полётное задание борта в GeoJSON |
| GET | `/api/plan/{plan_id}/export/{drone_id}.kml` | Полётное задание борта в KML 2.2 |
| GET | `/api/plan/{plan_id}/export.zip` | Все задания плана (GeoJSON + KML) и сводка |

## Запрос на планирование

```json
{
  "survey_area": {"type": "Polygon", "coordinates": [[[37.60, 55.60], [37.64, 55.60], [37.645, 55.615], [37.60, 55.62], [37.60, 55.60]]]},
  "allowed_area": null,
  "no_fly_zones": [{"type": "Polygon", "coordinates": [[[37.615, 55.607], [37.622, 55.607], [37.622, 55.612], [37.615, 55.612], [37.615, 55.607]]]}],
  "bases": [{"id": "A", "name": "ВПП Юг", "lon": 37.605, "lat": 55.595}],
  "reserve_sites": [{"id": "R1", "lon": 37.625, "lat": 55.63}],
  "drones": [
    {"id": "201-1", "model": "geoscan_201", "base_id": "A"},
    {"id": "gemini-1", "model": "geoscan_gemini", "payload": "pf1b"}
  ],
  "survey_type": "rgb",
  "requirements": {"gsd_cm": 3, "front_overlap": 0.75, "side_overlap": 0.65, "altitude_ceiling_m": null},
  "wind": {"speed_ms": 5, "from_deg": 270},
  "time_weight": 1.0,
  "reserve": 0.2,
  "nfz_buffer_m": 30,
  "use_terrain": true
}
```

| Поле | Описание |
|---|---|
| `survey_area` | Область съёмки: Polygon или MultiPolygon |
| `allowed_area` | Граница разрешённого воздушного пространства (необязательно) |
| `no_fly_zones` | Запретные зоны (Polygon) |
| `bases` | Взлётно-посадочные пункты |
| `reserve_sites` | Резервные площадки посадки |
| `drones[].model` | Ключ модели из `/api/fleet`: `geoscan_201`, `geoscan_gemini`, `geoscan_801`, `geoscan_401` |
| `drones[].payload` | Ключ нагрузки; если не задан — первая подходящая под тип съёмки |
| `drones[].base_id` | База борта; если не задана — ближайшая к его участку |
| `survey_type` | `rgb`, `multispectral`, `thermal`, `lidar`, `geophysics` |
| `requirements` | `gsd_cm`, `front_overlap`, `side_overlap`, `altitude_m` (явная высота), `lidar_density_pts_m2`, `line_spacing_m` (геофизика), `altitude_ceiling_m` (потолок AGL) |
| `wind` | Скорость, м/с, и направление, откуда дует, градусы |
| `time_weight` | Критерий: 1 — минимум времени работ, 0 — минимум суммарного налёта, между — компромисс |
| `reserve` | Резерв заряда, доля (0,2 = 20 %) |
| `nfz_buffer_m` | Запас вокруг запретных зон, м |
| `use_terrain` | Учитывать рельеф Copernicus DEM |

## Ответ

```json
{
  "plan_id": "a1b2c3d4e5f6",
  "summary": {"makespan_s": 3522, "total_flight_s": 12120, "sorties": 7, "area_km2": 6.27,
              "covered_km2": 6.27, "coverage_pct": 99.99, "drones_used": 4},
  "drones": [{
    "drone_id": "201-1", "model": "geoscan_201", "model_name": "Геоскан 201", "payload": "sony_rx1rm2",
    "params": {"altitude_agl_m": 233, "line_spacing_m": 83, "speed_ms": 19.4, "swath_m": 238, "gsd_cm": 3.0, "notes": []},
    "sweep_angle_deg": 90.0, "region": {"type": "Polygon", "coordinates": []}, "area_km2": 3.8,
    "sorties": [{
      "index": 0, "base_id": "A", "start_s": 0, "duration_s": 3400, "survey_length_m": 52000,
      "max_divert_s": 230, "divert_site": "A",
      "legs": [{"kind": "survey", "coordinates": [[37.61, 55.60, 233], [37.61, 55.62, 233]],
                "alt_amsl": [430.1, 430.1], "duration_s": 118, "distance_m": 2200}]
    }],
    "flight_time_s": 3400, "finish_s": 3400
  }],
  "excluded": [{"drone_id": "801-1", "reason": "Геоскан 801: ветер 11 м/с выше допустимого 10 м/с"}],
  "warnings": [],
  "working_area": {"type": "Polygon", "coordinates": []},
  "terrain": {"source": "Copernicus DEM GLO-30 (DSM, EGM2008)", "ground_min_m": 151.7, "ground_max_m": 196.7}
}
```

Этапы полёта (`legs[].kind`): `takeoff` — взлёт и набор высоты, `transit` — перелёт, `survey` — галс съёмки, `tie` — секущий маршрут (геофизика), `turn` — разворот, `return` — возврат, `landing` — посадка.
Высоты: третья координата — над землёй (AGL); `alt_amsl` — абсолютная высота над геоидом EGM2008 (если учтён рельеф).

Ошибки: `422` — план невозможен (пустая рабочая область, нет подходящих бортов), текст причины в `detail`; `404` — план не найден (планы хранятся в памяти процесса до перезапуска).

## Формат экспорта

**GeoJSON** (на борт): `FeatureCollection` с
- `region` — участок борта;
- `leg` — участки маршрута (LineString) со свойствами `sortie`, `step`, `phase`, `phase_ru`, `altitude_agl_m`, `speed_ms`, `duration_s`, `distance_m`, `altitude_amsl_m`;
- `waypoint` — ключевые точки (Point) с `seq`, `sortie`, `action` (`takeoff`, `survey_start`, `survey_end`, `land`), `altitude_agl_m`, `altitude_amsl_m`;
- `base`, `reserve_site` — площадки.

Параметры задания (модель, нагрузка, высота, скорость, шаг галсов, GSD, интервал съёмки, вылеты) — во внешнем члене `mission` (RFC 7946, п. 6.1).

**KML 2.2**: `Document` → папка «Площадки», папка «Область борта», папка на каждый вылет с участками (LineString, цвет по этапу) и точками взлёта и посадки. Высоты абсолютные, если учтён рельеф, иначе `relativeToGround`.

## Пример

```bash
curl -X POST http://127.0.0.1:8000/api/plan -H "Content-Type: application/json" \
     --data-binary @backend/data/scenarios/demo_basic.json
```
