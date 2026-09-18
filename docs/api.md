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
| `requirements` | `gsd_cm`, `front_overlap`, `side_overlap` (камеры), `altitude_m` (явная высота), `lidar_density_pts_m2`, `lidar_side_overlap` (перекрытие полос LiDAR, по умолчанию 0,2), `line_spacing_m` (геофизика), `altitude_ceiling_m` (потолок AGL) |
| `wind` | `speed_ms` — скорость, м/с; `from_deg` — откуда дует, градусы; `ref_height_m` — на какой высоте задан ветер (например, 10 м у метеостанции; тогда он пересчитывается на рабочую высоту каждого борта по степенному профилю). Без `ref_height_m` ветер считается заданным на рабочей высоте |
| `time_weight` | Критерий: 1 — минимум времени работ, 0 — минимум суммарного налёта, между — компромисс |
| `reserve` | Резерв заряда, доля (0,2 = 20 %) |
| `nfz_buffer_m` | Запас вокруг запретных зон, м |
| `use_terrain` | Учитывать рельеф Copernicus DEM |

## Ответ

```json
{
  "plan_id": "a1b2c3d4e5f6",
  "summary": {"makespan_s": 3522, "total_flight_s": 12120, "sorties": 7, "area_km2": 6.27,
              "covered_km2": 6.27, "coverage_pct": 99.99, "drones_used": 4,
              "min_separation_m": 275.5, "separation_pair": ["gemini-1", "gemini-2"], "separation_conflicts": 0},
  "drones": [{
    "drone_id": "201-1", "model": "geoscan_201", "model_name": "Геоскан 201", "payload": "sony_rx1rm2",
    "params": {"altitude_agl_m": 233, "line_spacing_m": 83, "speed_ms": 19.4, "swath_m": 238, "gsd_cm": 3.0, "notes": []},
    "sweep_angle_deg": 90.0, "region": {"type": "Polygon", "coordinates": []}, "area_km2": 3.8,
    "sorties": [{
      "index": 0, "base_id": "A", "start_s": 0, "duration_s": 3400, "survey_length_m": 52000,
      "max_divert_s": 230, "divert_site": "A",
      "legs": [{"kind": "survey", "coordinates": [[37.61, 55.60, 233], [37.61, 55.62, 233]],
                "alt_amsl": [430.1, 430.1], "duration_s": 118, "distance_m": 2200, "speed_ms": 19.4}]
    }],
    "flight_time_s": 3400, "finish_s": 3400, "transit_alt_agl_m": 253
  }],
  "excluded": [{"drone_id": "801-1", "reason": "Геоскан 801: ветер 11 м/с выше допустимого 10 м/с"}],
  "warnings": [],
  "working_area": {"type": "Polygon", "coordinates": []},
  "no_fly_zones": [{"type": "Polygon", "coordinates": []}],
  "allowed_area": null,
  "terrain": {"source": "Copernicus DEM GLO-30 (DSM, EGM2008)", "ground_min_m": 151.7, "ground_max_m": 196.7}
}
```

Этапы полёта (`legs[].kind`): `takeoff` — взлёт и набор высоты, `transit` — перелёт, `survey` — галс съёмки, `tie` — секущий маршрут (геофизика), `turn` — разворот, `return` — возврат, `landing` — посадка.
Высоты: третья координата — над землёй (AGL); `alt_amsl` — абсолютная высота над геоидом EGM2008 (если учтён рельеф). Взлёт — две точки над базой: земля → высота полёта; посадка — наоборот: высота полёта → земля (AGL 0).
Скорость участка `speed_ms`, м/с: на `survey`, `tie`, `turn` — скорость съёмки, на `transit`, `return` — транзитная (не ниже крейсерской), на `takeoff` — вертикальная скорость набора, на `landing` — средняя вертикальная скорость снижения (у самолёта — высота / время захода и спуска на парашюте).
`no_fly_zones` и `allowed_area` повторяют ограничения из запроса — они попадают в экспорт заданий.
Разведение бортов: `summary.min_separation_m` — наименьшее горизонтальное расстояние между бортами на близких высотах (разница < 15 м) вне окрестности ВПП, `separation_pair` — какие борта, `separation_conflicts` — число моментов (шаг 5 с) со сближением меньше 50 м; `null`, если борта ни разу не оказываются на одной высоте. `drones[].transit_alt_agl_m` — эшелон перелёта борта (к области и обратно), `sorties[].start_s` учитывает очередь стартов на ВПП.
Взлёт и посадка Геоскан 201 при ветре: `takeoff` — две точки (ВПП на земле → конец разгона против ветра), `landing` — три (начало захода на эшелоне → раскрытие парашюта на 100 м с наветренной стороны → ВПП на земле).

Ошибки: `422` — план невозможен (пустая рабочая область, нет подходящих бортов), текст причины в `detail`; `404` — план не найден (планы хранятся в памяти процесса до перезапуска).

## Формат экспорта

**GeoJSON** (на борт): `FeatureCollection` с
- `region` — участок борта;
- `leg` — участки маршрута (LineString) в порядке выполнения, включая вертикальные взлёт и посадку, со свойствами `sortie`, `step`, `phase`, `phase_ru`, `altitude_agl_m`, `speed_ms` (скорость этого участка), `duration_s`, `distance_m`, `altitude_amsl_m`;
- `waypoint` — ключевые точки (Point): все вершины участков по порядку — взлёт на земле, конец набора высоты, вершины перелётов и возврата (в том числе обходы запретных зон), точки разворотов, начала и концы галсов, заход на посадку и посадка на земле. Совпадающие точки соседних участков сливаются в одну. Вершины разворотов и галсов с огибанием рельефа прорежены (Дуглас — Пейкер, отклонение ≤ 5 м в плане и ≤ 2 м по высоте; если прореженная дуга задела бы запретную зону, берутся все вершины). Прямые отрезки между соседними точками не пересекают запретные зоны. Свойства:
  - `seq` — сквозной номер, `sortie` — номер вылета;
  - `phase` — этап (`legs[].kind`) отрезка, который начинается в этой точке;
  - `action` — `takeoff`, `climb` (набор высоты завершён), `waypoint`, `turn`, `survey_start`, `survey_end` (галс или секущий маршрут), `approach` (заход на посадку), `parachute` (раскрытие парашюта, самолёт), `land`;
  - `altitude_agl_m`, `altitude_amsl_m` (если учтён рельеф);
  - `speed_ms` — скорость на следующем отрезке (у точки посадки 0);
  - `eta_s` — время от начала вылета, с (время начала вылета — в `mission.sorties[].start_s`);
- `base`, `reserve_site` — площадки;
- `no_fly_zone` (с `index`), `allowed_area` — ограничения из запроса (Polygon).

Параметры задания (модель, нагрузка, высота, скорость съёмки, шаг галсов, GSD, интервал съёмки, вылеты) — во внешнем члене `mission` (RFC 7946, п. 6.1).

**KML 2.2**: `Document` → папка «Площадки», папка «Ограничения» (запретные зоны и граница разрешённой зоны), папка «Область борта», папка на каждый вылет с участками (LineString, цвет по этапу, в `ExtendedData` — `phase`, `speed_ms`, `duration_s`, `distance_m`) и вложенной папкой «Ключевые точки» (Point с именем «номер. действие», в `ExtendedData` — `phase`, `action`, `speed_ms`, `altitude_agl_m`, `altitude_amsl_m`, `eta_s`). Высоты абсолютные, если учтён рельеф, иначе `relativeToGround`.

## Пример

```bash
curl -X POST http://127.0.0.1:8000/api/plan -H "Content-Type: application/json" \
     --data-binary @backend/data/scenarios/demo_basic.json
```
