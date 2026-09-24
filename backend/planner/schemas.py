"""Схемы запроса и ответа API. Геометрия — GeoJSON (RFC 7946, WGS84, lon/lat)."""
from __future__ import annotations

import math
from datetime import datetime
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
from shapely.geometry import MultiPolygon, Polygon, mapping, shape
from shapely.validation import make_valid

from .fleet import SurveyType
from .sensors import SurveyParams, SurveyRequirements
from .wind import Wind


# Лимиты запроса: защита сервиса от запросов, которые надолго займут все ядра
MAX_VERTICES = 20_000
MAX_AREA_KM2 = 1_000.0
MAX_DRONES = 40
MAX_NFZ = 200
# зоны ограничений и препятствия приходят выгрузками (в примере заказчика — 341 зона
# и 5162 препятствия на регион), поэтому лимиты выше: фильтр по высотам и области отсеет лишнее
MAX_RESTRICTIONS = 5_000
MAX_OBSTACLES = 100_000
OBSTACLE_GEOMETRIES = ("Polygon", "MultiPolygon", "LineString", "MultiLineString", "Point")


def _polygon_geojson(g: Any, what: str) -> dict[str, Any]:
    """Проверка GeoJSON-полигона: тип Polygon/MultiPolygon, конечные координаты в диапазоне WGS84,
    непустая площадь. Невалидная геометрия (самопересечения) исправляется make_valid."""
    if not isinstance(g, dict) or g.get("type") not in ("Polygon", "MultiPolygon"):
        raise ValueError(f"{what}: нужен GeoJSON Polygon или MultiPolygon")
    try:
        geom = shape(g)
    except Exception as e:  # noqa: BLE001 — shapely бросает разные исключения на битый GeoJSON
        raise ValueError(f"{what}: некорректные координаты GeoJSON ({e})") from None
    coords = [c for p in getattr(geom, "geoms", [geom]) for r in [p.exterior, *p.interiors] for c in r.coords]
    if len(coords) > MAX_VERTICES:
        raise ValueError(f"{what}: слишком много вершин ({len(coords)}, не больше {MAX_VERTICES}) — упростите контур")
    for lon, lat, *_ in coords:
        if not (math.isfinite(lon) and math.isfinite(lat)):
            raise ValueError(f"{what}: координаты должны быть конечными числами")
        if not (-180 <= lon <= 180 and -90 <= lat <= 90):
            raise ValueError(f"{what}: координата ({lon}, {lat}) вне диапазона долгота ±180°, широта ±90°")
    if not geom.is_valid:
        geom = make_valid(geom)
        if not isinstance(geom, (Polygon, MultiPolygon)):
            polys = [x for x in getattr(geom, "geoms", []) if isinstance(x, (Polygon, MultiPolygon))]
            parts = [q for x in polys for q in getattr(x, "geoms", [x])]
            geom = MultiPolygon(parts) if parts else Polygon()
    if geom.is_empty or geom.area <= 0:
        raise ValueError(f"{what}: пустой полигон или полигон нулевой площади")
    # грубая оценка площади в км² (градусы → км на средней широте) — только для лимита
    lat_c = geom.centroid.y
    km2 = geom.area * 111.32 * 110.57 * math.cos(math.radians(lat_c))
    if what == "область съёмки" and km2 > MAX_AREA_KM2:
        raise ValueError(f"{what}: {km2:.0f} км² — больше {MAX_AREA_KM2:.0f} км²; разбейте на части")
    return mapping(geom)


def _feature(f: Any, what: str, types: tuple[str, ...]) -> dict[str, Any]:
    """Проверка Feature нашего формата (или «голой» геометрии): тип геометрии из types,
    координаты в диапазоне WGS84. properties проходят как есть — их разбирает geodata."""
    if not isinstance(f, dict):
        raise ValueError(f"{what}: нужен GeoJSON Feature или геометрия")
    geom = f.get("geometry") if f.get("type") == "Feature" else f
    if not isinstance(geom, dict) or geom.get("type") not in types:
        raise ValueError(f"{what}: геометрия должна быть {' | '.join(types)}")
    try:
        g = shape(geom)
    except Exception as e:  # noqa: BLE001 — shapely бросает разные исключения на битый GeoJSON
        raise ValueError(f"{what}: некорректные координаты GeoJSON ({e})") from None
    if g.is_empty:
        raise ValueError(f"{what}: пустая геометрия")
    minx, miny, maxx, maxy = g.bounds
    if not all(math.isfinite(v) for v in (minx, miny, maxx, maxy)):
        raise ValueError(f"{what}: координаты должны быть конечными числами")
    if not (-180 <= minx and maxx <= 180 and -90 <= miny and maxy <= 90):
        raise ValueError(f"{what}: координаты вне диапазона долгота ±180°, широта ±90°")
    return f if f.get("type") == "Feature" else {"type": "Feature", "geometry": geom, "properties": {}}


class Site(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)

    id: str = Field(min_length=1)
    name: str = ""
    lon: float = Field(ge=-180.0, le=180.0)
    lat: float = Field(ge=-90.0, le=90.0)


class DroneInstance(BaseModel):
    id: str = Field(min_length=1)
    model: str                      # ключ из fleet.yaml
    payload: str | None = None      # если не задано — первая подходящая нагрузка
    base_id: str | None = None      # если не задано — ближайшая к области база


class PlanRequest(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)

    survey_area: dict[str, Any]                 # GeoJSON Polygon | MultiPolygon
    allowed_area: dict[str, Any] | None = None  # граница разрешённого воздушного пространства
    no_fly_zones: list[dict[str, Any]] = Field(default_factory=list, max_length=MAX_NFZ)
    bases: list[Site]
    reserve_sites: list[Site] = Field(default_factory=list)
    drones: list[DroneInstance] = Field(min_length=1, max_length=MAX_DRONES)
    survey_type: SurveyType = "rgb"
    requirements: SurveyRequirements = Field(default_factory=SurveyRequirements)
    wind: Wind = Field(default_factory=Wind)
    # 1.0 — минимизация времени выполнения работ, 0.0 — минимизация суммарного налёта
    time_weight: float = Field(1.0, ge=0.0, le=1.0)
    # ε-ограничение: минимизировать суммарный налёт при времени работ ≤ makespan_cap_s (фронт Парето)
    makespan_cap_s: float | None = Field(None, gt=0.0)
    reserve: float = Field(0.2, ge=0.0, le=0.6)
    nfz_buffer_m: float = Field(30.0, ge=0.0, le=10_000.0)
    use_terrain: bool = True  # рельеф Copernicus DEM GLO-30
    # Зоны ограничений в нашем формате (docs/data_formats.md): с диапазоном высот и временем
    # действия. Мешает только та зона, чья полоса высот пересекает высоты работ и которая
    # действует в окно работ; остальные показываются справочно.
    restrictions: list[dict[str, Any]] = Field(default_factory=list, max_length=MAX_RESTRICTIONS)
    # Высотные препятствия (мачты, трубы, ЛЭП, лес): запретны те, чей верх с зазором выше
    # высоты съёмки — их не перелететь, не потеряв заданный GSD.
    obstacles: list[dict[str, Any]] = Field(default_factory=list, max_length=MAX_OBSTACLES)
    obstacle_clearance_m: float = Field(30.0, ge=0.0, le=1_000.0)  # вертикальный зазор над препятствием
    obstacle_buffer_m: float = Field(50.0, ge=0.0, le=10_000.0)    # горизонтальный обход препятствия
    mission_start: datetime | None = None          # начало работ (для временных ограничений)
    mission_window_h: float = Field(12.0, gt=0.0, le=168.0)  # окно, в котором проверяется действие зон

    @field_validator("restrictions")
    @classmethod
    def _check_restrictions(cls, v: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [_feature(z, f"зона ограничения {i + 1}", ("Polygon", "MultiPolygon")) for i, z in enumerate(v)]

    @field_validator("obstacles")
    @classmethod
    def _check_obstacles(cls, v: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [_feature(o, f"препятствие {i + 1}", OBSTACLE_GEOMETRIES) for i, o in enumerate(v)]

    @field_validator("survey_area")
    @classmethod
    def _check_survey_area(cls, v: dict[str, Any]) -> dict[str, Any]:
        return _polygon_geojson(v, "область съёмки")

    @field_validator("allowed_area")
    @classmethod
    def _check_allowed_area(cls, v: dict[str, Any] | None) -> dict[str, Any] | None:
        return None if v is None else _polygon_geojson(v, "разрешённая зона")

    @field_validator("no_fly_zones")
    @classmethod
    def _check_nfz(cls, v: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [_polygon_geojson(z, f"запретная зона {i + 1}") for i, z in enumerate(v)]

    @model_validator(mode="after")
    def _check_ids(self) -> "PlanRequest":
        def dups(ids: list[str]) -> list[str]:
            return sorted({i for i in ids if ids.count(i) > 1})

        if d := dups([x.id for x in self.drones]):
            raise ValueError(f"повторяющиеся id бортов: {', '.join(d)}")
        if d := dups([x.id for x in self.bases]):
            raise ValueError(f"повторяющиеся id баз: {', '.join(d)}")
        if d := dups([x.id for x in self.reserve_sites]):
            raise ValueError(f"повторяющиеся id резервных площадок: {', '.join(d)}")
        if d := sorted({x.id for x in self.bases} & {x.id for x in self.reserve_sites}):
            raise ValueError(f"id резервных площадок совпадают с id баз: {', '.join(d)}")
        return self


class LegOut(BaseModel):
    kind: str
    coordinates: list[list[float]]  # [lon, lat, alt_agl]
    alt_amsl: list[float] | None = None  # абсолютная высота точек (EGM2008), если есть рельеф
    duration_s: float
    distance_m: float
    # скорость на участке, м/с: съёмочная (survey, tie, turn), транзитная (transit, return),
    # вертикальная — набор высоты (takeoff) и средняя скорость снижения (landing)
    speed_ms: float = 0.0


class SortieOut(BaseModel):
    index: int
    base_id: str
    start_s: float
    duration_s: float
    survey_length_m: float
    max_divert_s: float = 0.0
    divert_site: str = ""
    legs: list[LegOut]


class DronePlanOut(BaseModel):
    drone_id: str
    model: str
    model_name: str
    payload: str
    params: SurveyParams
    sweep_angle_deg: float
    region: dict[str, Any]
    area_km2: float
    sorties: list[SortieOut]
    flight_time_s: float
    finish_s: float
    transit_alt_agl_m: float = 0.0  # эшелон перелётов (взлёт, транзит, возврат)


class TerrainInfo(BaseModel):
    source: str
    ground_min_m: float
    ground_max_m: float


class Summary(BaseModel):
    makespan_s: float
    total_flight_s: float
    sorties: int
    area_km2: float
    covered_km2: float
    coverage_pct: float
    drones_used: int
    # разведение бортов: наименьшее горизонтальное расстояние между бортами на близких высотах
    # (вне окрестности ВПП) и число моментов с конфликтом (шаг 5 с)
    min_separation_m: float | None = None
    separation_pair: list[str] = Field(default_factory=list)
    separation_conflicts: int = 0


class AirspaceInfo(BaseModel):
    """Что из данных о воздушном пространстве попало в расчёт."""

    restrictions_total: int = 0
    restrictions_applied: int = 0
    obstacles_total: int = 0
    obstacles_blocking: int = 0
    # полоса высот работ над землёй, по которой отбирались зоны
    alt_band_agl_m: list[float] = Field(default_factory=list)
    ground_min_m: float | None = None
    ground_max_m: float | None = None


class ExcludedDrone(BaseModel):
    drone_id: str
    reason: str


class PlanResponse(BaseModel):
    summary: Summary
    drones: list[DronePlanOut]
    excluded: list[ExcludedDrone]
    warnings: list[str]
    working_area: dict[str, Any]
    time_weight: float
    reserve_sites: list[Site] = Field(default_factory=list)
    bases: list[Site] = Field(default_factory=list)
    # ограничения из запроса — для экспорта заданий
    no_fly_zones: list[dict[str, Any]] = Field(default_factory=list)
    allowed_area: dict[str, Any] | None = None
    terrain: TerrainInfo | None = None
    # зоны ограничений и препятствия, попавшие в расчёт (Feature нашего формата)
    restrictions: list[dict[str, Any]] = Field(default_factory=list)
    obstacles: list[dict[str, Any]] = Field(default_factory=list)
    airspace: AirspaceInfo | None = None
