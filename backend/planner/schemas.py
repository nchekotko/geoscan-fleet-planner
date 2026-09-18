"""Схемы запроса и ответа API. Геометрия — GeoJSON (RFC 7946, WGS84, lon/lat)."""
from __future__ import annotations

import math
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
from shapely.geometry import MultiPolygon, Polygon, mapping, shape
from shapely.validation import make_valid

from .fleet import SurveyType
from .sensors import SurveyParams, SurveyRequirements
from .wind import Wind


def _polygon_geojson(g: Any, what: str) -> dict[str, Any]:
    """Проверка GeoJSON-полигона: тип Polygon/MultiPolygon, конечные координаты в диапазоне WGS84,
    непустая площадь. Невалидная геометрия (самопересечения) исправляется make_valid."""
    if not isinstance(g, dict) or g.get("type") not in ("Polygon", "MultiPolygon"):
        raise ValueError(f"{what}: нужен GeoJSON Polygon или MultiPolygon")
    try:
        geom = shape(g)
    except Exception as e:  # noqa: BLE001 — shapely бросает разные исключения на битый GeoJSON
        raise ValueError(f"{what}: некорректные координаты GeoJSON ({e})") from None
    for lon, lat, *_ in (c for p in getattr(geom, "geoms", [geom]) for r in [p.exterior, *p.interiors]
                         for c in r.coords):
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
    return mapping(geom)


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
    no_fly_zones: list[dict[str, Any]] = Field(default_factory=list)
    bases: list[Site]
    reserve_sites: list[Site] = Field(default_factory=list)
    drones: list[DroneInstance] = Field(min_length=1)
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
