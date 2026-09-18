"""Схемы запроса и ответа API. Геометрия — GeoJSON (RFC 7946, WGS84, lon/lat)."""
from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field

from .fleet import SurveyType
from .sensors import SurveyParams, SurveyRequirements
from .wind import Wind


class Site(BaseModel):
    id: str
    name: str = ""
    lon: float
    lat: float


class DroneInstance(BaseModel):
    id: str
    model: str                      # ключ из fleet.yaml
    payload: str | None = None      # если не задано — первая подходящая нагрузка
    base_id: str | None = None      # если не задано — ближайшая к области база


class PlanRequest(BaseModel):
    survey_area: dict[str, Any]                 # GeoJSON Polygon | MultiPolygon
    allowed_area: dict[str, Any] | None = None  # граница разрешённого воздушного пространства
    no_fly_zones: list[dict[str, Any]] = Field(default_factory=list)
    bases: list[Site]
    reserve_sites: list[Site] = Field(default_factory=list)
    drones: list[DroneInstance]
    survey_type: SurveyType = "rgb"
    requirements: SurveyRequirements = Field(default_factory=SurveyRequirements)
    wind: Wind = Field(default_factory=Wind)
    # 1.0 — минимизация времени выполнения работ, 0.0 — минимизация суммарного налёта
    time_weight: float = Field(1.0, ge=0.0, le=1.0)
    reserve: float = Field(0.2, ge=0.0, le=0.6)
    nfz_buffer_m: float = 30.0


class LegOut(BaseModel):
    kind: str
    coordinates: list[list[float]]  # [lon, lat, alt_agl]
    duration_s: float
    distance_m: float


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


class Summary(BaseModel):
    makespan_s: float
    total_flight_s: float
    sorties: int
    area_km2: float
    covered_km2: float
    coverage_pct: float
    drones_used: int


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
