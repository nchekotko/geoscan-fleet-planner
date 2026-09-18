"""Загрузка датасета парка (data/fleet.yaml) в типизированные профили."""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field

DATA_PATH = Path(__file__).resolve().parent.parent / "data" / "fleet.yaml"

SurveyType = Literal["rgb", "multispectral", "thermal", "lidar", "geophysics"]


class Payload(BaseModel):
    id: str
    name: str
    kind: Literal["camera", "lidar", "magnetometer", "gamma"]
    survey_types: list[SurveyType]
    # camera
    sensor_w_mm: float | None = None
    sensor_h_mm: float | None = None
    image_w_px: int | None = None
    image_h_px: int | None = None
    focal_mm: float | None = None
    min_trigger_interval_s: float | None = None
    # lidar
    pulse_rate_hz: float | None = None
    max_range_m: float | None = None
    effective_fov_deg: float | None = None
    # lidar / geophysics
    recommended_alt_m: float | None = None
    recommended_line_spacing_m: float | None = None
    tie_line_factor: int | None = None
    sample_rate_hz: float | None = None
    source: str = ""
    assumed: list[str] = Field(default_factory=list)


class DroneModel(BaseModel):
    id: str
    name: str
    type: Literal["fixed_wing", "multirotor"]
    airspeed_min_ms: float
    airspeed_max_ms: float
    cruise_speed_ms: float
    climb_rate_ms: float
    endurance_min: float
    max_route_km: float | None = None
    battery_wh: float
    max_wind_ms: float
    min_alt_agl_m: float
    max_alt_agl_m: float | None = None
    max_alt_amsl_m: float
    max_bank_deg: float = 30.0
    radio_range_km: float
    takeoff: str
    landing: str
    swap_time_min: float
    payloads: list[str]
    source: str = ""
    assumed: list[str] = Field(default_factory=list)

    @property
    def turn_radius_m(self) -> float:
        """Минимальный радиус разворота самолёта: R = V² / (g·tan φ). У мультиротора 0."""
        if self.type != "fixed_wing":
            return 0.0
        import math

        return self.cruise_speed_ms**2 / (9.81 * math.tan(math.radians(self.max_bank_deg)))


class Fleet(BaseModel):
    sources: dict[str, str]
    payloads: dict[str, Payload]
    drones: dict[str, DroneModel]


@lru_cache
def load_fleet(path: Path = DATA_PATH) -> Fleet:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    payloads = {k: Payload(id=k, **v) for k, v in raw["payloads"].items()}
    drones = {k: DroneModel(id=k, **v) for k, v in raw["drones"].items()}
    for d in drones.values():
        missing = [p for p in d.payloads if p not in payloads]
        if missing:
            raise ValueError(f"{d.id}: неизвестные нагрузки {missing}")
    return Fleet(sources=raw["sources"], payloads=payloads, drones=drones)
